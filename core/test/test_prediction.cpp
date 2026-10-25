// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "fiss_planner/common/math.hpp"
#include "fiss_planner/prediction/lane_following.hpp"

using namespace fiss_planner;

namespace {

MapLane laneFrom(int id, const std::vector<Vec2>& center, double width = 3.5) {
  MapLane l;
  l.id = id;
  l.centerline = center;
  for (size_t i = 0; i < center.size(); ++i) {
    const size_t a = i == 0 ? 0 : i - 1, b = i == 0 ? 1 : i;
    const double h = std::atan2(center[b].y - center[a].y, center[b].x - center[a].x);
    l.left.push_back({center[i].x - 0.5 * width * std::sin(h), center[i].y + 0.5 * width * std::cos(h)});
    l.right.push_back({center[i].x + 0.5 * width * std::sin(h), center[i].y - 0.5 * width * std::cos(h)});
  }
  return l;
}

std::vector<Vec2> straight(Vec2 from, double heading, double length) {
  std::vector<Vec2> pts;
  for (double s = 0.0; s <= length + 1e-9; s += 2.0) {
    pts.push_back({from.x + s * std::cos(heading), from.y + s * std::sin(heading)});
  }
  return pts;
}

/// Left-turning quarter circle of radius R starting at `from` heading +x.
std::vector<Vec2> quarterLeft(Vec2 from, double R) {
  std::vector<Vec2> pts;
  for (int i = 0; i <= 40; ++i) {
    const double a = 0.5 * kPi * i / 40.0;
    pts.push_back({from.x + R * std::sin(a), from.y + R * (1.0 - std::cos(a))});
  }
  return pts;
}

/// 50 m straight (1) forking into a left curve (2) and a straight lane (3).
RoadMap forkedRoad() {
  RoadMap map;
  auto a = laneFrom(1, straight({0.0, 0.0}, 0.0, 50.0));
  a.successors = {2, 3};
  map.addLane(a);
  map.addLane(laneFrom(2, quarterLeft({50.0, 0.0}, 30.0)));
  map.addLane(laneFrom(3, straight({50.0, 0.0}, 0.0, 100.0)));
  map.finalize();
  return map;
}

/// 20 m straight (1) followed by a left curve (2) only.
RoadMap bend() {
  RoadMap map;
  auto a = laneFrom(1, straight({0.0, 0.0}, 0.0, 20.0));
  a.successors = {2};
  map.addLane(a);
  map.addLane(laneFrom(2, quarterLeft({20.0, 0.0}, 25.0)));
  map.finalize();
  return map;
}

}  // namespace

TEST(LaneFollowing, FollowsACurveIntoTheSuccessorLane) {
  const RoadMap map = bend();
  const Obstacle ob = predictLaneFollowing(map, 5, 10.0, 0.0, 0.0, 10.0, 4.5, 1.9, 3.0, 0.1);
  ASSERT_EQ(ob.prediction.size(), 31u);
  // After 3 s it has driven 30 m: 10 m of straight, then 20 m along the R = 25 arc.
  const double a = 20.0 / 25.0;
  const auto& last = ob.prediction.back();
  EXPECT_NEAR(last.x, 20.0 + 25.0 * std::sin(a), 0.3);
  EXPECT_NEAR(last.y, 25.0 * (1.0 - std::cos(a)), 0.3);
  EXPECT_NEAR(normalizeAngle(last.yaw - a), 0.0, 0.1);
  EXPECT_NEAR(last.t, 3.0, 1e-9);
  // A constant-velocity prediction would have left the lane by 8 m or more.
  EXPECT_GT(25.0 * (1.0 - std::cos(a)), 7.0);
}

TEST(LaneFollowing, TakesTheStraightestBranchAndDecaysTheOffset) {
  const RoadMap map = forkedRoad();
  const Obstacle ob = predictLaneFollowing(map, 5, 30.0, 0.8, 0.0, 10.0, 4.5, 1.9, 4.0, 0.1);
  const auto& last = ob.prediction.back();
  EXPECT_NEAR(last.x, 70.0, 0.5);  // 40 m on, along the straight branch
  EXPECT_NEAR(last.y, 0.8 * std::exp(-4.0 / 1.5), 0.05);
  EXPECT_NEAR(ob.prediction.front().y, 0.8, 1e-9);
}

TEST(LaneFollowing, ContinuesStraightPastADeadEnd) {
  RoadMap map;
  map.addLane(laneFrom(1, straight({0.0, 0.0}, 0.0, 20.0)));
  map.finalize();
  const Obstacle ob = predictLaneFollowing(map, 5, 10.0, 0.0, 0.0, 10.0, 4.5, 1.9, 2.0, 0.1);
  EXPECT_NEAR(ob.prediction.back().x, 30.0, 0.1);
  EXPECT_NEAR(ob.prediction.back().y, 0.0, 1e-6);
}

TEST(LaneFollowing, FallsBackToConstantVelocity) {
  const RoadMap map = bend();
  const auto is_cv = [](const Obstacle& ob, double x, double y, double yaw, double v) {
    const auto& last = ob.prediction.back();
    return std::abs(last.x - (x + v * last.t * std::cos(yaw))) < 1e-6 &&
           std::abs(last.y - (y + v * last.t * std::sin(yaw))) < 1e-6;
  };
  // Crossing the lane at 90 degrees.
  EXPECT_TRUE(is_cv(predictLaneFollowing(map, 1, 10.0, 0.0, kPi / 2, 5.0, 4.5, 1.9, 2.0, 0.1), 10.0, 0.0,
                    kPi / 2, 5.0));
  // Off the map.
  EXPECT_TRUE(
      is_cv(predictLaneFollowing(map, 1, 10.0, 30.0, 0.0, 5.0, 4.5, 1.9, 2.0, 0.1), 10.0, 30.0, 0.0, 5.0));
  // Nearly stopped.
  EXPECT_TRUE(
      is_cv(predictLaneFollowing(map, 1, 10.0, 0.0, 0.0, 0.2, 4.5, 1.9, 2.0, 0.1), 10.0, 0.0, 0.0, 0.2));
}

TEST(CompletePredictions, ExtendsOnlySingleStateMovingObstacles) {
  const RoadMap map = bend();
  Obstacle tracked;  // perception output: current state only
  tracked.id = 1;
  tracked.length = 4.5;
  tracked.width = 1.9;
  tracked.prediction = {{-0.2, 10.0, 0.0, 0.0, 10.0}};
  Obstacle parked = tracked;
  parked.id = 2;
  parked.is_static = true;
  Obstacle predicted = predictConstantVelocity(3, 0.0, 0.0, 0.0, 5.0, 4.5, 1.9, 2.0, 0.1);
  std::vector<Obstacle> obs = {tracked, parked, predicted};

  auto as_given = obs;
  completePredictions(as_given, PredictionMode::kAsGiven, &map, 3.0, 0.1);
  EXPECT_EQ(as_given[0].prediction.size(), 1u);

  completePredictions(obs, predictionModeFromString("lane_following"), &map, 3.0, 0.1);
  ASSERT_EQ(obs[0].prediction.size(), 31u);
  EXPECT_NEAR(obs[0].prediction.front().t, -0.2, 1e-9);  // still relative to the observation
  EXPECT_GT(obs[0].prediction.back().y, 7.0);            // followed the bend
  EXPECT_EQ(obs[1].prediction.size(), 1u);               // static: unchanged
  EXPECT_EQ(obs[2].prediction.size(), predicted.prediction.size());
  EXPECT_THROW(predictionModeFromString("kalman"), std::invalid_argument);
}
