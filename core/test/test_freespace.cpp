// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include <gtest/gtest.h>

#include <cmath>

#include "fiss_planner/freespace/hybrid_astar.hpp"

using namespace fiss_planner;

namespace {

std::vector<Vec2> rect(double x0, double y0, double x1, double y1) {
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

VehicleParams car() {
  VehicleParams v;
  v.length = 4.8;
  v.width = 2.0;
  v.wheelbase = 2.9;
  v.rear_axle_to_center = 1.45;
  v.max_steer = 0.6;
  return v;
}

// A 7 m two-lane road along x with a 2.5 m parking strip along its right
// (y in [-2.5, 0]); parked cars at x = 0 and x = `ahead` leave a pocket between.
OccupancyGrid parkingStreet(double ahead) {
  OccupancyGrid grid(10.0, 0.0, 50.0, 0.1);
  grid.markFree(rect(-20.0, 0.0, 40.0, 7.0));
  grid.markFree(rect(-20.0, -2.5, 40.0, 0.0));
  grid.markOccupied({0.0, -1.25, 0.0, 4.6, 1.9});
  grid.markOccupied({ahead, -1.25, 0.0, 4.6, 1.9});
  grid.computeClearance();
  return grid;
}

}  // namespace

TEST(OccupancyGrid, FreesPolygonsAndMeasuresClearance) {
  OccupancyGrid grid(0.0, 0.0, 20.0, 0.1);
  EXPECT_EQ(grid.size(), 200);
  grid.markFree(rect(-5.0, -2.0, 5.0, 2.0));
  grid.markOccupied({3.0, 0.0, 0.0, 1.0, 1.0});
  grid.computeClearance();
  EXPECT_NEAR(grid.clearance(0.0, 0.0), 2.0, 0.1);   // to the long edges
  EXPECT_NEAR(grid.clearance(1.0, 0.0), 1.5, 0.1);   // to the box at x = 2.5
  EXPECT_DOUBLE_EQ(grid.clearance(3.0, 0.0), 0.0);   // inside the box
  EXPECT_DOUBLE_EQ(grid.clearance(0.0, 3.0), 0.0);   // outside the free area
  EXPECT_DOUBLE_EQ(grid.clearance(20.0, 0.0), 0.0);  // outside the grid
}

TEST(Dubins, ReachesTheGoalPose) {
  const double r = 5.0;
  const std::vector<std::pair<FreeSpacePose, FreeSpacePose>> cases{
      {{0, 0, 0}, {20, 0, 0}},       {{0, 0, 0}, {10, 10, M_PI / 2}}, {{0, 0, 0}, {-5, 3, M_PI}},
      {{1, 2, 0.3}, {4, -6, -2.0}},  {{0, 0, 0}, {2, 0, M_PI}},       {{0, 0, M_PI / 2}, {0, 1, -M_PI / 2}},
      {{3, -1, -2.5}, {-8, 4, 1.0}},
  };
  for (const auto& [from, to] : cases) {
    const auto path = dubinsPath(from, to, r, 0.2);
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.back().x, to.x, 1e-6);
    EXPECT_NEAR(path.back().y, to.y, 1e-6);
    EXPECT_NEAR(std::remainder(path.back().yaw - to.yaw, 2 * M_PI), 0.0, 1e-6);
    for (size_t i = 1; i < path.size(); ++i) {
      const double ds = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      EXPECT_LE(ds, 0.2 + 1e-9);
      EXPECT_FALSE(path[i].reverse);
    }
  }
  // Straight ahead is a straight line.
  const auto line = dubinsPath({0, 0, 0}, {20, 0, 0}, r, 0.2);
  for (const auto& p : line) EXPECT_NEAR(p.y, 0.0, 1e-9);
}

TEST(HybridAStar, DrivesForwardInTheOpen) {
  OccupancyGrid grid(10.0, 0.0, 50.0, 0.2);
  grid.markFree(rect(-15.0, -10.0, 35.0, 10.0));
  grid.computeClearance();
  FreeSpaceConfig config;
  const auto path = planHybridAStar(grid, {0, 0, 0}, {15, 3, 0}, car(), config);
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->back().x, 15.0, 0.5);
  EXPECT_NEAR(path->back().y, 3.0, 0.5);
  for (const auto& p : *path) EXPECT_FALSE(p.reverse);
}

TEST(HybridAStar, LeavesAParkingPocket) {
  // The ego is parked between two cars with 0.8 m to spare behind and 1.3 m
  // ahead, 0.25 m from the curb. It must reach the lane centre 12 m ahead.
  const OccupancyGrid grid = parkingStreet(12.0);
  const VehicleParams v = car();
  const FreeSpacePose start{5.5, -1.25, 0.0};
  const FreeSpacePose goal{17.0, 1.75, 0.0};
  FreeSpaceConfig config;
  config.margin = 0.1;
  ASSERT_TRUE(freeSpaceCollisionFree(grid, start, v, config.margin));
  const auto path = planHybridAStar(grid, start, goal, v, config);
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path->back().x, goal.x, 0.5);
  EXPECT_NEAR(path->back().y, goal.y, 0.5);
  bool reversed = false;
  for (const auto& p : *path) {
    EXPECT_TRUE(freeSpaceCollisionFree(grid, p, v, config.margin));
    reversed |= p.reverse;
  }
  EXPECT_TRUE(reversed);

  // Without reversing it cannot pull out of a pocket this tight.
  config.reverse = false;
  config.max_expansions = 5000;
  EXPECT_FALSE(planHybridAStar(grid, start, goal, v, config).has_value());
}

TEST(HybridAStar, FailsWhenTheGoalIsBlocked) {
  OccupancyGrid grid(0.0, 0.0, 30.0, 0.2);
  grid.markFree(rect(-15.0, -3.0, 15.0, 3.0));
  grid.markOccupied({10.0, 0.0, 0.0, 4.0, 2.0});
  grid.computeClearance();
  EXPECT_FALSE(planHybridAStar(grid, {0, 0, 0}, {10, 0, 0}, car(), {}).has_value());
}

TEST(FreeSpaceTrajectory, StopsAtEveryChangeOfDirection) {
  // 3 m forward, then 3 m back.
  std::vector<FreeSpacePose> path;
  for (int i = 0; i <= 15; ++i) path.push_back({0.2 * i, 0.0, 0.0, false});
  for (int i = 14; i >= 0; --i) path.push_back({0.2 * i, 0.0, 0.0, true});
  FreeSpaceConfig config;
  config.speed = 1.5;
  config.accel = 1.0;
  const Trajectory traj = freeSpaceTrajectory(path, 0.0, config, 0.1);
  ASSERT_GT(traj.size(), 10u);
  EXPECT_NEAR(traj.v.front(), 0.0, 1e-9);
  EXPECT_NEAR(traj.v.back(), 0.0, 0.15);
  EXPECT_NEAR(traj.x.back(), 0.0, 0.05);
  bool forward = false, backward = false;
  double x_max = 0.0;
  for (size_t i = 0; i < traj.size(); ++i) {
    EXPECT_LE(std::abs(traj.v[i]), config.speed + 1e-9);
    forward |= traj.v[i] > 0.5;
    backward |= traj.v[i] < -0.5;
    if (backward) EXPECT_LE(traj.v[i], 1e-9);  // never forward again
    x_max = std::max(x_max, traj.x[i]);
    EXPECT_DOUBLE_EQ(traj.yaw[i], 0.0);
  }
  EXPECT_TRUE(forward);
  EXPECT_TRUE(backward);
  EXPECT_NEAR(x_max, 3.0, 0.05);
}
