// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "fiss_planner/common/math.hpp"
#include "fiss_planner/planning/components.hpp"
#include "test_utils.hpp"

using namespace fiss_planner;

TEST(Math, LinspaceIncludesEndpointsExactly) {
  const auto v = linspace(3.0, 5.0, 7);
  ASSERT_EQ(v.size(), 7u);
  EXPECT_EQ(v.front(), 3.0);
  EXPECT_EQ(v.back(), 5.0);
}

// Sampling includes both end points exactly and supports a single sample.
TEST(Sampling, IncludesEndpointsAndHandlesSingleSample) {
  EXPECT_EQ(timeSamples(5.0, 0.1).size(), 51u);
  EXPECT_NEAR(timeSamples(5.0, 0.1).back(), 5.0, 1e-12);
  EXPECT_EQ(linspace(-1.0, 1.0, 1), std::vector<double>{0.0});

  SamplingConfig cfg;
  cfg.num_width = 1;
  const auto corridor = Corridor::uniform(0.0, 100.0, -1.75, 1.75);
  const EndStateGrid grid(cfg, corridor, VehicleParams{}, 10.0, 10.0);
  EXPECT_EQ(grid.dims()[0], 1);
  EXPECT_NEAR(grid.lateral()[0].d, 0.0, 1e-12);
}

TEST(EndStateGrid, SamplesEveryAllowedLane) {
  SamplingConfig cfg;
  cfg.num_width = 3;
  auto corridor = Corridor::parallelLanes(0.0, 200.0, 3.5, 1, 1);
  corridor.lanes()[0].change_allowed = false;  // right lane (offset -1) behind a solid line
  const EndStateGrid grid(cfg, corridor, VehicleParams{}, 50.0, 12.0);
  EXPECT_EQ(grid.dims()[0], 6);  // ego + left lane only
  for (size_t i = 1; i < grid.lateral().size(); ++i) {
    EXPECT_LT(grid.lateral()[i - 1].d, grid.lateral()[i].d);
  }
  EXPECT_EQ(grid.lateral().front().lane, 0);
  EXPECT_EQ(grid.lateral().back().lane, 1);
  EXPECT_EQ(grid.lateralInfo(3.4).lane, 1);
  EXPECT_EQ(grid.lateralInfo(-0.2).lane, 0);
  EXPECT_NEAR(grid.speeds().back(), 12.0, 1e-12);
}

TEST(GenerateFrenet, MatchesStartAndEndStates) {
  FrenetState start{5.0, 10.0, 0.5, 0.3, -0.1, 0.0};
  const EndState end{1.5, 12.0, 4.0};
  Trajectory traj;
  generateFrenet(start, end, 0.1, traj);
  ASSERT_EQ(traj.size(), 41u);
  EXPECT_NEAR(traj.s.front(), 5.0, 1e-12);
  EXPECT_NEAR(traj.s_dd.front(), 0.5, 1e-12);
  EXPECT_NEAR(traj.d.back(), 1.5, 1e-9);
  EXPECT_NEAR(traj.s_d.back(), 12.0, 1e-9);
  EXPECT_NEAR(traj.d_d.back(), 0.0, 1e-9);
}

TEST(CostEvaluator, JerkCostIsMeanSquaredJerk) {
  FrenetState start{0.0, 10.0, 0.0, 0.0, 0.0, 0.0};
  Trajectory traj;
  generateFrenet(start, EndState{2.0, 10.0, 3.0}, 0.1, traj);
  CostConfig w;
  w.w_time = w.w_speed = w.w_accel = w.w_lateral = w.w_lane_change = 0.0;
  w.w_jerk = 1.0;
  const CostEvaluator cost(w, SamplingConfig{});
  double expected = 0.0;
  for (size_t i = 0; i < traj.size(); ++i) {
    expected += traj.s_ddd[i] * traj.s_ddd[i] + traj.d_ddd[i] * traj.d_ddd[i];
  }
  expected /= static_cast<double>(traj.size());
  const LateralSample lat{2.0, 0, 0.0, 0.0};
  EXPECT_NEAR(cost.evaluate(traj, lat, 10.0).jerk, expected, 1e-12);

  // A more aggressive manoeuvre must cost more.
  Trajectory fast;
  generateFrenet(start, EndState{2.0, 10.0, 1.5}, 0.1, fast);
  EXPECT_GT(cost.evaluate(fast, lat, 10.0).jerk, cost.evaluate(traj, lat, 10.0).jerk);
}

TEST(ConvertToCartesian, EmptyTrajectory) {
  const auto ref = test::straightLine(50.0);
  Trajectory empty;
  EXPECT_TRUE(convertToCartesian(ref, empty));
  EXPECT_TRUE(empty.x.empty());
}

namespace {
Trajectory straightTrajectory(const ReferenceLine& ref, double v, double d = 0.0) {
  Trajectory traj;
  generateFrenet(FrenetState{0.0, v, 0.0, d, 0.0, 0.0}, EndState{d, v, 4.0}, 0.1, traj);
  convertToCartesian(ref, traj);
  return traj;
}
}  // namespace

// Zero coordinates are valid (e.g. a reference line through the origin).
TEST(ConstraintValidator, AcceptsPointsAtTheOrigin) {
  const auto ref = test::straightLine(200.0);
  const auto traj = straightTrajectory(ref, 10.0);
  EXPECT_EQ(traj.y.front(), 0.0);
  const ConstraintValidator validator(ConstraintConfig{}, VehicleParams{});
  EXPECT_EQ(validator.check(traj, nullptr), Rejection::kOk);
}

TEST(ConstraintValidator, RejectsLimitViolations) {
  const auto ref = test::straightLine(400.0);
  VehicleParams vehicle;
  vehicle.max_speed = 15.0;
  const ConstraintValidator validator(ConstraintConfig{}, vehicle);
  EXPECT_EQ(validator.check(straightTrajectory(ref, 20.0), nullptr), Rejection::kSpeed);

  // Hard braking from 15 m/s to 0 within 1 s.
  Trajectory brake;
  generateFrenet(FrenetState{0.0, 15.0, 0.0, 0.0, 0.0, 0.0}, EndState{0.0, 0.0, 1.0}, 0.1, brake);
  convertToCartesian(ref, brake);
  EXPECT_EQ(validator.check(brake, nullptr), Rejection::kAccel);

  // Sharp lateral jump at speed exceeds curvature / lateral acceleration.
  Trajectory swerve;
  generateFrenet(FrenetState{0.0, 12.0, 0.0, 0.0, 0.0, 0.0}, EndState{3.5, 12.0, 1.0}, 0.1, swerve);
  convertToCartesian(ref, swerve);
  const auto r = validator.check(swerve, nullptr);
  EXPECT_TRUE(r == Rejection::kCurvature || r == Rejection::kLatAccel) << static_cast<int>(r);
}

TEST(ConstraintValidator, RoadBounds) {
  const auto ref = test::straightLine(200.0);
  const auto corridor = Corridor::uniform(0.0, 200.0, -1.75, 1.75);
  const ConstraintValidator validator(ConstraintConfig{}, VehicleParams{});
  EXPECT_EQ(validator.check(straightTrajectory(ref, 10.0), &corridor), Rejection::kOk);

  Trajectory drift;
  generateFrenet(FrenetState{0.0, 10.0, 0.0, 0.0, 0.0, 0.0}, EndState{2.5, 10.0, 4.0}, 0.1, drift);
  convertToCartesian(ref, drift);
  EXPECT_EQ(validator.check(drift, &corridor), Rejection::kOffRoad);

  // Starting partly outside is tolerated as long as it does not get worse.
  Trajectory recover;
  generateFrenet(FrenetState{0.0, 10.0, 0.0, 1.2, 0.0, 0.0}, EndState{0.0, 10.0, 4.0}, 0.1, recover);
  convertToCartesian(ref, recover);
  EXPECT_EQ(validator.check(recover, &corridor), Rejection::kOk);
}

TEST(CollisionChecker, DetectsLeadVehicleAndRespectsTiming) {
  const auto ref = test::straightLine(300.0);
  const auto traj = straightTrajectory(ref, 10.0);
  const CollisionChecker checker(CollisionConfig{}, VehicleParams{});
  int checks = 0;

  // Stopped car 20 m ahead in the same lane: we reach it after ~1.5 s.
  std::vector<Obstacle> stopped = {predictConstantVelocity(1, 20.0, 0.0, 0.0, 0.0, 4.5, 1.8, 5.0, 0.1)};
  EXPECT_FALSE(checker.isCollisionFree(traj, stopped, checks));

  // Same car moving away faster than us: no collision.
  std::vector<Obstacle> leaving = {predictConstantVelocity(1, 20.0, 0.0, 0.0, 15.0, 4.5, 1.8, 5.0, 0.1)};
  EXPECT_TRUE(checker.isCollisionFree(traj, leaving, checks));

  // Car in the adjacent lane.
  std::vector<Obstacle> adjacent = {predictConstantVelocity(1, 20.0, 3.5, 0.0, 0.0, 4.5, 1.8, 5.0, 0.1)};
  EXPECT_TRUE(checker.isCollisionFree(traj, adjacent, checks));
  EXPECT_GT(checks, 0);
}
