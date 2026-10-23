// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "frenet_planner/control/controller.hpp"
#include "frenet_planner/planning/local_planner.hpp"
#include "frenet_planner/smoothing/piecewise_jerk.hpp"
#include "frenet_planner/smoothing/smoother.hpp"
#include "test_utils.hpp"

using namespace frenet_planner;

TEST(KinematicBicycle, MovesAlongItsHeading) {
  KinematicBicycle car(VehicleParams{}, {3.0, -2.0, kPi / 2, 5.0, 0.0, 0.0});
  for (int i = 0; i < 10; ++i) car.step({0.0, 0.0}, 0.1);
  EXPECT_NEAR(car.state().x, 3.0, 1e-9);
  EXPECT_NEAR(car.state().y, 3.0, 1e-9);
}

TEST(KinematicBicycle, SteadyStateCircleAndRateLimit) {
  VehicleParams v;
  const double delta = 0.2;
  KinematicBicycle car(v, {0.0, 0.0, 0.0, 5.0, 0.0, 0.0}, delta);
  double max_r = 0.0, min_r = 1e9;
  const double R_rear = v.wheelbase / std::tan(delta);
  const double R_center = std::hypot(R_rear, v.rear_axle_to_center);
  // Circle center from the rear axle: (−b, R_rear).
  for (int i = 0; i < 200; ++i) {
    car.step({delta, 0.0}, 0.05);
    const double r = std::hypot(car.state().x + v.rear_axle_to_center, car.state().y - R_rear);
    max_r = std::max(max_r, r);
    min_r = std::min(min_r, r);
  }
  EXPECT_NEAR(max_r, R_center, 1e-3);
  EXPECT_NEAR(min_r, R_center, 1e-3);

  KinematicBicycle slow(v, {0.0, 0.0, 0.0, 5.0, 0.0, 0.0});
  slow.step({0.5, 0.0}, 0.1);
  EXPECT_NEAR(slow.steer(), v.max_steer_rate * 0.1, 1e-12);
}

TEST(PiecewiseJerk, SolvesOrReportsUnavailable) {
  PiecewiseJerkProblem p;
  p.dt = 0.1;
  p.x0 = {0.0, 0.0, 0.0};
  p.x_ref.assign(40, 2.0);
  p.x_hi.assign(40, 1.5);  // cannot reach the reference: bounded at 1.5
  p.ddx_lo = -3.0;
  p.ddx_hi = 3.0;
  p.w_x = 1.0;
  p.w_ddx = 0.1;
  p.w_dddx = 0.1;
  const auto sol = solvePiecewiseJerk(p);
  if (!hasOptimizers()) {
    EXPECT_FALSE(sol);
    GTEST_SKIP() << "built without OSQP";
  }
  ASSERT_TRUE(sol);
  EXPECT_NEAR(sol->x.front(), 0.0, 1e-6);
  EXPECT_NEAR(sol->x.back(), 1.5, 0.05);
  for (size_t i = 0; i < sol->x.size(); ++i) {
    EXPECT_LE(sol->x[i], 1.5 + 1e-3);
    EXPECT_LE(sol->ddx[i], 3.0 + 1e-3);
    EXPECT_GE(sol->ddx[i], -3.0 - 1e-3);
  }
  // Kinematic consistency between knots.
  for (size_t i = 0; i + 1 < sol->x.size(); ++i) {
    const double dt = p.dt;
    EXPECT_NEAR(sol->x[i + 1],
                sol->x[i] + sol->dx[i] * dt + sol->ddx[i] * dt * dt / 3 + sol->ddx[i + 1] * dt * dt / 6,
                1e-4);
  }
}

namespace {
Obstacle staticBox(int id, double x, double y, double l = 4.5, double w = 1.9) {
  return predictConstantVelocity(id, x, y, 0.0, 0.0, l, w, 6.0, 0.1);
}
}  // namespace

TEST(Smoother, SpaceTimeCorridorKeepsTheSampledSide) {
  const auto ref = test::straightLine(300.0);
  const auto corridor = Corridor::parallelLanes(0.0, 300.0, 3.5, 1, 0);
  Trajectory traj;
  generateFrenet(FrenetState{0.0, 10.0, 0.0, 0.0, 0.0, 0.0}, EndState{3.5, 10.0, 5.0}, 0.1, traj);
  convertToCartesian(ref, traj);
  const std::vector<Obstacle> obs = {staticBox(1, 35.0, 0.0)};
  const auto c = buildSpaceTimeCorridor(traj, ref, corridor, obs, VehicleParams{}, 0.3, 1.0);
  // While passing the parked car (s ~ 35) the lower bound is to its left.
  bool constrained = false;
  for (size_t i = 0; i < traj.size(); ++i) {
    EXPECT_LE(c.d_lo[i], traj.d[i] + 1e-9);
    EXPECT_GE(c.d_hi[i], traj.d[i] - 1e-9);
    if (std::abs(traj.s[i] - 35.0) < 2.0 && traj.d[i] > 2.5) {
      EXPECT_GT(c.d_lo[i], 0.95 + 0.9 + 0.3 - 1e-6);
      constrained = true;
    }
  }
  EXPECT_TRUE(constrained);
}

TEST(Smoother, RefinesTrajectoryAndKeepsItValid) {
  if (!hasOptimizers()) GTEST_SKIP() << "built without OSQP";
  const auto ref = test::straightLine(300.0);
  const auto corridor = Corridor::parallelLanes(0.0, 300.0, 3.5, 1, 0);
  const std::vector<Obstacle> obs = {staticBox(1, 45.0, 0.0)};
  PlannerConfig cfg;
  cfg.type = PlannerType::kFop;
  auto planner = makePlanner(cfg, VehicleParams{});
  PlanningInput in;
  in.start = {10.0, 10.0, 0.0, 0.0, 0.0, 0.0};
  in.reference = &ref;
  in.corridor = &corridor;
  in.obstacles = &obs;
  in.target_speed = 12.0;
  auto res = planner->plan(in);
  ASSERT_TRUE(res.success);
  Trajectory smooth = res.trajectory;
  ASSERT_TRUE(smoothTrajectory(smooth, in, cfg, VehicleParams{}, SmootherConfig{}));
  const auto jerk = [](const Trajectory& t) {
    double j = 0.0;
    for (size_t i = 0; i < t.size(); ++i) j += t.s_ddd[i] * t.s_ddd[i] + t.d_ddd[i] * t.d_ddd[i];
    return j / static_cast<double>(t.size());
  };
  EXPECT_LT(jerk(smooth), jerk(res.trajectory));
  EXPECT_NEAR(smooth.d.front(), res.trajectory.d.front(), 1e-6);
  EXPECT_NEAR(smooth.s_d.front(), res.trajectory.s_d.front(), 1e-6);
  int checks = 0;
  CollisionConfig raw;
  raw.margin_lat = raw.margin_lon = 0.0;
  EXPECT_TRUE(CollisionChecker(raw, VehicleParams{}).isCollisionFree(smooth, obs, checks));
}

namespace {

/// Track a planned lane change + curve with each controller from an offset start.
struct TrackingResult {
  double rms_lat = 0.0;
  double max_lat = 0.0;
  double final_speed_err = 0.0;
};

TrackingResult track(Controller& ctrl, const ReferenceLine& ref, double v0, double y_offset) {
  VehicleParams v;
  Trajectory traj;
  generateFrenet(FrenetState{5.0, v0, 0.0, 0.0, 0.0, 0.0}, EndState{3.5, v0 + 3.0, 6.0}, 0.05, traj);
  convertToCartesian(ref, traj);
  const auto r0 = ref.at(5.0);
  KinematicBicycle car(
      v, {r0.x - std::sin(r0.yaw) * y_offset, r0.y + std::cos(r0.yaw) * y_offset, r0.yaw, v0, 0.0, 0.0});
  TrackingResult out;
  int n = 0;
  for (double t = 0.0; t < 5.5; t += 0.05) {
    const auto cmd = ctrl.compute(car.state(), traj, t);
    car.step(cmd, 0.05);
    const EgoState target = sampleTrajectory(traj, t + 0.05);
    const double e = -(car.state().x - target.x) * std::sin(target.yaw) +
                     (car.state().y - target.y) * std::cos(target.yaw);
    if (t > 1.5) {
      out.rms_lat += e * e;
      out.max_lat = std::max(out.max_lat, std::abs(e));
      ++n;
    }
    out.final_speed_err = car.state().v - target.v;
  }
  out.rms_lat = std::sqrt(out.rms_lat / std::max(1, n));
  return out;
}

}  // namespace

TEST(Controllers, StanleyTracksLaneChangeOnCurve) {
  StanleyPidController ctrl(VehicleParams{});
  const auto r = track(ctrl, test::sCurve(), 10.0, 0.5);
  EXPECT_LT(r.max_lat, 0.5);
  EXPECT_LT(std::abs(r.final_speed_err), 0.5);
}

TEST(Controllers, MpcTracksLaneChangeOnCurveBetterThanStanley) {
  if (!hasOptimizers()) GTEST_SKIP() << "built without OSQP";
  MpcController mpc(VehicleParams{});
  StanleyPidController stanley(VehicleParams{});
  const auto ref = test::sCurve();
  const auto rm = track(mpc, ref, 10.0, 0.5);
  const auto rs = track(stanley, ref, 10.0, 0.5);
  EXPECT_TRUE(mpc.lastSolveOk());
  EXPECT_LT(rm.max_lat, 0.2);
  EXPECT_LT(rm.rms_lat, rs.rms_lat);
  EXPECT_LT(std::abs(rm.final_speed_err), 0.3);
  // Also at highway speed.
  const auto fast = track(mpc, ref, 25.0, 0.3);
  EXPECT_LT(fast.max_lat, 0.3);
}

TEST(LocalPlanner, SmootherIntegration) {
  if (!hasOptimizers()) GTEST_SKIP() << "built without OSQP";
  LocalPlannerConfig cfg;
  cfg.smoother.enabled = true;
  LocalPlanner planner(cfg, VehicleParams{});
  planner.setReferencePath({-20.0, 100.0, 300.0}, {0.0, 0.0, 0.0});
  const std::vector<Obstacle> obs = {staticBox(1, 45.0, 0.0)};
  const auto out = planner.step(0.0, {10.0, 0.0, 0.0, 10.0, 0.0, 0.0}, obs);
  ASSERT_TRUE(out.success);
  EXPECT_TRUE(out.smoothed);
}

TEST(Controllers, Factory) {
  EXPECT_EQ(makeController("mpc", VehicleParams{})->name(), "mpc");
  EXPECT_EQ(makeController("stanley_pid", VehicleParams{})->name(), "stanley_pid");
  EXPECT_THROW(makeController("pure_pursuit", VehicleParams{}), std::invalid_argument);
}
