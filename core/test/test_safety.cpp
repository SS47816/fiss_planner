// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "fiss_planner/planning/frenet_obstacles.hpp"
#include "fiss_planner/planning/local_planner.hpp"
#include "fiss_planner/planning/planner.hpp"
#include "test_utils.hpp"

using namespace fiss_planner;

namespace {

Obstacle car(int id, double x, double y, double v, double yaw = 0.0) {
  return predictConstantVelocity(id, x, y, yaw, v, 4.5, 1.9, 6.0, 0.1);
}

/// Lane-keeping trajectory at constant speed v from station s0 (Frenet only).
Trajectory cruise(double s0, double v, double T = 4.0, double d = 0.0) {
  Trajectory traj;
  traj.end = {d, v, T};
  generateFrenet({s0, v, 0.0, d, 0.0, 0.0}, traj.end, 0.1, traj);
  return traj;
}

struct Road {
  ReferenceLine ref = test::straightLine(400.0);
  Corridor corridor = Corridor::parallelLanes(0.0, 400.0, 3.5, 1, 0);
  FrenetObstacles project(const std::vector<Obstacle>& obs) const {
    return FrenetObstacles(obs, ref, -100.0, 400.0, -10.0, 10.0, 5.0);
  }
};

}  // namespace

TEST(FrenetObstacles, ProjectsExtentAndSpeedOnStraightAndCurvedReferences) {
  const Road road;
  const auto fo = road.project({car(1, 50.0, 3.5, 8.0)});
  ASSERT_EQ(fo.tracks().size(), 1u);
  FrenetBox box;
  double v = 0.0;
  ASSERT_TRUE(fo.tracks()[0].at(1.25, box, v));  // between grid samples
  EXPECT_NEAR(0.5 * (box.s_min + box.s_max), 60.0, 1e-6);
  EXPECT_NEAR(box.s_max - box.s_min, 4.5, 1e-6);
  EXPECT_NEAR(0.5 * (box.d_min + box.d_max), 3.5, 1e-6);
  EXPECT_NEAR(v, 8.0, 1e-6);
  EXPECT_FALSE(fo.tracks()[0].at(6.5, box, v));  // beyond the prediction

  // On an arc the car follows the line: s grows at its speed, d stays constant.
  const auto arc = test::arc(80.0, 1.5);
  const auto p = arc.at(40.0);
  const Obstacle on_arc = car(2, p.x, p.y, 0.0, p.yaw);  // parked
  const FrenetObstacles fa({on_arc}, arc, 0.0, 120.0, -5.0, 5.0, 5.0);
  ASSERT_EQ(fa.tracks().size(), 1u);
  ASSERT_TRUE(fa.tracks()[0].at(3.0, box, v));
  EXPECT_NEAR(0.5 * (box.s_min + box.s_max), 40.0, 0.05);
  EXPECT_NEAR(0.5 * (box.d_min + box.d_max), 0.0, 0.05);
  EXPECT_NEAR(v, 0.0, 1e-9);

  // Far outside the window: dropped.
  EXPECT_TRUE(road.project({car(3, 50.0, 40.0, 0.0)}).empty());
}

TEST(GapEvaluator, ProximityCostIsZeroWithoutConflictAndGrowsWhenCloser) {
  const Road road;
  const GapEvaluator gaps(CostConfig{}, SafetyConfig{}, VehicleParams{}, 0.3);
  const Trajectory traj = cruise(10.0, 10.0);
  EXPECT_EQ(gaps.cost(traj, road.project({})), 0.0);
  EXPECT_EQ(gaps.cost(traj, road.project({car(1, -20.0, 0.0, 10.0)})), 0.0);  // behind
  EXPECT_EQ(gaps.cost(traj, road.project({car(1, 200.0, 0.0, 10.0)})), 0.0);  // far ahead
  EXPECT_EQ(gaps.cost(traj, road.project({car(1, 40.0, 3.5, 10.0)})), 0.0);   // next lane, normal spacing

  const double far = gaps.cost(traj, road.project({car(1, 32.0, 0.0, 10.0)}));
  const double near = gaps.cost(traj, road.project({car(1, 25.0, 0.0, 10.0)}));
  const double closing = gaps.cost(traj, road.project({car(1, 25.0, 0.0, 6.0)}));
  EXPECT_GT(far, 0.0);
  EXPECT_GT(near, far);
  EXPECT_GT(closing, near);  // TTC term

  // Lateral clearance: squeezing past a car alongside costs more than passing wide.
  const double wide = gaps.cost(traj, road.project({car(1, 10.0, 3.5, 10.0)}));  // alongside
  const double tight = gaps.cost(traj, road.project({car(1, 10.0, 2.6, 10.0)}));
  EXPECT_GT(tight, wide);
}

TEST(GapEvaluator, EnvelopeRejectsTailgatingAndReportsTheObstacle) {
  const Road road;
  const GapEvaluator gaps(CostConfig{}, SafetyConfig{}, VehicleParams{}, 0.3);
  // 3.5 m behind a slower car: may not get any closer once the reaction time is over.
  Trajectory close = cruise(10.0, 10.0);
  EXPECT_FALSE(gaps.isSafe(close, road.project({car(7, 18.0, 0.0, 8.0)})));
  EXPECT_EQ(close.blocking_id, 7);
  EXPECT_NEAR(close.blocking_t, SafetyConfig{}.reaction_time, 1e-9);
  // Changing lanes into a 3.5 m gap behind a car gets no grace at all.
  Trajectory merge;
  merge.end = {3.5, 10.0, 3.0};
  generateFrenet({10.0, 10.0, 0.0, 0.0, 0.0, 0.0}, merge.end, 0.1, merge);
  EXPECT_FALSE(gaps.isSafe(merge, road.project({car(8, 18.0, 3.5, 10.0)})));
  EXPECT_EQ(merge.blocking_id, 8);

  Trajectory ok = cruise(10.0, 10.0);
  EXPECT_TRUE(gaps.isSafe(ok, road.project({car(7, 40.0, 0.0, 10.0)})));
  // Closing on a stopped car: fine early on, but the TTC floor is hit later.
  Trajectory into = cruise(10.0, 10.0);
  EXPECT_FALSE(gaps.isSafe(into, road.project({car(7, 60.0, 0.0, 0.0)})));
  EXPECT_GT(into.blocking_t, 2.0);
  // Only checked up to the given horizon.
  Trajectory short_horizon = cruise(10.0, 10.0);
  EXPECT_TRUE(gaps.isSafe(short_horizon, road.project({car(7, 60.0, 0.0, 0.0)}), 1.0));
  // Disabled envelope.
  SafetyConfig off;
  off.enabled = false;
  Trajectory any = cruise(10.0, 10.0);
  EXPECT_TRUE(GapEvaluator(CostConfig{}, off, VehicleParams{}, 0.3)
                  .isSafe(any, road.project({car(7, 18.0, 0.0, 8.0)})));
}

TEST(GapEvaluator, CrossingAndOncomingTrafficIsLeftToTheCollisionCheck) {
  const Road road;
  const GapEvaluator gaps(CostConfig{}, SafetyConfig{}, VehicleParams{}, 0.3);
  // A car crossing our lane 12 m ahead, and one coming towards us in our lane.
  const auto crossing = road.project({car(1, 26.5, -3.0, 3.0, kPi / 2)});
  const auto oncoming = road.project({car(2, 60.0, 0.0, 10.0, kPi)});
  for (const auto* fo : {&crossing, &oncoming}) {
    Trajectory stop;
    stop.end = {0.0, 0.0, 4.0};
    generateFrenet({10.0, 6.0, 0.0, 0.0, 0.0, 0.0}, stop.end, 0.1, stop);
    EXPECT_TRUE(gaps.isSafe(stop, *fo));
    EXPECT_EQ(gaps.leadGap(10.0, 0.0, 6.0, *fo).id, -1);
  }
  // Parked across the lane still counts (static).
  Trajectory into = cruise(10.0, 10.0);
  EXPECT_FALSE(gaps.isSafe(into, road.project({car(3, 50.0, 0.0, 0.0, kPi / 2)})));
}

TEST(GapEvaluator, LeadGapMeasuresTheClosestObstacleInPath) {
  const Road road;
  const GapEvaluator gaps(CostConfig{}, SafetyConfig{}, VehicleParams{}, 0.3);
  const auto fo = road.project({car(1, 50.0, 0.0, 5.0), car(2, 30.0, 0.0, 8.0), car(3, 20.0, 3.5, 0.0)});
  const LeadGap lead = gaps.leadGap(10.0, 0.0, 10.0, fo);
  EXPECT_EQ(lead.id, 2);
  EXPECT_NEAR(lead.gap, 30.0 - 10.0 - 4.5, 1e-6);
  EXPECT_NEAR(lead.time_headway, lead.gap / 10.0, 1e-9);
  EXPECT_NEAR(lead.ttc, lead.gap / 2.0, 1e-9);
  EXPECT_EQ(gaps.leadGap(10.0, 0.0, 10.0, road.project({})).id, -1);
}

TEST(Planner, CloseCutInStillLeavesABrakingCandidate) {
  const Road road;
  // A slower car merges from the left lane into a ~4 m gap ahead of our bumper.
  Obstacle cut;
  cut.id = 9;
  cut.length = 4.5;
  cut.width = 1.9;
  for (double t = 0.0; t <= 6.0; t += 0.1) {
    cut.prediction.push_back({t, 22.0 + 7.0 * t, 3.5 * std::max(0.0, 1.0 - t), t < 1.0 ? -0.3 : 0.0, 7.0});
  }
  const std::vector<Obstacle> obs = {cut};
  for (const auto type : {PlannerType::kFop, PlannerType::kFiss, PlannerType::kFissPlus}) {
    PlannerConfig cfg;
    cfg.type = type;
    auto planner = makePlanner(cfg, VehicleParams{});
    PlanningInput in{{10.0, 10.0, 0.0, 0.0, 0.0, 0.0}, &road.ref, &road.corridor, &obs, 12.0};
    const auto res = planner->plan(in);
    ASSERT_TRUE(res.success) << toString(type);
    EXPECT_LT(res.trajectory.s_d.back(), 10.0) << toString(type);  // it backs off
    EXPECT_GT(res.stats.lead.gap, 0.0);
  }
}

TEST(Planner, FollowsASlowerLeadAtTheDesiredGap) {
  const Road road;
  const std::vector<Obstacle> obs = {car(1, 30.0, 0.0, 8.0)};
  const Corridor single = Corridor::parallelLanes(0.0, 400.0, 3.5, 0, 0);  // no overtaking
  PlanningInput in{{10.0, 8.0, 0.0, 0.0, 0.0, 0.0}, &road.ref, &single, &obs, 15.0};
  PlannerConfig with;
  with.type = PlannerType::kFissPlus;
  PlannerConfig without = with;
  without.cost.w_headway = without.cost.w_ttc = without.cost.w_clearance = 0.0;
  without.cost.follow_time = 0.0;
  const auto a = makePlanner(with, VehicleParams{})->plan(in);
  const auto b = makePlanner(without, VehicleParams{})->plan(in);
  ASSERT_TRUE(a.success && b.success);
  const auto gap_at_end = [&](const Trajectory& t) {
    return 30.0 + 8.0 * t.t.back() - 2.25 - t.s.back() - 2.25;
  };
  const double desired = CostConfig{}.desired_gap + CostConfig{}.desired_time_headway * 8.0;
  EXPECT_GT(gap_at_end(a.trajectory), gap_at_end(b.trajectory));
  EXPECT_GT(gap_at_end(a.trajectory), 0.8 * desired);
  EXPECT_NEAR(a.stats.lead.speed, 8.0, 1e-6);
}

TEST(LocalPlanner, OvertakesASlowerCarWhenTheNextLaneIsFree) {
  // Closed loop with perfect tracking: catch up with a car at 5 m/s, then pass it
  // in the free left lane, without ever needing an emergency stop.
  LocalPlannerConfig cfg;
  cfg.path.num_right_lanes = 0;
  cfg.desired_speed = 14.0;
  LocalPlanner lp(cfg, VehicleParams{});
  lp.setReferencePath({0.0, 600.0}, {0.0, 0.0});
  EgoState ego{10.0, 0.0, 0.0, 12.0, 0.0, 0.0};
  bool passed = false;
  double max_d = 0.0;
  for (int k = 0; k < 200 && !passed; ++k) {
    const double t = 0.1 * k;
    const auto out = lp.step(t, ego, {car(1, 30.0 + 5.0 * t, 0.0, 5.0)});
    ASSERT_FALSE(out.emergency) << "t = " << t;
    ego = sampleTrajectory(out.trajectory, 0.1);
    max_d = std::max(max_d, ego.y);
    passed = ego.x - 4.5 > 30.0 + 5.0 * (t + 0.1);
  }
  EXPECT_TRUE(passed);
  EXPECT_GT(max_d, 2.5);
}

TEST(Planner, ReportsTheBlockingObstacle) {
  const Road road;
  const std::vector<Obstacle> obs = {car(4, 16.0, 0.0, 0.0), car(5, 16.0, 3.5, 0.0)};
  PlanningInput in{{0.0, 15.0, 0.0, 0.0, 0.0, 0.0}, &road.ref, &road.corridor, &obs, 15.0};
  const auto res = makePlanner(PlannerConfig{}, VehicleParams{})->plan(in);
  EXPECT_FALSE(res.success);
  EXPECT_TRUE(res.stats.blocking_id == 4 || res.stats.blocking_id == 5);
}

TEST(Planner, RearVehiclesInTheEgoLaneAreIgnoredOnlyWhenKeepingTheLane) {
  const Road road;
  // A much faster car right behind us in our lane, and one behind in the next lane.
  const std::vector<Obstacle> obs = {car(1, 2.0, 0.0, 20.0), car(2, 0.0, 3.5, 20.0)};
  PlanningInput in{{10.0, 8.0, 0.0, 0.0, 0.0, 0.0}, &road.ref, &road.corridor, &obs, 8.0};
  PlannerConfig cfg;
  cfg.type = PlannerType::kFop;
  cfg.keep_candidates = true;
  const auto strict = makePlanner(cfg, VehicleParams{})->plan(in);
  cfg.collision.ignore_rear_in_lane = true;
  const auto reactive = makePlanner(cfg, VehicleParams{})->plan(in);
  ASSERT_TRUE(reactive.success);
  EXPECT_EQ(reactive.trajectory.lane, 0);
  int lane_change_collisions = 0;
  for (const auto& c : reactive.candidates) {
    if (c.lane == 0) EXPECT_NE(c.blocking_id, 1);  // ignored while keeping the lane
    lane_change_collisions += c.lane == 1 && c.rejection == Rejection::kCollision;
  }
  EXPECT_GT(lane_change_collisions, 0);  // lane changes still check both cars
  EXPECT_GT(strict.stats.rejections[static_cast<int>(Rejection::kCollision)],
            reactive.stats.rejections[static_cast<int>(Rejection::kCollision)]);
}

TEST(LocalPlanner, DegradedRetryAvoidsAnEmergencyStopForADistantConflict) {
  // A long obstacle sliding across the road (straight-line prediction) blocks
  // every candidate after about 3 s, none before.
  LocalPlannerConfig cfg;
  cfg.path.num_left_lanes = cfg.path.num_right_lanes = 0;
  cfg.planner.sampling.min_speed = 6.0;  // no stopping candidates
  Obstacle wall;
  wall.id = 3;
  wall.length = 80.0;
  wall.width = 2.0;
  for (double t = 0.0; t <= 6.0; t += 0.1) wall.prediction.push_back({t, 50.0, -30.0 + 10.0 * t, 0.0, 10.0});
  const std::vector<Obstacle> obs = {wall};
  auto strict_cfg = cfg;
  strict_cfg.degraded_horizon = 0.0;
  LocalPlanner strict_lp(strict_cfg, VehicleParams{});
  LocalPlanner lp(cfg, VehicleParams{});
  strict_lp.setReferencePath({0.0, 300.0}, {0.0, 0.0});
  lp.setReferencePath({0.0, 300.0}, {0.0, 0.0});
  const EgoState ego{10.0, 0.0, 0.0, 10.0, 0.0, 0.0};
  const auto a = strict_lp.step(0.0, ego, obs);
  const auto b = lp.step(0.0, ego, obs);
  EXPECT_TRUE(a.emergency);
  EXPECT_EQ(a.stats.blocking_id, 3);
  EXPECT_TRUE(b.success && b.degraded && !b.emergency);
}

TEST(LocalPlanner, CloseCutInFallsBackToACollisionFreePlanInsteadOfBraking) {
  // A car merges into a ~1 m gap ahead: no candidate keeps the envelope, but
  // collision-free (close) braking plans exist and beat an emergency stop.
  LocalPlannerConfig cfg;
  cfg.path.num_left_lanes = cfg.path.num_right_lanes = 0;
  Obstacle cut;
  cut.id = 9;
  cut.length = 4.5;
  cut.width = 1.9;
  for (double t = 0.0; t <= 6.0; t += 0.1) {
    cut.prediction.push_back({t, 18.5 + 7.0 * t, 3.5 * std::max(0.0, 1.0 - t), t < 1.0 ? -0.3 : 0.0, 7.0});
  }
  LocalPlanner lp(cfg, VehicleParams{});
  lp.setReferencePath({0.0, 300.0}, {0.0, 0.0});
  const auto out = lp.step(0.0, {10.0, 0.0, 0.0, 10.0, 0.0, 0.0}, {cut});
  EXPECT_TRUE(out.success && out.degraded && !out.emergency);
  EXPECT_GT(out.stats.rejections[static_cast<int>(Rejection::kSafetyDistance)], 0);
  EXPECT_EQ(out.stats.blocking_id, 9);
}

TEST(LocalPlanner, ResumesAfterAnEmergencyStopOnceTheWayIsClear) {
  LocalPlannerConfig cfg;
  cfg.path.num_left_lanes = cfg.path.num_right_lanes = 0;
  cfg.degraded_horizon = 0.0;
  LocalPlanner lp(cfg, VehicleParams{});
  lp.setReferencePath({0.0, 600.0}, {0.0, 0.0});
  // A car stopped right in front: emergency stop.
  const auto stop = lp.step(0.0, {10.0, 0.0, 0.0, 12.0, 0.0, 0.0}, {car(1, 22.0, 0.0, 0.0)});
  ASSERT_TRUE(stop.emergency);
  // It drives off; the ego is braking hard at that moment.
  const auto out = lp.step(0.1, {11.2, 0.0, 0.0, 11.4, -6.0, 0.0}, {});
  ASSERT_TRUE(out.success);
  EXPECT_FALSE(out.stitched);
  EXPECT_GE(out.trajectory.s_dd.front(), -cfg.recovery_decel - 1e-6);
  EXPECT_GT(out.trajectory.s_d.back(), 11.4);  // resumes towards the desired speed
}

TEST(CollisionChecker, KeepsATimeGapToCrossingTraffic) {
  const Road road;
  const CollisionChecker checker(CollisionConfig{}, VehicleParams{});
  const double gap = SafetyConfig{}.crossing_time_gap;
  int checks = 0;
  // Ego cruises at 10 m/s and reaches x = 30 at t = 2.8 s.
  Trajectory traj = cruise(2.0, 10.0);
  ASSERT_TRUE(convertToCartesian(road.ref, traj));
  // A car crossing the road at x = 30 clears the ego's lane at about t = 2.3 s,
  // just before the ego arrives: no collision, but too close behind it.
  const Obstacle crossing = car(4, 30.0, -20.0, 10.0, kPi / 2);
  EXPECT_TRUE(checker.isCollisionFree(traj, {crossing}, checks));
  int hit = -1;
  EXPECT_FALSE(checker.keepsTimeGap(traj, {crossing}, gap, checks, &hit));
  EXPECT_EQ(hit, 4);
  EXPECT_TRUE(checker.keepsTimeGap(traj, {crossing}, 0.0, checks));  // off

  // Following traffic and parked cars are not affected.
  EXPECT_TRUE(checker.keepsTimeGap(traj, {car(5, 60.0, 0.0, 10.0)}, gap, checks));
  EXPECT_TRUE(checker.keepsTimeGap(traj, {car(6, 30.0, 4.0, 0.0, kPi / 2)}, gap, checks));
}
