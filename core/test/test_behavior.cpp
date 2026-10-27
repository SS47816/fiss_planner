// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "fiss_planner/behavior/behavior_planner.hpp"
#include "fiss_planner/behavior/behavior_tree.hpp"
#include "fiss_planner/common/math.hpp"
#include "fiss_planner/planning/local_planner.hpp"

using namespace fiss_planner;

namespace {

/// A straight lane along x in three parts: 0-60 m, a junction lane 60-80 m, 80-200 m.
std::shared_ptr<RoadMap> junctionRoad() {
  auto map = std::make_shared<RoadMap>();
  const double w = 3.5;
  const double bounds[3][2] = {{0.0, 60.0}, {60.0, 80.0}, {80.0, 200.0}};
  for (int k = 0; k < 3; ++k) {
    MapLane l;
    l.id = k + 1;
    l.junction = k == 1;
    for (double x = bounds[k][0]; x <= bounds[k][1] + 1e-9; x += 2.0) {
      l.centerline.push_back({x, 0.0});
      l.left.push_back({x, w / 2});
      l.right.push_back({x, -w / 2});
    }
    if (k < 2) l.successors = {k + 2};
    map->addLane(std::move(l));
  }
  map->finalize();
  return map;
}

LocalPlannerConfig behaviorConfig() {
  LocalPlannerConfig cfg;
  cfg.behavior.enabled = true;
  cfg.stitching.enabled = false;
  return cfg;
}

bool has(const std::vector<std::string>& v, const std::string& s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

double stopStation(const LocalPlanOutput& out) {
  double s = 1e9;
  for (const auto& ob : out.virtual_obstacles) {
    if (ob.id <= -2000) s = std::min(s, ob.prediction.front().x);
  }
  return s;
}

}  // namespace

TEST(BehaviorTree, SequenceAndFallbackSemantics) {
  using namespace behavior;
  struct B {
    int ticks = 0;
  };
  auto ok = [](B& b) {
    ++b.ticks;
    return Status::kSuccess;
  };
  auto fail = [](B& b) {
    ++b.ticks;
    return Status::kFailure;
  };
  B b;
  auto seq = sequence<B>("s", action<B>("a", ok), action<B>("f", fail), action<B>("never", ok));
  EXPECT_EQ(seq->tick(b), Status::kFailure);
  EXPECT_EQ(b.ticks, 2);
  b.ticks = 0;
  auto fb = fallback<B>("f", action<B>("f", fail), action<B>("a", ok), action<B>("never", ok));
  EXPECT_EQ(fb->tick(b), Status::kSuccess);
  EXPECT_EQ(b.ticks, 2);
  b.ticks = 0;
  auto opt = optional<B>("o", action<B>("f", fail));
  EXPECT_EQ(opt->tick(b), Status::kSuccess);
  auto cond = condition<B>("c", [](const B& x) { return x.ticks > 0; });
  EXPECT_EQ(cond->tick(b), Status::kSuccess);
}

TEST(Behavior, YieldsAtAJunctionEntryToCrossingTraffic) {
  LocalPlanner lp(behaviorConfig(), VehicleParams{});
  lp.setMap(junctionRoad(), {1, 2, 3});
  // A car crossing the junction at x = 70 when the ego would get there.
  const Obstacle crossing = predictConstantVelocity(7, 70.0, -40.0, kPi / 2, 8.0, 4.5, 1.9, 8.0, 0.1);
  const auto out = lp.step(0.0, EgoState{30.0, 0.0, 0.0, 6.0}, {crossing});
  EXPECT_TRUE(has(out.behavior, "yield_junction"));
  EXPECT_LT(stopStation(out), 60.0);
  ASSERT_GE(out.trajectory.size(), 2u);
  EXPECT_LT(out.trajectory.x.back() + 0.5 * VehicleParams{}.length, 60.0);  // waits before the entry

  // Without crossing traffic the ego proceeds.
  LocalPlanner free(behaviorConfig(), VehicleParams{});
  free.setMap(junctionRoad(), {1, 2, 3});
  const auto go = free.step(0.0, EgoState{30.0, 0.0, 0.0, 6.0}, {});
  EXPECT_TRUE(has(go.behavior, "proceed"));
  EXPECT_GT(go.trajectory.x.back(), 60.0);
}

TEST(Behavior, CommitsOnceInsideTheJunction) {
  LocalPlanner lp(behaviorConfig(), VehicleParams{});
  lp.setMap(junctionRoad(), {1, 2, 3});
  const Obstacle crossing = predictConstantVelocity(7, 75.0, -30.0, kPi / 2, 8.0, 4.5, 1.9, 8.0, 0.1);
  const auto out = lp.step(0.0, EgoState{62.0, 0.0, 0.0, 5.0}, {crossing});
  EXPECT_FALSE(has(out.behavior, "yield_junction"));
}

TEST(Behavior, IgnoresTrafficAlongTheEgoLane) {
  LocalPlanner lp(behaviorConfig(), VehicleParams{});
  lp.setMap(junctionRoad(), {1, 2, 3});
  const Obstacle lead = predictConstantVelocity(7, 50.0, 0.0, 0.0, 6.0, 4.5, 1.9, 8.0, 0.1);
  const auto out = lp.step(0.0, EgoState{30.0, 0.0, 0.0, 6.0}, {lead});
  EXPECT_FALSE(has(out.behavior, "yield_junction"));
}

TEST(Behavior, PassesAYellowLightOnlyIfStoppingIsUncomfortable) {
  for (const double v : {15.0, 6.0}) {
    auto map = junctionRoad();
    map->addStopLine(StopLine{1, 1, 58.0, true, true, false});
    LocalPlanner lp(behaviorConfig(), VehicleParams{});
    lp.setMap(map, {1, 2, 3});
    // 15 m/s, 10 m from the line: passes. 6 m/s: stops comfortably.
    const auto out = lp.step(0.0, EgoState{58.0 - 10.0 - 2.5, 0.0, 0.0, v}, {});
    EXPECT_EQ(has(out.behavior, "pass_yellow"), v > 10.0) << v;
    const bool line_kept = std::any_of(out.virtual_obstacles.begin(), out.virtual_obstacles.end(),
                                       [](const Obstacle& o) { return o.id == -1001; });
    EXPECT_EQ(line_kept, v < 10.0) << v;
  }
}

TEST(Behavior, StopsAtAStopSignThenGoes) {
  BehaviorConfig cfg;
  cfg.enabled = true;
  BehaviorPlanner bp(cfg, VehicleParams{});
  const ReferenceLine ref(std::vector<double>{0.0, 100.0}, std::vector<double>{0.0, 0.0});
  const std::vector<RouteCorridor::StopLineAhead> signs = {{4, 50.0, false, true}};
  const std::vector<Obstacle> none;
  BehaviorInput in;
  in.reference = &ref;
  in.stop_lines = &signs;
  in.obstacles = &none;
  in.dt = 0.1;
  // Approaching: keep the stop.
  in.start = FrenetState{20.0, 8.0, 0.0, 0.0, 0.0, 0.0};
  auto d = bp.plan(in);
  EXPECT_TRUE(has(d.active, "stop_sign_stop"));
  EXPECT_TRUE(d.released_stop_lines.empty());
  // Standing at the line: released after the wait, and stays released.
  in.start = FrenetState{50.0 - 2.5 - 1.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  int released_at = -1;
  for (int k = 0; k < 10 && released_at < 0; ++k) {
    if (!bp.plan(in).released_stop_lines.empty()) released_at = k;
  }
  EXPECT_GE(released_at, 3);
  EXPECT_LE(released_at, 6);
  in.start.s_d = 3.0;
  d = bp.plan(in);
  EXPECT_TRUE(has(d.active, "stop_sign_go"));
  ASSERT_EQ(d.released_stop_lines.size(), 1u);
  EXPECT_EQ(d.released_stop_lines[0], 4);
}

namespace {

/// Two lanes along x: lane 1 eastbound (y = 0) and lane 2 westbound (y = 3.5) beside it.
std::shared_ptr<RoadMap> twoWayRoad() {
  auto map = std::make_shared<RoadMap>();
  const double w = 3.5;
  MapLane east, west;
  east.id = 1;
  west.id = 2;
  for (double x = 0.0; x <= 200.0 + 1e-9; x += 2.0) {
    east.centerline.push_back({x, 0.0});
    east.left.push_back({x, w / 2});
    east.right.push_back({x, -w / 2});
  }
  for (double x = 200.0; x >= -1e-9; x -= 2.0) {
    west.centerline.push_back({x, w});
    west.left.push_back({x, w / 2});
    west.right.push_back({x, 1.5 * w});
  }
  east.left_opposite = 2;
  west.left_opposite = 1;
  map->addLane(std::move(east));
  map->addLane(std::move(west));
  map->finalize();
  return map;
}

LocalPlannerConfig passingConfig() {
  LocalPlannerConfig cfg = behaviorConfig();
  cfg.route.oncoming_lane = true;
  return cfg;
}

const LaneBand* oncomingBand(const LocalPlanOutput& out) {
  for (const auto& lane : out.corridor.lanes()) {
    if (lane.oncoming) return &lane;
  }
  return nullptr;
}

}  // namespace

TEST(Behavior, PassesABlockedLaneThroughTheOncomingLane) {
  LocalPlanner lp(passingConfig(), VehicleParams{});
  lp.setMap(twoWayRoad(), {1});
  Obstacle blocker = predictConstantVelocity(5, 60.0, 0.0, 0.0, 0.0, 4.5, 2.0, 6.0, 0.1);
  blocker.is_static = true;
  const auto out = lp.step(0.0, EgoState{30.0, 0.0, 0.0, 6.0}, {blocker});
  EXPECT_TRUE(has(out.behavior, "pass_blocked"));
  const LaneBand* band = oncomingBand(out);
  ASSERT_NE(band, nullptr);
  EXPECT_TRUE(band->change_allowed);
  EXPECT_NEAR(out.corridor.center(*band, 40.0), 3.5, 0.2);
  EXPECT_GT(out.trajectory.y.back(), 1.75);  // heads into the oncoming lane

  // Nothing blocks: the oncoming lane stays closed.
  LocalPlanner free(passingConfig(), VehicleParams{});
  free.setMap(twoWayRoad(), {1});
  const auto go = free.step(0.0, EgoState{30.0, 0.0, 0.0, 6.0}, {});
  EXPECT_FALSE(has(go.behavior, "pass_blocked"));
  ASSERT_NE(oncomingBand(go), nullptr);
  EXPECT_FALSE(oncomingBand(go)->change_allowed);
  EXPECT_LT(std::abs(go.trajectory.y.back()), 0.5);
}

TEST(Behavior, DoesNotPassAQueueBeforeAJunction) {
  auto map = junctionRoad();
  LocalPlanner lp(behaviorConfig(), VehicleParams{});
  lp.setMap(map, {1, 2, 3});
  // A car standing 10 m before the junction is queueing, not blocking.
  const Obstacle queued = predictConstantVelocity(5, 50.0, 0.0, 0.0, 0.0, 4.5, 1.9, 6.0, 0.1);
  const auto out = lp.step(0.0, EgoState{20.0, 0.0, 0.0, 5.0}, {queued});
  EXPECT_FALSE(has(out.behavior, "pass_blocked"));
}

TEST(Behavior, WaitsForOncomingTrafficBeforePassing) {
  Obstacle blocker = predictConstantVelocity(5, 60.0, 0.0, 0.0, 0.0, 4.5, 2.0, 6.0, 0.1);
  for (const double x_oncoming : {100.0, 190.0}) {
    LocalPlanner lp(passingConfig(), VehicleParams{});
    lp.setMap(twoWayRoad(), {1});
    // Oncoming at 10 m/s: at x = 100 it meets the ego in the blocked stretch, at x = 190 it does not.
    const Obstacle oncoming = predictConstantVelocity(6, x_oncoming, 3.5, kPi, 10.0, 4.5, 1.9, 6.0, 0.1);
    const auto out = lp.step(0.0, EgoState{40.0, 0.0, 0.0, 3.0}, {blocker, oncoming});
    EXPECT_TRUE(has(out.behavior, "pass_blocked"));
    ASSERT_NE(oncomingBand(out), nullptr);
    EXPECT_EQ(oncomingBand(out)->change_allowed, x_oncoming > 150.0) << x_oncoming;
    if (x_oncoming < 150.0)
      EXPECT_LT(out.trajectory.x.back() + 2.5, 60.0 - 2.25);  // waits behind the blocker
  }
}
