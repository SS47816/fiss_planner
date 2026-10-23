// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "frenet_planner/map/route_corridor.hpp"
#include "frenet_planner/planning/local_planner.hpp"
#include "test_utils.hpp"

using namespace frenet_planner;

namespace {

/// Straight two-lane road along +x made of three 60 m segments per lane.
/// Right lane: 1 -> 2 -> 3, left lane: 11 -> 12 -> 13 (y = 3.5).
/// The marking between the lanes is solid along segment 2.
RoadMap twoLaneRoad() {
  RoadMap map;
  const double w = 3.5, seg = 60.0;
  for (int lane = 0; lane < 2; ++lane) {
    for (int k = 0; k < 3; ++k) {
      MapLane l;
      l.id = (lane == 0 ? 1 : 11) + k;
      const double yc = lane * w;
      for (double x = k * seg; x <= (k + 1) * seg + 1e-9; x += 5.0) {
        l.centerline.push_back({x, yc});
        l.left.push_back({x, yc + w / 2});
        l.right.push_back({x, yc - w / 2});
      }
      if (k < 2) l.successors = {l.id + 1};
      const bool solid = (k == 1);
      if (lane == 0) {
        l.left_neighbor = 11 + k;
        l.can_change_left = !solid;
        l.left_type = solid ? LineType::kSolid : LineType::kDashed;
        l.right_type = LineType::kRoadEdge;
      } else {
        l.right_neighbor = 1 + k;
        l.can_change_right = !solid;
        l.right_type = solid ? LineType::kSolid : LineType::kDashed;
        l.left_type = LineType::kRoadEdge;
      }
      l.speed_limit = 20.0;
      map.addLane(std::move(l));
    }
  }
  map.finalize();
  return map;
}

EgoState at(double x, double y, double v = 10.0) { return {x, y, 0.0, v, 0.0, 0.0}; }

const LaneBand* band(const Corridor& c, int offset) {
  for (const auto& l : c.lanes())
    if (l.offset == offset) return &l;
  return nullptr;
}

}  // namespace

TEST(RoadMap, LocateAndPolylineHelpers) {
  const auto map = twoLaneRoad();
  EXPECT_EQ(map.locate(10.0, 0.3, 0.0), 1);
  EXPECT_EQ(map.locate(70.0, 3.0, 0.0), 12);
  EXPECT_EQ(map.locate(70.0, 3.0, kPi), -1);  // wrong direction
  EXPECT_EQ(map.lane(2)->predecessors, std::vector<int>{1});
  EXPECT_NEAR(map.lane(1)->length(), 60.0, 1e-9);
  double heading = 1.0;
  const auto p = polylinePoint(map.lane(1)->centerline, 12.5, &heading);
  EXPECT_NEAR(p.x, 12.5, 1e-9);
  EXPECT_NEAR(heading, 0.0, 1e-12);
}

TEST(RouteCorridor, BuildsBandsFromNeighbours) {
  const auto map = twoLaneRoad();
  RouteCorridorConfig cfg;
  const auto rc = buildRouteCorridor(map, {}, at(10.0, 0.0), cfg);
  ASSERT_TRUE(rc);
  EXPECT_EQ(rc->ego_lane, 1);
  EXPECT_NEAR(rc->ego_s, 10.0, 1e-3);
  EXPECT_EQ(rc->ego_chain, (std::vector<int>{1, 2, 3}));
  const auto* left = band(rc->corridor, 1);
  ASSERT_NE(left, nullptr);
  EXPECT_TRUE(left->change_allowed);
  EXPECT_NEAR(rc->corridor.center(*left, 30.0), 3.5, 1e-3);
  EXPECT_NEAR(rc->corridor.left(*left, 30.0), 5.25, 1e-3);
  EXPECT_EQ(band(rc->corridor, -1), nullptr);  // no right neighbour
  EXPECT_NEAR(rc->speed_limit, 20.0, 1e-12);
  const auto bounds = rc->corridor.roadBounds(30.0);
  EXPECT_NEAR(bounds.first, -1.75, 1e-3);
  EXPECT_NEAR(bounds.second, 5.25, 1e-3);
}

TEST(RouteCorridor, SolidLineBlocksLaneChange) {
  const auto map = twoLaneRoad();
  const auto rc = buildRouteCorridor(map, {}, at(80.0, 0.0), RouteCorridorConfig{});
  ASSERT_TRUE(rc);
  EXPECT_EQ(rc->ego_lane, 2);
  EXPECT_FALSE(band(rc->corridor, 1)->change_allowed);
  // The blocked lane is excluded from the road bounds.
  EXPECT_NEAR(rc->corridor.roadBounds(90.0).second, 1.75, 1e-3);
}

TEST(RouteCorridor, RouteUrgencyRampsUpBeforeTheRouteLeavesTheLane) {
  const auto map = twoLaneRoad();
  const std::vector<int> route = {1, 11, 12, 13};  // must move left before segment 2
  RouteCorridorConfig cfg;
  cfg.urgency_distance = 80.0;
  const auto far = buildRouteCorridor(map, route, at(5.0, 0.0), cfg);
  const auto near = buildRouteCorridor(map, route, at(50.0, 0.0), cfg);
  ASSERT_TRUE(far && near);
  EXPECT_NEAR(band(far->corridor, 1)->extra_cost, 0.0, 1e-12);
  EXPECT_GT(band(far->corridor, 0)->extra_cost, 0.0);
  EXPECT_GT(band(near->corridor, 0)->extra_cost, band(far->corridor, 0)->extra_cost);
}

TEST(RouteCorridor, ActiveStopLinesBecomeObstacles) {
  auto map = twoLaneRoad();
  map.addStopLine({5, 2, 20.0, true});
  const auto rc = buildRouteCorridor(map, {}, at(10.0, 0.0), RouteCorridorConfig{});
  ASSERT_TRUE(rc);
  ASSERT_EQ(rc->stop_lines.size(), 1u);
  EXPECT_NEAR(rc->stop_lines[0].prediction[0].x, 80.0, 1e-6);
  EXPECT_NEAR(rc->stop_lines[0].width, 3.5, 1e-6);
  map.setStopLineActive(5, false);
  EXPECT_TRUE(buildRouteCorridor(map, {}, at(10.0, 0.0), RouteCorridorConfig{})->stop_lines.empty());
}

TEST(RouteCorridor, OffMapReturnsNothing) {
  const auto map = twoLaneRoad();
  EXPECT_FALSE(buildRouteCorridor(map, {}, at(10.0, 40.0), RouteCorridorConfig{}));
}

namespace {

/// Closed loop with perfect tracking: the ego jumps to the planned state one
/// control period later (like the CommonRoad harness).
struct Loop {
  LocalPlanner planner;
  EgoState ego;
  double t = 0.0;
  int successes = 0;
  int stitched = 0;

  Loop(const LocalPlannerConfig& cfg, EgoState start) : planner(cfg, VehicleParams{}), ego(start) {}

  LocalPlanOutput step(const std::vector<Obstacle>& world, double dt = 0.1) {
    std::vector<Obstacle> obs;
    for (const auto& ob : world) {  // re-time predictions relative to now
      Obstacle o = ob;
      for (auto& p : o.prediction) p.t -= t;
      obs.push_back(o);
    }
    auto out = planner.step(t, ego, obs);
    successes += out.success;
    stitched += out.stitched;
    if (out.trajectory.size() > 1) ego = sampleTrajectory(out.trajectory, dt);
    t += dt;
    return out;
  }
};

bool overlapsAny(const EgoState& ego, const std::vector<Obstacle>& world, double t) {
  const VehicleParams v;
  const Box2d box{ego.x, ego.y, ego.yaw, v.length, v.width};
  for (const auto& ob : world) {
    const auto b = ob.boxAt(t);
    if (b && overlaps(box, *b)) return true;
  }
  return false;
}

}  // namespace

TEST(LocalPlanner, PathModeOvertakesStaticObstacleInClosedLoop) {
  LocalPlannerConfig cfg;
  cfg.desired_speed = 12.0;
  cfg.path.num_right_lanes = 0;
  Loop loop(cfg, at(0.0, 0.0, 10.0));
  std::vector<double> x, y;
  for (double s = -20.0; s <= 400.0; s += 10.0) {
    x.push_back(s);
    y.push_back(0.0);
  }
  loop.planner.setReferencePath(x, y);
  const std::vector<Obstacle> world = {predictConstantVelocity(1, 80.0, 0.0, 0.0, 0.0, 4.5, 2.0, 60.0, 0.1)};
  double max_y = 0.0;
  for (int i = 0; i < 150; ++i) {
    loop.step(world);
    ASSERT_FALSE(overlapsAny(loop.ego, world, loop.t)) << "collision at t=" << loop.t;
    max_y = std::max(max_y, loop.ego.y);
  }
  EXPECT_GT(loop.ego.x, 120.0);  // passed the obstacle
  EXPECT_GT(max_y, 2.5);         // via the left lane
  EXPECT_GE(loop.successes, 140);
  EXPECT_GE(loop.stitched, 140);
}

TEST(LocalPlanner, MapModeStopsAtRedLight) {
  auto map = std::make_shared<RoadMap>(twoLaneRoad());
  map->addStopLine({1, 2, 20.0, true});   // at x = 80 in the right lane
  map->addStopLine({2, 12, 20.0, true});  // and in the left lane
  LocalPlannerConfig cfg;
  cfg.desired_speed = 12.0;
  Loop loop(cfg, at(0.0, 0.0, 10.0));
  loop.planner.setMap(map);
  for (int i = 0; i < 200; ++i) loop.step({});
  EXPECT_LT(loop.ego.x + VehicleParams{}.length / 2, 80.0);
  EXPECT_GT(loop.ego.x, 60.0);
  EXPECT_LT(loop.ego.v, 0.5);
}

TEST(LocalPlanner, MapModeFollowsRouteLaneChange) {
  auto map = std::make_shared<RoadMap>(twoLaneRoad());
  LocalPlannerConfig cfg;
  cfg.desired_speed = 12.0;
  Loop loop(cfg, at(0.0, 0.0, 10.0));
  loop.planner.setMap(map, {1, 11, 12, 13});
  for (int i = 0; i < 100; ++i) loop.step({});
  EXPECT_NEAR(loop.ego.y, 3.5, 0.5);  // ended up in the route's lane
}

TEST(LocalPlanner, CurvatureLimitsTargetSpeed) {
  LocalPlannerConfig cfg;
  cfg.desired_speed = 25.0;
  LocalPlanner planner(cfg, VehicleParams{});
  const auto ref = test::arc(30.0, kPi);
  std::vector<double> x, y;
  for (const auto& p : ref.sample(2.0)) {
    x.push_back(p.x);
    y.push_back(p.y);
  }
  planner.setReferencePath(x, y);
  const auto out = planner.step(0.0, at(0.0, 0.0, 8.0), {});
  EXPECT_LE(out.target_speed, std::sqrt(VehicleParams{}.max_lat_accel * 30.0) + 1e-6);
}

TEST(LocalPlanner, RuntimePlannerSwitchAndFallback) {
  LocalPlannerConfig cfg;
  LocalPlanner planner(cfg, VehicleParams{});
  planner.setReferencePath({0.0, 100.0, 200.0}, {0.0, 0.0, 0.0});
  EXPECT_EQ(planner.planner().type(), PlannerType::kFissPlus);
  planner.requestPlannerType(PlannerType::kFop);
  auto out = planner.step(0.0, at(10.0, 0.0), {});
  EXPECT_EQ(planner.planner().type(), PlannerType::kFop);
  ASSERT_TRUE(out.success);

  // A wall right ahead in every lane: no plan, and the previous plan would hit
  // it, so the planner brakes at the limit in its lane.
  const std::vector<Obstacle> wall = {predictConstantVelocity(1, 13.0, 0.0, 0.0, 0.0, 1.0, 12.0, 6.0, 0.1)};
  out = planner.step(0.1, at(11.0, 0.0), wall);
  EXPECT_FALSE(out.success);
  EXPECT_FALSE(out.fallback);
  ASSERT_TRUE(out.emergency);
  ASSERT_GT(out.trajectory.size(), 1u);
  EXPECT_NEAR(out.trajectory.s_dd.front(), -VehicleParams{}.max_decel, 1e-9);
  EXPECT_NEAR(out.trajectory.v.back(), 0.0, 1e-6);
  EXPECT_NEAR(out.trajectory.y.back(), 0.0, 1e-6);
}

TEST(LocalPlanner, FallsBackToPreviousPlanWhenItIsStillSafe) {
  LocalPlanner planner(LocalPlannerConfig{}, VehicleParams{});
  planner.setReferencePath({0.0, 100.0, 200.0}, {0.0, 0.0, 0.0});
  ASSERT_TRUE(planner.step(0.0, at(10.0, 0.0), {}).success);
  // An obstacle that blocks every new candidate's end but not the near-term
  // previous plan is hard to construct; instead make planning impossible via
  // an unreachable target (all candidates violate the speed limit).
  auto cfg = planner.config();
  cfg.planner.sampling.min_speed = 40.0;  // above vehicle max_speed
  planner.requestConfig(cfg);
  const auto out = planner.step(0.1, at(11.0, 0.0), {});
  EXPECT_FALSE(out.success);
  EXPECT_TRUE(out.fallback);
  EXPECT_FALSE(out.emergency);
}
