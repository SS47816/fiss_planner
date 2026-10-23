// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "frenet_planner/planning/planner.hpp"
#include "test_utils.hpp"

using namespace frenet_planner;

namespace {

struct Scenario {
  ReferenceLine ref;
  Corridor corridor;
  std::vector<Obstacle> obstacles;
  FrenetState start;
  double target_speed = 12.0;

  PlanningInput input() const {
    PlanningInput in;
    in.start = start;
    in.reference = &ref;
    in.corridor = &corridor;
    in.obstacles = &obstacles;
    in.target_speed = target_speed;
    return in;
  }
};

Scenario freeRoad() {
  Scenario sc;
  sc.ref = test::straightLine(300.0);
  sc.corridor = Corridor::parallelLanes(0.0, 300.0, 3.5, 1, 0);
  sc.start = {10.0, 10.0, 0.0, 0.0, 0.0, 0.0};
  return sc;
}

Obstacle box(int id, double x, double y, double v) {
  return predictConstantVelocity(id, x, y, 0.0, v, 4.5, 1.9, 6.0, 0.1);
}

/// Independent check with the raw (un-inflated) footprints.
bool collides(const Trajectory& traj, const std::vector<Obstacle>& obstacles, const VehicleParams& v) {
  for (size_t i = 0; i < traj.size(); ++i) {
    const Box2d ego{traj.x[i], traj.y[i], traj.yaw[i], v.length, v.width};
    for (const auto& ob : obstacles) {
      const auto b = ob.boxAt(traj.t[i]);
      if (b && overlaps(ego, *b)) return true;
    }
  }
  return false;
}

PlannerConfig configFor(PlannerType type) {
  PlannerConfig cfg;
  cfg.type = type;
  return cfg;
}

const PlannerType kAllTypes[] = {PlannerType::kFop, PlannerType::kFopPlus, PlannerType::kFiss,
                                 PlannerType::kFissPlus};

}  // namespace

class AllPlanners : public ::testing::TestWithParam<PlannerType> {};

INSTANTIATE_TEST_SUITE_P(Planners, AllPlanners, ::testing::ValuesIn(kAllTypes),
                         [](const auto& info) { return toString(info.param); });

TEST_P(AllPlanners, FreeRoadKeepsLaneAndReachesTargetSpeed) {
  const auto sc = freeRoad();
  auto planner = makePlanner(configFor(GetParam()), VehicleParams{});
  const auto res = planner->plan(sc.input());
  ASSERT_TRUE(res.success);
  const auto& traj = res.trajectory;
  ASSERT_TRUE(traj.is_cartesian);
  EXPECT_NEAR(traj.d.back(), 0.0, 0.5);
  EXPECT_EQ(traj.lane, 0);
  EXPECT_NEAR(traj.s_d.back(), sc.target_speed, 1.5);
  EXPECT_GT(res.stats.num_generated, 0);
  EXPECT_GE(res.stats.num_validated, 1);
}

TEST_P(AllPlanners, AvoidsStaticObstacleByChangingLane) {
  auto sc = freeRoad();
  sc.obstacles = {box(1, 45.0, 0.0, 0.0)};
  auto planner = makePlanner(configFor(GetParam()), VehicleParams{});
  const auto res = planner->plan(sc.input());
  ASSERT_TRUE(res.success);
  EXPECT_FALSE(collides(res.trajectory, sc.obstacles, planner->vehicle()));
  EXPECT_EQ(res.trajectory.lane, 1);  // overtakes on the left
}

TEST_P(AllPlanners, SolidLineForcesFollowingInsteadOfOvertaking) {
  auto sc = freeRoad();
  sc.corridor.lanes().back().change_allowed = false;  // left lane not reachable
  sc.obstacles = {box(1, 30.0, 0.0, 5.0)};            // slower lead vehicle
  auto planner = makePlanner(configFor(GetParam()), VehicleParams{});
  const auto res = planner->plan(sc.input());
  ASSERT_TRUE(res.success);
  EXPECT_FALSE(collides(res.trajectory, sc.obstacles, planner->vehicle()));
  EXPECT_EQ(res.trajectory.lane, 0);
  for (double d : res.trajectory.d) EXPECT_LT(d, 1.75);
  EXPECT_LT(res.trajectory.s_d.back(), sc.target_speed - 1.0);  // slows down behind the lead
}

TEST_P(AllPlanners, CutInVehicleIsAvoided) {
  auto sc = freeRoad();
  // A vehicle in the left lane merging into ours just ahead of us.
  Obstacle cut;
  cut.id = 7;
  cut.length = 4.5;
  cut.width = 1.9;
  for (double t = 0.0; t <= 6.0; t += 0.1) {
    const double lateral = 3.5 * std::max(0.0, 1.0 - t / 2.0);
    cut.prediction.push_back({t, 22.0 + 7.0 * t, lateral, -0.3 * (t < 2.0), 7.0});
  }
  sc.obstacles = {cut};
  auto planner = makePlanner(configFor(GetParam()), VehicleParams{});
  const auto res = planner->plan(sc.input());
  ASSERT_TRUE(res.success);
  EXPECT_FALSE(collides(res.trajectory, sc.obstacles, planner->vehicle()));
}

TEST_P(AllPlanners, FailsGracefullyWhenBlocked) {
  auto sc = freeRoad();
  sc.start.s_d = 15.0;
  sc.obstacles = {box(1, 16.0, 0.0, 0.0), box(2, 16.0, 3.5, 0.0)};
  auto planner = makePlanner(configFor(GetParam()), VehicleParams{});
  const auto res = planner->plan(sc.input());
  EXPECT_FALSE(res.success);
  EXPECT_GT(res.stats.rejections[static_cast<int>(Rejection::kCollision)] +
                res.stats.rejections[static_cast<int>(Rejection::kAccel)],
            0);
}

TEST_P(AllPlanners, KeepCandidatesReturnsConvertedTrajectories) {
  auto sc = freeRoad();
  auto cfg = configFor(GetParam());
  cfg.keep_candidates = true;
  auto planner = makePlanner(cfg, VehicleParams{});
  const auto res = planner->plan(sc.input());
  ASSERT_TRUE(res.success);
  ASSERT_FALSE(res.candidates.empty());
  for (const auto& c : res.candidates) EXPECT_TRUE(c.is_cartesian);
}

TEST(PlannerInvariants, LazyValidationFindsTheSameOptimumAsExhaustiveSearch) {
  auto sc = freeRoad();
  sc.obstacles = {box(1, 45.0, 0.0, 0.0), box(2, 70.0, 3.5, 3.0)};
  const auto fop = makePlanner(configFor(PlannerType::kFop), VehicleParams{})->plan(sc.input());
  const auto fop_plus = makePlanner(configFor(PlannerType::kFopPlus), VehicleParams{})->plan(sc.input());
  ASSERT_TRUE(fop.success && fop_plus.success);
  EXPECT_NEAR(fop.trajectory.totalCost(), fop_plus.trajectory.totalCost(), 1e-12);
  EXPECT_LT(fop_plus.stats.num_validated, fop.stats.num_validated);
}

TEST(PlannerInvariants, SearchPlannersAreNeverBetterThanTheGridOptimumAndGenerateLess) {
  auto sc = freeRoad();
  sc.obstacles = {box(1, 45.0, 0.0, 0.0)};
  const auto fop = makePlanner(configFor(PlannerType::kFop), VehicleParams{})->plan(sc.input());
  const auto fiss = makePlanner(configFor(PlannerType::kFiss), VehicleParams{})->plan(sc.input());
  auto plus_cfg = configFor(PlannerType::kFissPlus);
  plus_cfg.fiss_plus.refine = false;
  const auto fiss_plus = makePlanner(plus_cfg, VehicleParams{})->plan(sc.input());
  ASSERT_TRUE(fop.success && fiss.success && fiss_plus.success);
  EXPECT_GE(fiss.trajectory.totalCost(), fop.trajectory.totalCost() - 1e-12);
  EXPECT_GE(fiss_plus.trajectory.totalCost(), fop.trajectory.totalCost() - 1e-12);
  EXPECT_LT(fiss.stats.num_generated, fop.stats.num_generated);
  EXPECT_LT(fiss_plus.stats.num_generated, fop.stats.num_generated);
}

TEST(PlannerInvariants, RefinementNeverIncreasesCost) {
  auto sc = freeRoad();
  sc.obstacles = {box(1, 45.0, 0.0, 0.0)};
  auto coarse_cfg = configFor(PlannerType::kFissPlus);
  coarse_cfg.fiss_plus.refine = false;
  const auto coarse = makePlanner(coarse_cfg, VehicleParams{})->plan(sc.input());
  const auto refined = makePlanner(configFor(PlannerType::kFissPlus), VehicleParams{})->plan(sc.input());
  ASSERT_TRUE(coarse.success && refined.success);
  EXPECT_LE(refined.trajectory.totalCost(), coarse.trajectory.totalCost() + 1e-12);
  EXPECT_FALSE(collides(refined.trajectory, sc.obstacles, VehicleParams{}));
}

// Multi-threaded validation must give results identical to serial validation.
TEST(PlannerInvariants, ParallelValidationMatchesSerial) {
  auto sc = freeRoad();
  sc.obstacles = {box(1, 45.0, 0.0, 0.0), box(2, 60.0, 3.5, 2.0)};
  for (auto type : {PlannerType::kFop, PlannerType::kFopPlus}) {
    auto serial_cfg = configFor(type);
    auto parallel_cfg = serial_cfg;
    parallel_cfg.num_threads = 4;
    const auto a = makePlanner(serial_cfg, VehicleParams{})->plan(sc.input());
    const auto b = makePlanner(parallel_cfg, VehicleParams{})->plan(sc.input());
    ASSERT_EQ(a.success, b.success);
    EXPECT_DOUBLE_EQ(a.trajectory.totalCost(), b.trajectory.totalCost());
    EXPECT_EQ(a.trajectory.idx, b.trajectory.idx);
    if (type == PlannerType::kFop) {
      EXPECT_EQ(a.stats.num_collision_checks, b.stats.num_collision_checks);
    }
  }
}

TEST(Planner, HistoryHeuristicIsUsedAndResettable) {
  auto sc = freeRoad();
  auto planner = makePlanner(configFor(PlannerType::kFiss), VehicleParams{});
  const auto first = planner->plan(sc.input());
  const auto second = planner->plan(sc.input());
  ASSERT_TRUE(first.success && second.success);
  EXPECT_LE(second.stats.num_generated, first.stats.num_generated);
  planner->reset();
  const auto third = planner->plan(sc.input());
  EXPECT_EQ(third.stats.num_generated, first.stats.num_generated);
}

TEST(Planner, RequiresReferenceAndCorridor) {
  auto planner = makePlanner(PlannerConfig{}, VehicleParams{});
  EXPECT_THROW(planner->plan(PlanningInput{}), std::invalid_argument);
}
