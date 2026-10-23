// SPDX-License-Identifier: Apache-2.0
// Micro-benchmarks of one planning cycle on a two-lane road with traffic.
#include <benchmark/benchmark.h>

#include "frenet_planner/planning/planner.hpp"

using namespace frenet_planner;

namespace {
struct Fixture {
  ReferenceLine ref;
  Corridor corridor;
  std::vector<Obstacle> obstacles;
  Fixture() {
    std::vector<double> x, y;
    for (double s = 0.0; s <= 400.0; s += 5.0) {
      x.push_back(s);
      y.push_back(10.0 * std::sin(s / 80.0));
    }
    ref = ReferenceLine(x, y);
    corridor = Corridor::parallelLanes(0.0, 400.0, 3.5, 1, 1);
    const double lanes[] = {-3.5, 0.0, 3.5};
    int id = 0;
    for (double s0 : {35.0, 60.0, 90.0, 130.0}) {
      for (double d : lanes) {
        if ((id++ % 2) != 0) continue;
        const auto r = ref.at(s0);
        obstacles.push_back(predictConstantVelocity(id, r.x - std::sin(r.yaw) * d, r.y + std::cos(r.yaw) * d,
                                                    r.yaw, 4.0 + d, 4.5, 1.9, 6.0, 0.1));
      }
    }
  }
};

void runPlanner(benchmark::State& state, PlannerType type, int threads) {
  static const Fixture fx;
  PlannerConfig cfg;
  cfg.type = type;
  cfg.num_threads = threads;
  cfg.sampling.num_width = static_cast<int>(state.range(0));
  cfg.sampling.num_speed = static_cast<int>(state.range(0));
  cfg.sampling.num_t = static_cast<int>(state.range(0));
  auto planner = makePlanner(cfg, VehicleParams{});
  PlanningInput in;
  in.start = {10.0, 10.0, 0.0, 0.0, 0.0, 0.0};
  in.reference = &fx.ref;
  in.corridor = &fx.corridor;
  in.obstacles = &fx.obstacles;
  in.target_speed = 15.0;
  PlanStats last;
  for (auto _ : state) {
    planner->reset();
    auto res = planner->plan(in);
    benchmark::DoNotOptimize(res.success);
    last = res.stats;
  }
  state.counters["generated"] = last.num_generated;
  state.counters["validated"] = last.num_validated;
  state.counters["collision_checks"] = last.num_collision_checks;
}
}  // namespace

BENCHMARK_CAPTURE(runPlanner, fop, PlannerType::kFop, 1)
    ->Arg(5)
    ->Arg(7)
    ->Arg(9)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(runPlanner, fop_x8, PlannerType::kFop, 8)
    ->Arg(5)
    ->Arg(7)
    ->Arg(9)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(runPlanner, fop_plus, PlannerType::kFopPlus, 1)
    ->Arg(5)
    ->Arg(7)
    ->Arg(9)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(runPlanner, fiss, PlannerType::kFiss, 1)
    ->Arg(5)
    ->Arg(7)
    ->Arg(9)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_CAPTURE(runPlanner, fiss_plus, PlannerType::kFissPlus, 1)
    ->Arg(5)
    ->Arg(7)
    ->Arg(9)
    ->Unit(benchmark::kMillisecond);
