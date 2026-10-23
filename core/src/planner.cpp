// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/planning/planner.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "planners.hpp"

namespace frenet_planner {

namespace {
const std::vector<Obstacle> kNoObstacles;

void accumulate(PlanStats& into, const PlanStats& from) {
  into.num_validated += from.num_validated;
  into.num_collision_checks += from.num_collision_checks;
  for (size_t i = 0; i < into.rejections.size(); ++i) into.rejections[i] += from.rejections[i];
}
}  // namespace

Planner::Planner(const PlannerConfig& config, const VehicleParams& vehicle)
    : config_(config),
      vehicle_(vehicle),
      cost_(config.cost, config.sampling),
      constraints_(config.constraints, vehicle),
      collisions_(config.collision, vehicle) {
  config_.validate();
}

void Planner::setConfig(const PlannerConfig& config) {
  config.validate();
  config_ = config;
  cost_ = CostEvaluator(config.cost, config.sampling);
  constraints_ = ConstraintValidator(config.constraints, vehicle_);
  collisions_ = CollisionChecker(config.collision, vehicle_);
}

PlanResult Planner::plan(const PlanningInput& input) {
  PlanResult result;
  const auto t0 = Clock::now();
  if (input.reference == nullptr || input.reference->empty() || input.corridor == nullptr ||
      input.corridor->empty()) {
    throw std::invalid_argument("Planner::plan: reference line and corridor are required");
  }

  // Lane geometry is evaluated roughly where the trajectories end.
  const double t_mid = 0.5 * (config_.sampling.min_t + config_.sampling.max_t);
  const double s_lookahead = input.start.s + std::max(0.0, input.start.s_d) * t_mid;
  const EndStateGrid grid(config_.sampling, *input.corridor, vehicle_, s_lookahead, input.target_speed);

  Context ctx{input, grid, t0, result.stats};
  Trajectory best;
  result.success = search(ctx, best, config_.keep_candidates ? &result.candidates : nullptr);
  if (result.success) {
    prev_best_ = best.end;
    result.trajectory = std::move(best);
  } else {
    prev_best_.reset();
  }
  result.stats.time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  return result;
}

Trajectory Planner::generate(Context& ctx, int i, int j, int k) const {
  Trajectory traj = generate(ctx, ctx.grid.endState(i, j, k));
  traj.idx = {i, j, k};
  return traj;
}

Trajectory Planner::generate(Context& ctx, const EndState& end) const {
  Trajectory traj;
  generateFrenet(ctx.input.start, end, config_.sampling.dt, traj);
  const LateralSample lat = ctx.grid.lateralInfo(end.d);
  traj.lane = lat.lane;
  traj.cost = cost_.evaluate(traj, lat, ctx.input.target_speed);
  ++ctx.stats.num_generated;
  return traj;
}

double Planner::estimate(const Context& ctx, int i, int j, int k) const {
  const EndState end = ctx.grid.endState(i, j, k);
  double cost = cost_.estimate(end, ctx.grid.lateral()[i], ctx.input.target_speed);
  if (prev_best_) {
    // History heuristic of FISS, in normalized continuous coordinates so that
    // it stays meaningful when the grid changes between cycles.
    const auto lo = ctx.grid.lower(), hi = ctx.grid.upper();
    const double values[3] = {end.d, end.s_d, end.T};
    const double prev[3] = {prev_best_->d, prev_best_->s_d, prev_best_->T};
    double dist2 = 0.0;
    for (int dim = 0; dim < 3; ++dim) {
      const double range = hi[dim] - lo[dim];
      if (range > 1e-9) dist2 += std::pow((values[dim] - prev[dim]) / range, 2);
    }
    cost += config_.cost.w_heuristic * dist2 / 3.0;
  }
  return cost;
}

bool Planner::validate(const Context& ctx, Trajectory& traj, PlanStats& stats) const {
  ++stats.num_validated;
  if (!traj.is_cartesian && !convertToCartesian(*ctx.input.reference, traj)) {
    traj.rejection = Rejection::kInvalid;
  } else {
    traj.rejection = constraints_.check(traj, ctx.input.corridor);
  }
  if (traj.rejection == Rejection::kOk) {
    const auto& obstacles = ctx.input.obstacles ? *ctx.input.obstacles : kNoObstacles;
    if (!collisions_.isCollisionFree(traj, obstacles, stats.num_collision_checks)) {
      traj.rejection = Rejection::kCollision;
    }
  }
  ++stats.rejections[static_cast<int>(traj.rejection)];
  return traj.rejection == Rejection::kOk;
}

std::vector<char> Planner::validateAll(const Context& ctx, std::vector<Trajectory>& trajs,
                                       PlanStats& stats) const {
  std::vector<char> ok(trajs.size(), 0);
  const int threads = std::min<int>(std::max(1, config_.num_threads), static_cast<int>(trajs.size()));
  if (threads <= 1) {
    for (size_t i = 0; i < trajs.size(); ++i) ok[i] = validate(ctx, trajs[i], stats);
    return ok;
  }
  std::vector<PlanStats> local(static_cast<size_t>(threads));
  std::atomic<size_t> next{0};
  std::vector<std::thread> workers;
  workers.reserve(static_cast<size_t>(threads));
  for (int w = 0; w < threads; ++w) {
    workers.emplace_back([&, w] {
      for (size_t i = next++; i < trajs.size(); i = next++) {
        ok[i] = validate(ctx, trajs[i], local[static_cast<size_t>(w)]);
      }
    });
  }
  for (auto& t : workers) t.join();
  for (const auto& s : local) accumulate(stats, s);
  return ok;
}

double Planner::elapsedSeconds(const Context& ctx) const {
  return std::chrono::duration<double>(Clock::now() - ctx.start_time).count();
}

std::unique_ptr<Planner> makePlanner(const PlannerConfig& config, const VehicleParams& vehicle) {
  switch (config.type) {
    case PlannerType::kFop:
      return std::make_unique<FopPlanner>(config, vehicle);
    case PlannerType::kFopPlus:
      return std::make_unique<FopPlusPlanner>(config, vehicle);
    case PlannerType::kFiss:
      return std::make_unique<FissPlanner>(config, vehicle);
    case PlannerType::kFissPlus:
      return std::make_unique<FissPlusPlanner>(config, vehicle);
  }
  throw std::invalid_argument("makePlanner: unknown planner type");
}

}  // namespace frenet_planner
