// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/planning/planner.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "planners.hpp"

namespace fiss_planner {

namespace {
const std::vector<Obstacle> kNoObstacles;

void accumulate(PlanStats& into, const PlanStats& from) {
  into.num_validated += from.num_validated;
  into.num_collision_checks += from.num_collision_checks;
  for (size_t i = 0; i < into.rejections.size(); ++i) into.rejections[i] += from.rejections[i];
  if (from.blocking_cost < into.blocking_cost) {
    into.blocking_id = from.blocking_id;
    into.blocking_t = from.blocking_t;
    into.blocking_cost = from.blocking_cost;
  }
}
}  // namespace

Planner::Planner(const PlannerConfig& config, const VehicleParams& vehicle)
    : config_(config),
      vehicle_(vehicle),
      cost_(config.cost, config.sampling, vehicle, config.collision.margin_lat),
      constraints_(config.constraints, vehicle),
      collisions_(config.collision, vehicle),
      gaps_(config.cost, config.safety, vehicle, config.collision.margin_lat) {
  config_.validate();
}

void Planner::setConfig(const PlannerConfig& config) {
  config.validate();
  config_ = config;
  cost_ = CostEvaluator(config.cost, config.sampling, vehicle_, config.collision.margin_lat);
  constraints_ = ConstraintValidator(config.constraints, vehicle_);
  collisions_ = CollisionChecker(config.collision, vehicle_);
  gaps_ = GapEvaluator(config.cost, config.safety, vehicle_, config.collision.margin_lat);
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

  // Obstacles that can matter: from behind the ego (for the rear rule) to
  // beyond the farthest reachable station, within the road plus a margin.
  const auto& obstacles = input.obstacles ? *input.obstacles : kNoObstacles;
  const auto& sc = config_.sampling;
  const double reach = std::max({input.start.s_d, input.target_speed, 5.0}) * sc.max_t;
  const double s_lo = input.start.s - 80.0, s_hi = input.start.s + reach + 60.0;
  double d_lo = std::numeric_limits<double>::infinity(), d_hi = -d_lo;
  for (double s = input.start.s; s <= s_hi; s += 20.0) {
    const auto [lo, hi] = input.corridor->roadBounds(s);
    d_lo = std::min(d_lo, lo - 3.0);
    d_hi = std::max(d_hi, hi + 3.0);
  }
  const FrenetObstacles frenet(obstacles, *input.reference, s_lo, s_hi, d_lo, d_hi, sc.max_t);
  std::vector<char> rear_in_lane;
  if (config_.collision.ignore_rear_in_lane) {
    if (const LaneBand* ego = input.corridor->egoLane()) {
      rear_in_lane =
          rearInLane(frenet, obstacles.size(), input.start.s, 0.5 * vehicle_.length,
                     input.corridor->right(*ego, input.start.s), input.corridor->left(*ego, input.start.s));
    }
  }
  result.stats.lead = gaps_.leadGap(input.start.s, input.start.d, input.start.s_d, frenet);

  // Car following: per lane, the speed that closes the gap to the vehicle
  // ahead to the desired one within follow_time.
  std::vector<std::pair<int, double>> lane_speeds;
  const auto& w = config_.cost;
  if (w.follow_time > 0.0) {
    for (const auto& lane : input.corridor->lanes()) {
      const double right = input.corridor->right(lane, input.start.s);
      const double left = input.corridor->left(lane, input.start.s);
      const double center = input.corridor->center(lane, input.start.s);
      const LeadGap lead = gaps_.leadGap(input.start.s, center, input.start.s_d, frenet, 0.0, right, left);
      double speed = input.target_speed;
      if (lead.id >= 0) {
        const double desired = w.desired_gap + w.desired_time_headway * lead.speed;
        speed = std::clamp(lead.speed + (lead.gap - desired) / w.follow_time, 0.0, input.target_speed);
      }
      lane_speeds.emplace_back(lane.offset, speed);
    }
  }

  Context ctx{input, grid, t0, result.stats, frenet, rear_in_lane, lane_speeds};
  if (const LaneBand* ego = input.corridor->egoLane())
    ctx.deficit_weight = 1.0 - std::clamp(ego->urgency, 0.0, 1.0);
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
  LateralSample lat = ctx.grid.lateralInfo(end.d);
  const double target = laneTerms(ctx, lat);
  traj.lane = lat.lane;
  traj.cost = cost_.evaluate(traj, lat, target, &ctx.obstacles);
  ++ctx.stats.num_generated;
  return traj;
}

double Planner::estimate(const Context& ctx, int i, int j, int k) const {
  const EndState end = ctx.grid.endState(i, j, k);
  LateralSample lat = ctx.grid.lateral()[i];
  const double target = laneTerms(ctx, lat);
  double cost = cost_.estimate(end, lat, target);
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

double Planner::laneTerms(const Context& ctx, LateralSample& lat) const {
  const double target = ctx.targetSpeed(lat.lane);
  const double deficit = ctx.input.target_speed - target;
  lat.lane_cost += ctx.deficit_weight * config_.cost.w_speed * deficit * deficit;
  return target;
}

bool Planner::validate(const Context& ctx, Trajectory& traj, PlanStats& stats) const {
  ++stats.num_validated;
  if (!traj.is_cartesian && !convertToCartesian(*ctx.input.reference, traj)) {
    traj.rejection = Rejection::kInvalid;
  } else {
    traj.rejection = constraints_.check(traj, ctx.input.corridor);
  }
  const double horizon = ctx.input.collision_horizon;
  if (traj.rejection == Rejection::kOk && ctx.input.safety_envelope &&
      !gaps_.isSafe(traj, ctx.obstacles, horizon)) {
    traj.rejection = Rejection::kSafetyDistance;
  }
  if (traj.rejection == Rejection::kOk) {
    const auto& obstacles = ctx.input.obstacles ? *ctx.input.obstacles : kNoObstacles;
    const bool skip_rear = traj.lane == 0 && !ctx.rear_in_lane.empty();
    if (!collisions_.isCollisionFree(traj, obstacles, stats.num_collision_checks, &traj.blocking_id,
                                     &traj.blocking_t, horizon, skip_rear ? &ctx.rear_in_lane : nullptr)) {
      traj.rejection = Rejection::kCollision;
    } else if (ctx.input.safety_envelope && config_.safety.enabled &&
               !collisions_.keepsTimeGap(traj, obstacles, config_.safety.crossing_time_gap,
                                         stats.num_collision_checks, &traj.blocking_id, &traj.blocking_t,
                                         horizon)) {
      traj.rejection = Rejection::kSafetyDistance;
    }
  }
  if ((traj.rejection == Rejection::kCollision || traj.rejection == Rejection::kSafetyDistance) &&
      traj.totalCost() < stats.blocking_cost) {
    stats.blocking_id = traj.blocking_id;
    stats.blocking_t = traj.blocking_t;
    stats.blocking_cost = traj.totalCost();
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

}  // namespace fiss_planner
