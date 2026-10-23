// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/planning/local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "frenet_planner/common/math.hpp"
#include "frenet_planner/geometry/polynomial.hpp"

namespace frenet_planner {

EgoState sampleTrajectory(const Trajectory& traj, double t) {
  if (traj.size() == 0) return {};
  if (t <= traj.t.front()) return traj.cartesianStateAt(0);
  if (t >= traj.t.back()) return traj.cartesianStateAt(traj.size() - 1);
  const auto it = std::upper_bound(traj.t.begin(), traj.t.end(), t);
  const size_t i = static_cast<size_t>(it - traj.t.begin());
  const double w = (t - traj.t[i - 1]) / (traj.t[i] - traj.t[i - 1]);
  const auto lerp = [w](double a, double b) { return a + w * (b - a); };
  EgoState s;
  s.x = lerp(traj.x[i - 1], traj.x[i]);
  s.y = lerp(traj.y[i - 1], traj.y[i]);
  s.yaw = traj.yaw[i - 1] + w * normalizeAngle(traj.yaw[i] - traj.yaw[i - 1]);
  s.v = lerp(traj.v[i - 1], traj.v[i]);
  s.a = lerp(traj.a[i - 1], traj.a[i]);
  s.kappa = lerp(traj.kappa[i - 1], traj.kappa[i]);
  return s;
}

namespace {

/// Points of `traj` with t in [t_from, t_to), re-timed to start at 0.
Trajectory slice(const Trajectory& traj, double t_from, double t_to) {
  Trajectory out;
  for (size_t i = 0; i < traj.size(); ++i) {
    if (traj.t[i] < t_from - 1e-9 || traj.t[i] >= t_to - 1e-9) continue;
    out.t.push_back(traj.t[i] - t_from);
    for (auto [dst, src] : {std::pair{&out.s, &traj.s},
                            {&out.s_d, &traj.s_d},
                            {&out.s_dd, &traj.s_dd},
                            {&out.s_ddd, &traj.s_ddd},
                            {&out.d, &traj.d},
                            {&out.d_d, &traj.d_d},
                            {&out.d_dd, &traj.d_dd},
                            {&out.d_ddd, &traj.d_ddd},
                            {&out.x, &traj.x},
                            {&out.y, &traj.y},
                            {&out.yaw, &traj.yaw},
                            {&out.v, &traj.v},
                            {&out.a, &traj.a},
                            {&out.kappa, &traj.kappa}}) {
      dst->push_back((*src)[i]);
    }
  }
  out.end = traj.end;
  out.lane = traj.lane;
  out.cost = traj.cost;
  out.is_cartesian = traj.is_cartesian;
  return out;
}

/// Append `tail` (re-timed by `offset`) to `head`.
void append(Trajectory& head, const Trajectory& tail, double offset) {
  for (size_t i = 0; i < tail.size(); ++i) {
    head.t.push_back(tail.t[i] + offset);
    for (auto [dst, src] : {std::pair{&head.s, &tail.s},
                            {&head.s_d, &tail.s_d},
                            {&head.s_dd, &tail.s_dd},
                            {&head.s_ddd, &tail.s_ddd},
                            {&head.d, &tail.d},
                            {&head.d_d, &tail.d_d},
                            {&head.d_dd, &tail.d_dd},
                            {&head.d_ddd, &tail.d_ddd},
                            {&head.x, &tail.x},
                            {&head.y, &tail.y},
                            {&head.yaw, &tail.yaw},
                            {&head.v, &tail.v},
                            {&head.a, &tail.a},
                            {&head.kappa, &tail.kappa}}) {
      dst->push_back((*src)[i]);
    }
  }
}

}  // namespace

LocalPlanner::LocalPlanner(const LocalPlannerConfig& config, const VehicleParams& vehicle)
    : config_(config), vehicle_(vehicle), planner_(makePlanner(config.planner, vehicle)) {}

void LocalPlanner::setMap(std::shared_ptr<const RoadMap> map, std::vector<int> route) {
  map_ = map ? std::make_shared<RoadMap>(*map) : nullptr;
  route_ = std::move(route);
  path_ref_.reset();
  reset();
}

void LocalPlanner::setReferencePath(const std::vector<double>& x, const std::vector<double>& y) {
  auto ref = std::make_shared<ReferenceLine>(x, y);
  const auto& p = config_.path;
  path_corridor_ =
      Corridor::parallelLanes(0.0, ref->length(), p.lane_width, p.num_left_lanes, p.num_right_lanes);
  for (auto& lane : path_corridor_.lanes()) {
    if (lane.offset > 0) lane.change_allowed = p.allow_left;
    if (lane.offset < 0) lane.change_allowed = p.allow_right;
  }
  path_ref_ = std::move(ref);
  map_.reset();
  route_.clear();
  reset();
}

void LocalPlanner::requestConfig(const LocalPlannerConfig& config) {
  config.planner.validate();
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_config_ = config;
}

void LocalPlanner::requestPlannerType(PlannerType type) {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  if (!pending_config_) pending_config_ = config_;
  pending_config_->planner.type = type;
}

void LocalPlanner::setStopLineActive(int stop_line_id, bool active) {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_stop_lines_.emplace_back(stop_line_id, active);
}

void LocalPlanner::reset() {
  previous_.reset();
  previous_emergency_ = false;
  previous_ego_lane_ = -1;
  planner_->reset();
}

void LocalPlanner::applyPendingConfig() {
  std::optional<LocalPlannerConfig> next;
  std::vector<std::pair<int, bool>> stop_lines;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    next.swap(pending_config_);
    stop_lines.swap(pending_stop_lines_);
  }
  if (map_) {
    for (const auto& [id, active] : stop_lines) map_->setStopLineActive(id, active);
  }
  if (!next) return;
  const bool type_changed = next->planner.type != config_.planner.type;
  const bool path_changed = next->path.lane_width != config_.path.lane_width ||
                            next->path.num_left_lanes != config_.path.num_left_lanes ||
                            next->path.num_right_lanes != config_.path.num_right_lanes ||
                            next->path.allow_left != config_.path.allow_left ||
                            next->path.allow_right != config_.path.allow_right;
  config_ = *next;
  if (type_changed) {
    planner_ = makePlanner(config_.planner, vehicle_);
  } else {
    planner_->setConfig(config_.planner);
  }
  if (path_changed && path_ref_) {
    const auto& p = config_.path;
    path_corridor_ =
        Corridor::parallelLanes(0.0, path_ref_->length(), p.lane_width, p.num_left_lanes, p.num_right_lanes);
    for (auto& lane : path_corridor_.lanes()) {
      if (lane.offset > 0) lane.change_allowed = p.allow_left;
      if (lane.offset < 0) lane.change_allowed = p.allow_right;
    }
  }
}

double LocalPlanner::targetSpeed(const ReferenceLine& ref, double s, double v, double desired,
                                 double lane_limit) const {
  double target = desired;
  if (config_.use_speed_limit && std::isfinite(lane_limit) && lane_limit > 0.0) {
    target = std::min(target, lane_limit);
  }
  if (config_.use_curvature_speed_limit) {
    const double horizon = std::max({v, target, 5.0}) * config_.planner.sampling.max_t;
    for (double ds = 0.0; ds <= horizon; ds += 2.0) {
      const double k = std::abs(ref.at(s + ds).kappa);
      if (k > 1e-4) target = std::min(target, std::sqrt(vehicle_.max_lat_accel / k));
    }
  }
  return std::min(target, vehicle_.max_speed);
}

Trajectory LocalPlanner::emergencyStop(const FrenetState& start, const ReferenceLine& ref) const {
  // Brake at the limit while bringing the lateral motion to rest at the
  // current offset (quintic over 1.5 s), until standstill plus one second.
  const double decel = config_.emergency_decel > 0.0 ? config_.emergency_decel : vehicle_.max_decel;
  const double dt = config_.planner.sampling.dt;
  const double v0 = std::max(0.0, start.s_d);
  const double t_stop = v0 / decel;
  const double T_lat = 1.5;
  const double d_end = start.d + start.d_d * 0.5 * T_lat;
  const QuinticPolynomial lat(start.d, start.d_d, start.d_dd, d_end, 0.0, 0.0, T_lat);
  Trajectory traj;
  traj.end = {d_end, 0.0, t_stop + 1.0};
  for (double t : timeSamples(std::max(t_stop + 1.0, T_lat), dt)) {
    const double tb = std::min(t, t_stop);
    traj.t.push_back(t);
    traj.s.push_back(start.s + v0 * tb - 0.5 * decel * tb * tb);
    traj.s_d.push_back(std::max(0.0, v0 - decel * t));
    traj.s_dd.push_back(t < t_stop ? -decel : 0.0);
    traj.s_ddd.push_back(0.0);
    const double tl = std::min(t, T_lat);
    traj.d.push_back(lat.x(tl));
    traj.d_d.push_back(t < T_lat ? lat.dx(tl) : 0.0);
    traj.d_dd.push_back(t < T_lat ? lat.ddx(tl) : 0.0);
    traj.d_ddd.push_back(t < T_lat ? lat.dddx(tl) : 0.0);
  }
  convertToCartesian(ref, traj);
  return traj;
}

LocalPlanOutput LocalPlanner::step(double t_now, const EgoState& ego, const std::vector<Obstacle>& obstacles,
                                   double desired_speed) {
  applyPendingConfig();
  LocalPlanOutput out;
  const auto& st = config_.stitching;
  const double latency = st.enabled ? std::max(0.0, st.latency) : 0.0;

  // --- Start state (stitching) ------------------------------------------------
  EgoState start = ego;
  std::optional<Trajectory> prefix;
  if (previous_emergency_) {
    // Recover from an emergency stop from the measured state: stitching onto the
    // stop profile would carry its full braking into every new plan.
    start.a = std::max(start.a, -config_.recovery_decel);
  } else if (st.enabled && previous_ && previous_->size() > 1) {
    const double t_rel = t_now - previous_t0_;
    if (t_rel + latency <= previous_->t.back()) {
      const EgoState expected = sampleTrajectory(*previous_, t_rel);
      const double pos_err = std::hypot(expected.x - ego.x, expected.y - ego.y);
      const double yaw_err = std::abs(normalizeAngle(expected.yaw - ego.yaw));
      if (pos_err <= st.max_position_error && yaw_err <= st.max_heading_error) {
        start = sampleTrajectory(*previous_, t_rel + latency);
        if (latency > 0.0) prefix = slice(*previous_, t_rel, t_rel + latency);
        out.stitched = true;
      }
    }
  }

  // --- Reference and corridor -------------------------------------------------
  double lane_limit = std::numeric_limits<double>::infinity();
  if (map_) {
    auto rc = buildRouteCorridor(*map_, route_, start, config_.route, previous_ego_lane_);
    if (!rc) {
      previous_.reset();
      return out;  // off the map
    }
    out.reference = std::make_shared<ReferenceLine>(std::move(rc->reference));
    out.corridor = std::move(rc->corridor);
    out.ego_lane = rc->ego_lane;
    out.virtual_obstacles = std::move(rc->stop_lines);
    lane_limit = rc->speed_limit;
    previous_ego_lane_ = rc->ego_lane;
  } else if (path_ref_) {
    out.reference = path_ref_;
    out.corridor = path_corridor_;
  } else {
    throw std::logic_error("LocalPlanner::step: call setMap() or setReferencePath() first");
  }
  const ReferenceLine& ref = *out.reference;

  out.start = ref.toFrenet(start);
  out.start.s_d = std::max(0.0, out.start.s_d);
  const double desired = desired_speed > 0.0 ? desired_speed : config_.desired_speed;
  out.target_speed = targetSpeed(ref, out.start.s, start.v, desired, lane_limit);

  // --- Obstacles relative to the start time -----------------------------------
  std::vector<Obstacle> all = obstacles;
  all.insert(all.end(), out.virtual_obstacles.begin(), out.virtual_obstacles.end());
  if (latency > 0.0) {
    for (auto& ob : all) {
      if (ob.is_static) continue;
      for (auto& p : ob.prediction) p.t -= latency;
    }
  }

  PlanningInput in;
  in.start = out.start;
  in.reference = &ref;
  in.corridor = &out.corridor;
  in.obstacles = &all;
  in.target_speed = out.target_speed;
  PlanResult res = planner_->plan(in);
  out.stats = res.stats;
  out.candidates = std::move(res.candidates);

  // Keep executing what is left of the last plan if it is still collision
  // free; failing that, retry without the safety envelope (a close but
  // collision-free plan beats hard braking in traffic), then with a shorter
  // collision horizon; otherwise brake. Planning starts afresh next cycle.
  std::optional<Trajectory> rest;
  if (!res.success && previous_) {
    const double t_rel = t_now - previous_t0_;
    if (t_rel < previous_->t.back() - config_.planner.sampling.dt) {
      Trajectory candidate = slice(*previous_, t_rel, previous_->t.back() + 1.0);
      int checks = 0;
      const CollisionChecker checker(config_.planner.collision, vehicle_);
      if (checker.isCollisionFree(candidate, obstacles, checks)) rest = std::move(candidate);
    }
  }
  const auto retry = [&](bool envelope, double horizon) {
    in.safety_envelope = envelope;
    in.collision_horizon = horizon;
    PlanResult r = planner_->plan(in);
    out.stats.time_ms += r.stats.time_ms;
    if (!r.success) return;
    out.degraded = true;
    res.success = true;
    res.trajectory = std::move(r.trajectory);
  };
  const bool envelope_rejected = res.stats.rejections[static_cast<int>(Rejection::kSafetyDistance)] > 0;
  if (!res.success && !rest && envelope_rejected && config_.planner.safety.enabled) {
    retry(false, std::numeric_limits<double>::infinity());
  }
  if (!res.success && !rest && config_.degraded_horizon > 0.0) retry(false, config_.degraded_horizon);

  if (res.success) {
    out.success = true;
    if (config_.smoother.enabled) {
      out.smoothed = smoothTrajectory(res.trajectory, in, config_.planner, vehicle_, config_.smoother);
    }
    if (prefix) {
      out.trajectory = std::move(*prefix);
      append(out.trajectory, res.trajectory, latency);
      out.trajectory.end = res.trajectory.end;
      out.trajectory.lane = res.trajectory.lane;
      out.trajectory.cost = res.trajectory.cost;
      out.trajectory.idx = res.trajectory.idx;
    } else {
      out.trajectory = std::move(res.trajectory);
    }
    previous_ = out.trajectory;
    previous_t0_ = t_now;
    previous_emergency_ = false;
  } else if (rest) {
    out.trajectory = std::move(*rest);
    out.fallback = true;
  } else {
    out.trajectory = emergencyStop(out.start, ref);
    out.emergency = true;
    previous_ = out.trajectory;
    previous_t0_ = t_now;
    previous_emergency_ = true;
  }
  return out;
}

}  // namespace frenet_planner
