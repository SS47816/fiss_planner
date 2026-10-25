// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/planning/components.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fiss_planner/common/math.hpp"
#include "fiss_planner/geometry/polynomial.hpp"

namespace fiss_planner {

// ----------------------------------------------------------------------------
// EndStateGrid
// ----------------------------------------------------------------------------

EndStateGrid::EndStateGrid(const SamplingConfig& cfg, const Corridor& corridor, const VehicleParams& vehicle,
                           double s_lookahead, double target_speed) {
  if (corridor.empty()) throw std::invalid_argument("EndStateGrid: empty corridor");

  double min_step = std::numeric_limits<double>::infinity();
  for (const auto& lane : corridor.lanes()) {
    if (!lane.change_allowed && lane.offset != 0) continue;
    const double right = corridor.right(lane, s_lookahead);
    const double left = corridor.left(lane, s_lookahead);
    const double center = corridor.center(lane, s_lookahead);
    // Adjacent lanes too narrow for the vehicle (ending/merging) are not targets.
    if (lane.offset != 0 && left - right < vehicle.width) continue;
    lanes_.push_back(lane);
    lane_left_.push_back(left);
    lane_right_.push_back(right);

    // Keep the whole footprint (plus margin) inside the lane where possible.
    const double half_range = 0.5 * (left - right - vehicle.width) - cfg.lane_margin;
    if (half_range < 1e-3 || cfg.num_width <= 1) {
      lateral_.push_back({center, lane.offset, center, lane.extra_cost});
      continue;
    }
    const auto ds = linspace(center - half_range, center + half_range, cfg.num_width);
    for (double d : ds) lateral_.push_back({d, lane.offset, center, lane.extra_cost});
    min_step = std::min(min_step, 2.0 * half_range / (cfg.num_width - 1));
  }
  if (lateral_.empty()) throw std::invalid_argument("EndStateGrid: no allowed lane");
  std::stable_sort(lateral_.begin(), lateral_.end(),
                   [](const LateralSample& a, const LateralSample& b) { return a.d < b.d; });

  const double v_hi = std::max(target_speed, cfg.min_speed);
  speeds_ = linspace(cfg.min_speed, v_hi, std::max(1, cfg.num_speed));
  if (v_hi - cfg.min_speed < 1e-6) speeds_.assign(1, v_hi);
  times_ = linspace(cfg.min_t, cfg.max_t, std::max(1, cfg.num_t));
  if (cfg.max_t - cfg.min_t < 1e-6) times_.assign(1, cfg.max_t);

  lower_ = {lateral_.front().d, speeds_.front(), times_.front()};
  upper_ = {lateral_.back().d, speeds_.back(), times_.back()};
  const auto step = [](const std::vector<double>& v) {
    return v.size() > 1 ? (v.back() - v.front()) / static_cast<double>(v.size() - 1) : 0.0;
  };
  resolution_ = {std::isfinite(min_step) ? min_step : 0.0, step(speeds_), step(times_)};
}

std::array<int, 3> EndStateGrid::unflat(int f) const {
  const auto n = dims();
  return {f / (n[1] * n[2]), (f / n[2]) % n[1], f % n[2]};
}

LateralSample EndStateGrid::lateralInfo(double d) const {
  size_t best = 0;
  double best_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < lanes_.size(); ++i) {
    // Distance outside the band (0 inside); ties resolved by center distance.
    const double outside = std::max({0.0, lane_right_[i] - d, d - lane_left_[i]});
    const double center = 0.5 * (lane_left_[i] + lane_right_[i]);
    const double dist = outside * 1e3 + std::abs(d - center);
    if (dist < best_dist) {
      best_dist = dist;
      best = i;
    }
  }
  const double center = 0.5 * (lane_left_[best] + lane_right_[best]);
  return {d, lanes_[best].offset, center, lanes_[best].extra_cost};
}

// ----------------------------------------------------------------------------
// Trajectory generation
// ----------------------------------------------------------------------------

void generateFrenet(const FrenetState& start, const EndState& end, double dt, Trajectory& traj) {
  const QuinticPolynomial lat(start.d, start.d_d, start.d_dd, end.d, 0.0, 0.0, end.T);
  const QuarticPolynomial lon(start.s, start.s_d, start.s_dd, end.s_d, 0.0, end.T);

  traj.end = end;
  traj.t = timeSamples(end.T, dt);
  const size_t n = traj.t.size();
  for (auto* v : {&traj.s, &traj.s_d, &traj.s_dd, &traj.s_ddd, &traj.d, &traj.d_d, &traj.d_dd, &traj.d_ddd}) {
    v->resize(n);
  }
  for (size_t i = 0; i < n; ++i) {
    const double t = traj.t[i];
    traj.s[i] = lon.x(t);
    traj.s_d[i] = lon.dx(t);
    traj.s_dd[i] = lon.ddx(t);
    traj.s_ddd[i] = lon.dddx(t);
    traj.d[i] = lat.x(t);
    traj.d_d[i] = lat.dx(t);
    traj.d_dd[i] = lat.ddx(t);
    traj.d_ddd[i] = lat.dddx(t);
  }
  traj.is_cartesian = false;
}

bool convertToCartesian(const ReferenceLine& ref, Trajectory& traj) {
  const size_t n = traj.size();
  for (auto* v : {&traj.x, &traj.y, &traj.yaw, &traj.v, &traj.a, &traj.kappa}) v->resize(n);
  bool finite = true;
  for (size_t i = 0; i < n; ++i) {
    const EgoState st = ref.toCartesian(traj.frenetStateAt(i));
    traj.x[i] = st.x;
    traj.y[i] = st.y;
    traj.yaw[i] = st.yaw;
    traj.v[i] = st.v;
    traj.a[i] = st.a;
    traj.kappa[i] = st.kappa;
    finite = finite && std::isfinite(st.x) && std::isfinite(st.y) && std::isfinite(st.yaw) &&
             std::isfinite(st.kappa);
  }
  traj.is_cartesian = true;
  return finite;
}

// ----------------------------------------------------------------------------
// Cost
// ----------------------------------------------------------------------------

double CostEvaluator::estimate(const EndState& end, const LateralSample& lat, double target_speed) const {
  const double dv = end.s_d - target_speed;
  const double dd = end.d - lat.lane_center;
  return w_.w_time * (max_t_ - end.T) + w_.w_speed * dv * dv + w_.w_lateral * dd * dd +
         w_.w_lane_change * std::abs(lat.lane) + lat.lane_cost;
}

CostTerms CostEvaluator::evaluate(const Trajectory& traj, const LateralSample& lat, double target_speed,
                                  const FrenetObstacles* obstacles) const {
  // Terminal terms (time, speed, lateral offset, lane) equal the estimate;
  // comfort terms are mean squared acceleration and jerk.
  const size_t n = traj.size();
  double accel = 0.0, jerk = 0.0;
  for (size_t i = 0; i < n; ++i) {
    accel += traj.s_dd[i] * traj.s_dd[i] + traj.d_dd[i] * traj.d_dd[i];
    jerk += traj.s_ddd[i] * traj.s_ddd[i] + traj.d_ddd[i] * traj.d_ddd[i];
  }
  const double inv_n = n > 0 ? 1.0 / static_cast<double>(n) : 0.0;
  const double dv = (n > 0 ? traj.s_d.back() : traj.end.s_d) - target_speed;
  const double dd = (n > 0 ? traj.d.back() : traj.end.d) - lat.lane_center;
  CostTerms c;
  c.time = w_.w_time * (max_t_ - traj.end.T);
  c.speed = w_.w_speed * dv * dv;
  c.accel = w_.w_accel * accel * inv_n;
  c.jerk = w_.w_jerk * jerk * inv_n;
  c.lateral = w_.w_lateral * dd * dd;
  c.lane_change = w_.w_lane_change * std::abs(lat.lane) + lat.lane_cost;
  if (obstacles != nullptr) c.proximity = gaps_.cost(traj, *obstacles);
  return c;
}

// ----------------------------------------------------------------------------
// Constraints
// ----------------------------------------------------------------------------

Rejection ConstraintValidator::check(const Trajectory& traj, const Corridor* corridor) const {
  if (!traj.is_cartesian) return Rejection::kInvalid;
  const double kappa_max = vehicle_.maxCurvature();
  const size_t n = traj.size();
  // A start state at or beyond an acceleration limit (e.g. stitched onto a
  // full stop, whose Frenet deceleration exceeds the limit slightly on curves)
  // may not get worse, so the planner can take over instead of failing.
  constexpr double kAccelSlack = 0.05;  // [m/s^2]
  const double s_dd0 = n > 0 ? traj.s_dd[0] : 0.0;
  const double max_accel = std::max(vehicle_.max_accel, s_dd0 + kAccelSlack);
  const double max_decel = std::max(vehicle_.max_decel, -s_dd0 + kAccelSlack);
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(traj.x[i]) || !std::isfinite(traj.y[i]) || !std::isfinite(traj.s_d[i])) {
      return Rejection::kInvalid;
    }
    if (traj.v[i] > vehicle_.max_speed || traj.s_d[i] < -0.1) return Rejection::kSpeed;
    if (traj.s_dd[i] > max_accel || traj.s_dd[i] < -max_decel) return Rejection::kAccel;
    if (traj.v[i] >= cfg_.curvature_check_min_speed) {
      if (cfg_.check_curvature && std::abs(traj.kappa[i]) > kappa_max) return Rejection::kCurvature;
      if (cfg_.check_lat_accel && traj.v[i] * traj.v[i] * std::abs(traj.kappa[i]) > vehicle_.max_lat_accel) {
        return Rejection::kLatAccel;
      }
    }
  }

  if (cfg_.check_road_bounds && corridor != nullptr && !corridor->empty()) {
    // Lateral half-extent of the footprint given its heading relative to the
    // reference line. A start state that already violates the bounds may not
    // get (much) worse, so the planner can recover instead of deadlocking; the
    // slack covers the corner swinging out while the vehicle turns back in.
    constexpr double kRecoverySlack = 0.25;  // [m]
    double allowed = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double rel_yaw = std::atan2(traj.d_d[i], std::max(std::abs(traj.s_d[i]), 1e-3));
      const double half = 0.5 * (vehicle_.width * std::abs(std::cos(rel_yaw)) +
                                 vehicle_.length * std::abs(std::sin(rel_yaw)));
      const auto [lo, hi] = corridor->roadBounds(traj.s[i]);
      const double violation = std::max(lo - (traj.d[i] - half), (traj.d[i] + half) - hi);
      if (i == 0 && violation > 0.0) allowed = violation + kRecoverySlack;
      if (violation > allowed + 1e-6) return Rejection::kOffRoad;
    }
  }
  return Rejection::kOk;
}

// ----------------------------------------------------------------------------
// Collisions
// ----------------------------------------------------------------------------

bool CollisionChecker::isCollisionFree(const Trajectory& traj, const std::vector<Obstacle>& obstacles,
                                       int& num_checks, int* hit_id, double* hit_t, double max_t,
                                       const std::vector<char>* skip) const {
  if (obstacles.empty()) return true;
  const double len = vehicle_.length + 2.0 * cfg_.margin_lon;
  const double wid = vehicle_.width + 2.0 * cfg_.margin_lat;
  const size_t stride = static_cast<size_t>(std::max(1, cfg_.stride));
  const size_t n = traj.size();
  for (size_t i = 0; i < n && traj.t[i] <= max_t; i += stride) {
    const Box2d ego{traj.x[i], traj.y[i], traj.yaw[i], len, wid};
    for (size_t k = 0; k < obstacles.size(); ++k) {
      if (skip != nullptr && k < skip->size() && (*skip)[k]) continue;
      const auto box = obstacles[k].boxAt(traj.t[i]);
      if (!box) continue;
      ++num_checks;
      if (overlaps(ego, *box)) {
        if (hit_id != nullptr) *hit_id = obstacles[k].id;
        if (hit_t != nullptr) *hit_t = traj.t[i];
        return false;
      }
    }
  }
  return true;
}

bool CollisionChecker::keepsTimeGap(const Trajectory& traj, const std::vector<Obstacle>& obstacles,
                                    double gap, int& num_checks, int* hit_id, double* hit_t,
                                    double max_t) const {
  const size_t n = traj.size();
  if (gap <= 0.0 || obstacles.empty() || n == 0) return true;
  const double len = vehicle_.length + 2.0 * cfg_.margin_lon;
  const double wid = vehicle_.width + 2.0 * cfg_.margin_lat;
  const size_t stride = static_cast<size_t>(std::max(1, cfg_.stride));
  constexpr double kGapFractions[] = {-1.0, -0.5, 0.5, 1.0};
  // Bounds of the ego's path, to skip vehicles that never come near it.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  double x_lo = kInf, x_hi = -kInf, y_lo = kInf, y_hi = -kInf, t_hi = traj.t[0];
  for (size_t i = 0; i < n && traj.t[i] <= max_t; ++i) {
    x_lo = std::min(x_lo, traj.x[i]);
    x_hi = std::max(x_hi, traj.x[i]);
    y_lo = std::min(y_lo, traj.y[i]);
    y_hi = std::max(y_hi, traj.y[i]);
    t_hi = traj.t[i];
  }
  const double r_ego = 0.5 * std::hypot(len, wid);
  for (const Obstacle& ob : obstacles) {
    if (ob.is_static || ob.prediction.size() < 2) continue;
    const double r = r_ego + 0.5 * std::hypot(ob.length, ob.width);
    bool near = false;
    for (size_t j = 1; j < ob.prediction.size() && !near; ++j) {
      const auto& a = ob.prediction[j - 1];
      const auto& b = ob.prediction[j];
      if (b.t < traj.t[0] - gap || a.t > t_hi + gap) continue;
      near = std::max(a.x, b.x) >= x_lo - r && std::min(a.x, b.x) <= x_hi + r &&
             std::max(a.y, b.y) >= y_lo - r && std::min(a.y, b.y) <= y_hi + r;
    }
    if (!near) continue;
    // Where the vehicle is shortly before and after the ego passes. Its past
    // before the first predicted state is unknown, so that part is skipped.
    for (size_t i = 0; i < n && traj.t[i] <= max_t; i += stride) {
      const Box2d ego{traj.x[i], traj.y[i], traj.yaw[i], len, wid};
      for (const double f : kGapFractions) {
        const double t = traj.t[i] + f * gap;
        if (t < ob.prediction.front().t) continue;
        const auto box = ob.boxAt(t);
        if (!box || std::abs(normalizeAngle(box->yaw - traj.yaw[i])) <= kPi / 4.0) continue;
        ++num_checks;
        if (overlaps(ego, *box)) {
          if (hit_id != nullptr) *hit_id = ob.id;
          if (hit_t != nullptr) *hit_t = traj.t[i];
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace fiss_planner
