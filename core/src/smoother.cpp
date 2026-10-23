// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/smoothing/smoother.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "frenet_planner/smoothing/piecewise_jerk.hpp"

namespace frenet_planner {

namespace {

struct FrenetBox {
  double s_min, s_max, d_min, d_max;
};

FrenetBox toFrenetBox(const Box2d& box, const ReferenceLine& ref, double s_hint) {
  FrenetBox fb{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
               std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  for (const auto& c : box.corners()) {
    const auto p = ref.project(c.x, c.y, s_hint);
    fb.s_min = std::min(fb.s_min, p.s);
    fb.s_max = std::max(fb.s_max, p.s);
    fb.d_min = std::min(fb.d_min, p.d);
    fb.d_max = std::max(fb.d_max, p.d);
  }
  return fb;
}

}  // namespace

SpaceTimeCorridor buildSpaceTimeCorridor(const Trajectory& traj, const ReferenceLine& ref,
                                         const Corridor& corridor, const std::vector<Obstacle>& obstacles,
                                         const VehicleParams& vehicle, double margin_lat, double margin_lon) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  constexpr double kRoadMargin = 0.05;
  const size_t n = traj.size();
  SpaceTimeCorridor c;
  c.t = traj.t;
  c.d_lo.assign(n, -kInf);
  c.d_hi.assign(n, kInf);
  c.s_lo.assign(n, -kInf);
  c.s_hi.assign(n, kInf);
  const double half_w = 0.5 * vehicle.width;
  const double half_l = 0.5 * vehicle.length;

  for (size_t i = 0; i < n; ++i) {
    const double s = traj.s[i], d = traj.d[i];
    if (!corridor.empty()) {
      const auto [lo, hi] = corridor.roadBounds(s);
      c.d_lo[i] = lo + half_w + kRoadMargin;
      c.d_hi[i] = hi - half_w - kRoadMargin;
    }
    for (const auto& ob : obstacles) {
      const auto box = ob.boxAt(traj.t[i]);
      if (!box) continue;
      const FrenetBox fb = toFrenetBox(*box, ref, s);
      const bool lon_overlap = fb.s_max >= s - half_l - margin_lon && fb.s_min <= s + half_l + margin_lon;
      const bool lat_overlap = fb.d_max >= d - half_w - margin_lat && fb.d_min <= d + half_w + margin_lat;
      if (lon_overlap && !lat_overlap) {
        // Passing beside the obstacle: keep to the side the trajectory takes.
        if (d > fb.d_max) {
          c.d_lo[i] = std::max(c.d_lo[i], fb.d_max + half_w + margin_lat);
        } else {
          c.d_hi[i] = std::min(c.d_hi[i], fb.d_min - half_w - margin_lat);
        }
      } else if (lat_overlap) {
        // Sharing the lateral band: stay behind (or ahead of) it.
        if (fb.s_min >= s) {
          c.s_hi[i] = std::min(c.s_hi[i], fb.s_min - half_l - margin_lon);
        } else if (fb.s_max <= s) {
          c.s_lo[i] = std::max(c.s_lo[i], fb.s_max + half_l + margin_lon);
        }
      }
    }
    // The sampled trajectory is feasible; never cut it out of its own corridor.
    c.d_lo[i] = std::min(c.d_lo[i], d);
    c.d_hi[i] = std::max(c.d_hi[i], d);
    c.s_lo[i] = std::min(c.s_lo[i], s);
    c.s_hi[i] = std::max(c.s_hi[i], s);
  }
  return c;
}

bool smoothTrajectory(Trajectory& traj, const PlanningInput& input, const PlannerConfig& planner,
                      const VehicleParams& vehicle, const SmootherConfig& cfg) {
  if (!hasOptimizers() || traj.size() < 3 || input.reference == nullptr || input.corridor == nullptr) {
    return false;
  }
  static const std::vector<Obstacle> kNone;
  const auto& obstacles = input.obstacles ? *input.obstacles : kNone;
  const ReferenceLine& ref = *input.reference;
  const size_t n = traj.size();
  const double dt = traj.t[1] - traj.t[0];

  // Lane center of the trajectory's target lane along the trajectory.
  const LaneBand* lane = nullptr;
  for (const auto& l : input.corridor->lanes()) {
    if (l.offset == traj.lane) lane = &l;
  }

  // --- lateral QP -----------------------------------------------------------
  auto corridor =
      buildSpaceTimeCorridor(traj, ref, *input.corridor, obstacles, vehicle, cfg.margin_lat, cfg.margin_lon);
  PiecewiseJerkProblem lat;
  lat.dt = dt;
  lat.x0 = {traj.d[0], traj.d_d[0], traj.d_dd[0]};
  lat.x_ref.resize(n);
  const double w_sum = cfg.w_lat_ref + cfg.w_lat_target;
  for (size_t i = 0; i < n; ++i) {
    const double center = lane ? input.corridor->center(*lane, traj.s[i]) : traj.end.d;
    lat.x_ref[i] = w_sum > 0.0 ? (cfg.w_lat_ref * traj.d[i] + cfg.w_lat_target * center) / w_sum : traj.d[i];
  }
  lat.x_lo = corridor.d_lo;
  lat.x_hi = corridor.d_hi;
  lat.ddx_lo = -vehicle.max_lat_accel;
  lat.ddx_hi = vehicle.max_lat_accel;
  lat.w_x = w_sum;
  lat.w_ddx = cfg.w_lat_accel;
  lat.w_dddx = cfg.w_lat_jerk;
  lat.w_end_dx = 10.0;
  lat.w_end_ddx = 10.0;
  const auto lat_sol = solvePiecewiseJerk(lat);
  if (!lat_sol) return false;

  // --- longitudinal QP (bounds re-derived with the new lateral motion) -------
  Trajectory shifted = traj;
  shifted.d = lat_sol->x;
  corridor = buildSpaceTimeCorridor(shifted, ref, *input.corridor, obstacles, vehicle, cfg.margin_lat,
                                    cfg.margin_lon);
  PiecewiseJerkProblem lon;
  lon.dt = dt;
  lon.x0 = {traj.s[0], traj.s_d[0], traj.s_dd[0]};
  lon.x_ref = traj.s;
  // One quadratic speed term: a weighted mix of the sampled profile (timing)
  // and the target speed.
  const double wv = cfg.w_lon_speed_ref + cfg.w_lon_speed;
  lon.dx_ref.resize(n);
  for (size_t i = 0; i < n; ++i) {
    lon.dx_ref[i] = wv > 0.0 ? (cfg.w_lon_speed_ref * traj.s_d[i] + cfg.w_lon_speed * input.target_speed) / wv
                             : traj.s_d[i];
  }
  lon.x_lo = corridor.s_lo;
  lon.x_hi = corridor.s_hi;
  lon.dx_lo = 0.0;
  lon.dx_hi = vehicle.max_speed;
  lon.ddx_lo = -vehicle.max_decel;
  lon.ddx_hi = vehicle.max_accel;
  lon.w_x = cfg.w_lon_ref;
  lon.w_dx = wv;
  lon.w_ddx = cfg.w_lon_accel;
  lon.w_dddx = cfg.w_lon_jerk;
  lon.w_end_ddx = 10.0;
  const auto lon_sol = solvePiecewiseJerk(lon);
  if (!lon_sol) return false;

  Trajectory out;
  out.t = traj.t;
  out.s = lon_sol->x;
  out.s_d = lon_sol->dx;
  out.s_dd = lon_sol->ddx;
  out.s_ddd = lon_sol->dddx;
  out.d = lat_sol->x;
  out.d_d = lat_sol->dx;
  out.d_dd = lat_sol->ddx;
  out.d_ddd = lat_sol->dddx;
  out.end = {out.d.back(), out.s_d.back(), traj.end.T};
  out.idx = traj.idx;
  out.lane = traj.lane;
  if (!convertToCartesian(ref, out)) return false;

  // Same acceptance test as every sampled candidate.
  if (ConstraintValidator(planner.constraints, vehicle).check(out, input.corridor) != Rejection::kOk) {
    return false;
  }
  int checks = 0;
  if (!CollisionChecker(planner.collision, vehicle).isCollisionFree(out, obstacles, checks)) return false;

  LateralSample lat_info{out.end.d, out.lane, lane ? input.corridor->center(*lane, out.s.back()) : out.end.d,
                         lane ? lane->extra_cost : 0.0};
  out.cost = CostEvaluator(planner.cost, planner.sampling).evaluate(out, lat_info, input.target_speed);
  traj = std::move(out);
  return true;
}

}  // namespace frenet_planner
