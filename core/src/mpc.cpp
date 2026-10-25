// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// Linear time-varying MPC on kinematic bicycle error dynamics.
#ifdef FISS_PLANNER_HAS_OSQP

#include <algorithm>
#include <cmath>
#include <vector>

#include "fiss_planner/common/math.hpp"
#include "fiss_planner/control/controller.hpp"
#include "fiss_planner/planning/local_planner.hpp"
#include "qp.hpp"

namespace fiss_planner {

MpcController::MpcController(const VehicleParams& vehicle, const MpcConfig& config)
    : vehicle_(vehicle), cfg_(config), backup_(vehicle) {}

// Error state z = [e_lat, e_heading, e_lon, e_speed] of the rear axle relative
// to the reference rear-axle pose, in the reference frame rotating at
// w = v_ref * kappa_ref. Inputs u = [delta - delta_ref, a - a_ref].
//   e_lat'     =  v_ref e_heading - w e_lon
//   e_heading' =  kappa_ref e_speed + v_ref / (L cos^2 delta_ref) u_delta
//   e_lon'     =  w e_lat + e_speed
//   e_speed'   =  u_a
ControlCommand MpcController::compute(const EgoState& ego, const Trajectory& traj, double t) {
  last_ok_ = false;
  if (traj.size() < 2 || !traj.is_cartesian) return backup_.compute(ego, traj, t);
  const int N = std::max(2, cfg_.horizon);
  const double dt = cfg_.dt;
  const double L = vehicle_.wheelbase, b = vehicle_.rear_axle_to_center;

  struct Ref {
    double x, y, yaw, v, a, kappa, delta;
  };
  std::vector<Ref> ref(static_cast<size_t>(N + 1));
  for (int k = 0; k <= N; ++k) {
    const EgoState r = sampleTrajectory(traj, t + k * dt);
    const double delta = std::clamp(std::atan(L * r.kappa), -vehicle_.max_steer, vehicle_.max_steer);
    ref[k] = {r.x - b * std::cos(r.yaw), r.y - b * std::sin(r.yaw), r.yaw, r.v, r.a, r.kappa, delta};
  }

  const double rx = ego.x - b * std::cos(ego.yaw), ry = ego.y - b * std::sin(ego.yaw);
  const double c0 = std::cos(ref[0].yaw), s0 = std::sin(ref[0].yaw);
  const double z0[4] = {-(rx - ref[0].x) * s0 + (ry - ref[0].y) * c0, normalizeAngle(ego.yaw - ref[0].yaw),
                        (rx - ref[0].x) * c0 + (ry - ref[0].y) * s0, ego.v - ref[0].v};

  const int nz = 4 * (N + 1), nu = 2 * N;
  const auto Z = [](int k, int i) { return 4 * k + i; };
  const auto U = [nz](int k, int j) { return nz + 2 * k + j; };
  detail::SparseQp qp(nz + nu);
  const auto sq = [&](int i, double w, double target) {  // w (z_i - target)^2
    qp.addQuadratic(i, i, 2.0 * w);
    qp.q(i) -= 2.0 * w * target;
  };
  const auto diff = [&](int i, int j, double c, double w) {  // w (z_i - z_j + c)^2
    qp.addQuadratic(i, i, 2.0 * w);
    qp.addQuadratic(j, j, 2.0 * w);
    qp.addQuadratic(i, j, -2.0 * w);
    qp.q(i) += 2.0 * w * c;
    qp.q(j) -= 2.0 * w * c;
  };

  const double q[4] = {cfg_.q_lat, cfg_.q_heading, cfg_.q_lon, cfg_.q_speed};
  for (int k = 1; k <= N; ++k)
    for (int i = 0; i < 4; ++i) sq(Z(k, i), q[i], 0.0);
  for (int k = 0; k < N; ++k) {
    sq(U(k, 0), cfg_.r_steer, 0.0);
    sq(U(k, 1), cfg_.r_accel, 0.0);
  }
  // Smoothness of the applied commands (not of the deviations).
  sq(U(0, 0), cfg_.rd_steer, prev_steer_ - ref[0].delta);
  for (int k = 1; k < N; ++k) {
    diff(U(k, 0), U(k - 1, 0), ref[k].delta - ref[k - 1].delta, cfg_.rd_steer);
    diff(U(k, 1), U(k - 1, 1), ref[k].a - ref[k - 1].a, cfg_.rd_accel);
  }

  for (int i = 0; i < 4; ++i) qp.addRow({{Z(0, i), 1.0}}, z0[i], z0[i]);
  for (int k = 0; k < N; ++k) {
    const Ref& r = ref[k];
    const double w = r.v * r.kappa;
    const double cd = std::cos(r.delta);
    const double b_delta = r.v / (L * cd * cd) * dt;
    // z_{k+1} - (I + A dt) z_k - B dt u_k = 0
    qp.addRow({{Z(k + 1, 0), 1.0}, {Z(k, 0), -1.0}, {Z(k, 1), -r.v * dt}, {Z(k, 2), w * dt}}, 0.0, 0.0);
    qp.addRow({{Z(k + 1, 1), 1.0}, {Z(k, 1), -1.0}, {Z(k, 3), -r.kappa * dt}, {U(k, 0), -b_delta}}, 0.0, 0.0);
    qp.addRow({{Z(k + 1, 2), 1.0}, {Z(k, 2), -1.0}, {Z(k, 0), -w * dt}, {Z(k, 3), -dt}}, 0.0, 0.0);
    qp.addRow({{Z(k + 1, 3), 1.0}, {Z(k, 3), -1.0}, {U(k, 1), -dt}}, 0.0, 0.0);
    // Actuator limits on the applied commands.
    qp.addRow({{U(k, 0), 1.0}}, -vehicle_.max_steer - r.delta, vehicle_.max_steer - r.delta);
    qp.addRow({{U(k, 1), 1.0}}, -vehicle_.max_decel - r.a, vehicle_.max_accel - r.a);
    const double rate = vehicle_.max_steer_rate * dt;
    if (k == 0) {
      qp.addRow({{U(0, 0), 1.0}}, prev_steer_ - rate - r.delta, prev_steer_ + rate - r.delta);
    } else {
      const double c = ref[k].delta - ref[k - 1].delta;
      qp.addRow({{U(k, 0), 1.0}, {U(k - 1, 0), -1.0}}, -rate - c, rate - c);
    }
  }

  const auto sol = qp.solve(2000, 1e-4);
  if (!sol) {
    ControlCommand cmd = backup_.compute(ego, traj, t);
    prev_steer_ = cmd.steer;
    return cmd;
  }
  last_ok_ = true;
  ControlCommand cmd;
  cmd.steer = std::clamp((*sol)(U(0, 0)) + ref[0].delta, -vehicle_.max_steer, vehicle_.max_steer);
  cmd.accel = std::clamp((*sol)(U(0, 1)) + ref[0].a, -vehicle_.max_decel, vehicle_.max_accel);
  prev_steer_ = cmd.steer;
  return cmd;
}

}  // namespace fiss_planner

#endif
