// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/control/controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "frenet_planner/common/math.hpp"
#include "frenet_planner/planning/local_planner.hpp"

namespace frenet_planner {

// ----------------------------------------------------------------------------
// Kinematic bicycle
// ----------------------------------------------------------------------------

KinematicBicycle::KinematicBicycle(const VehicleParams& vehicle, const EgoState& initial,
                                   double initial_steer)
    : vehicle_(vehicle), state_(initial), steer_(initial_steer) {}

void KinematicBicycle::step(const ControlCommand& cmd, double dt) {
  const double max_d = vehicle_.max_steer_rate * dt;
  steer_ = std::clamp(steer_ + std::clamp(cmd.steer - steer_, -max_d, max_d), -vehicle_.max_steer,
                      vehicle_.max_steer);
  double a = std::clamp(cmd.accel, -vehicle_.max_decel, vehicle_.max_accel);
  if (state_.v <= 0.0 && a < 0.0) a = 0.0;

  // Integrate the rear-axle point: x' = v cos psi, y' = v sin psi,
  // psi' = v tan(delta) / L, v' = a.
  const double b = vehicle_.rear_axle_to_center;
  const double L = vehicle_.wheelbase;
  const double k = std::tan(steer_) / L;
  using S = std::array<double, 4>;
  const auto f = [&](const S& s) { return S{s[3] * std::cos(s[2]), s[3] * std::sin(s[2]), s[3] * k, a}; };
  S s{state_.x - b * std::cos(state_.yaw), state_.y - b * std::sin(state_.yaw), state_.yaw, state_.v};
  const auto add = [](const S& x, const S& y, double h) {
    return S{x[0] + h * y[0], x[1] + h * y[1], x[2] + h * y[2], x[3] + h * y[3]};
  };
  const S k1 = f(s), k2 = f(add(s, k1, dt / 2)), k3 = f(add(s, k2, dt / 2)), k4 = f(add(s, k3, dt));
  for (int i = 0; i < 4; ++i) s[i] += dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  if (s[3] < 0.0) {
    s[3] = 0.0;
    a = 0.0;
  }
  state_.yaw = normalizeAngle(s[2]);
  state_.x = s[0] + b * std::cos(state_.yaw);
  state_.y = s[1] + b * std::sin(state_.yaw);
  state_.v = s[3];
  state_.a = a;
  state_.kappa = k;
}

// ----------------------------------------------------------------------------
// Stanley + PID
// ----------------------------------------------------------------------------

namespace {

/// Closest point of the trajectory polyline to (x, y): position and heading.
struct PathPoint {
  double x, y, yaw;
};
PathPoint closestPoint(const Trajectory& traj, double x, double y) {
  PathPoint best{traj.x[0], traj.y[0], traj.yaw[0]};
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < traj.size(); ++i) {
    const double ax = traj.x[i], ay = traj.y[i];
    const double dx = traj.x[i + 1] - ax, dy = traj.y[i + 1] - ay;
    const double len2 = dx * dx + dy * dy;
    const double u = len2 > 1e-12 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0) : 0.0;
    const double px = ax + u * dx, py = ay + u * dy;
    const double d2 = (px - x) * (px - x) + (py - y) * (py - y);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = {px, py, traj.yaw[i] + u * normalizeAngle(traj.yaw[i + 1] - traj.yaw[i])};
    }
  }
  return best;
}

}  // namespace

StanleyPidController::StanleyPidController(const VehicleParams& vehicle, const StanleyPidConfig& config)
    : vehicle_(vehicle), cfg_(config) {}

void StanleyPidController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  has_prev_ = false;
}

ControlCommand StanleyPidController::compute(const EgoState& ego, const Trajectory& traj, double t) {
  ControlCommand cmd;
  if (traj.size() < 2 || !traj.is_cartesian) return {0.0, -vehicle_.max_decel};

  // Lateral: Stanley at the front axle.
  const double front = vehicle_.wheelbase - vehicle_.rear_axle_to_center;
  const double fx = ego.x + front * std::cos(ego.yaw), fy = ego.y + front * std::sin(ego.yaw);
  const PathPoint p = closestPoint(traj, fx, fy);
  const double e_left = -(fx - p.x) * std::sin(p.yaw) + (fy - p.y) * std::cos(p.yaw);
  const double heading = normalizeAngle(p.yaw - ego.yaw);
  cmd.steer =
      cfg_.k_heading * heading + std::atan2(-cfg_.k_cross * e_left, cfg_.k_soft + std::max(0.0, ego.v));
  cmd.steer = std::clamp(cmd.steer, -vehicle_.max_steer, vehicle_.max_steer);

  // Longitudinal: feedforward acceleration plus PID on the speed error.
  const EgoState ref = sampleTrajectory(traj, t);
  const double err = ref.v - ego.v;
  integral_ = std::clamp(integral_ + err * cfg_.dt, -5.0, 5.0);
  const double deriv = has_prev_ ? (err - prev_error_) / cfg_.dt : 0.0;
  prev_error_ = err;
  has_prev_ = true;
  cmd.accel = std::clamp(ref.a + cfg_.kp * err + cfg_.ki * integral_ + cfg_.kd * deriv, -vehicle_.max_decel,
                         vehicle_.max_accel);
  return cmd;
}

#ifndef FRENET_PLANNER_HAS_OSQP
MpcController::MpcController(const VehicleParams& vehicle, const MpcConfig& config)
    : vehicle_(vehicle), cfg_(config), backup_(vehicle) {}
ControlCommand MpcController::compute(const EgoState& ego, const Trajectory& traj, double t) {
  last_ok_ = false;
  return backup_.compute(ego, traj, t);
}
#endif

std::unique_ptr<Controller> makeController(const std::string& name, const VehicleParams& vehicle) {
  if (name == "mpc") return std::make_unique<MpcController>(vehicle);
  if (name == "stanley_pid" || name == "stanley") return std::make_unique<StanleyPidController>(vehicle);
  throw std::invalid_argument("unknown controller '" + name + "' (expected mpc or stanley_pid)");
}

}  // namespace frenet_planner
