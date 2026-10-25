// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <cmath>
#include <cstddef>

namespace fiss_planner {

/// Cartesian state of the ego vehicle. (x, y) is the geometric center of the
/// vehicle's bounding box; adapters convert from rear-axle frames if needed.
struct EgoState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;    ///< heading [rad]
  double v = 0.0;      ///< speed [m/s]
  double a = 0.0;      ///< longitudinal acceleration [m/s^2]
  double kappa = 0.0;  ///< path curvature [1/m]
};

/// Frenet state with time derivatives of both s and d.
struct FrenetState {
  double s = 0.0;
  double s_d = 0.0;
  double s_dd = 0.0;
  double d = 0.0;
  double d_d = 0.0;
  double d_dd = 0.0;
};

/// Target end state of a sampled trajectory (longitudinal position is free).
struct EndState {
  double d = 0.0;    ///< lateral offset [m]
  double s_d = 0.0;  ///< longitudinal speed [m/s]
  double T = 0.0;    ///< duration [s]
};

/// Geometric and kinematic limits of the ego vehicle.
struct VehicleParams {
  double length = 4.5;                ///< [m]
  double width = 1.8;                 ///< [m]
  double wheelbase = 2.7;             ///< [m]
  double rear_axle_to_center = 1.35;  ///< rear axle to box center [m]
  double max_speed = 30.0;            ///< [m/s]
  double max_accel = 3.0;             ///< [m/s^2]
  double max_decel = 6.0;             ///< magnitude of max deceleration [m/s^2]
  double max_steer = 0.6;             ///< [rad]
  double max_steer_rate = 0.5;        ///< [rad/s]
  double max_lat_accel = 4.0;         ///< [m/s^2]

  /// Curvature limit implied by the steering limit (kinematic bicycle).
  double maxCurvature() const { return std::tan(max_steer) / wheelbase; }
};

}  // namespace fiss_planner
