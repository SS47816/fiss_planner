// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <vector>

#include "fiss_planner/planning/planner.hpp"

namespace fiss_planner {

/// Space-time driving corridor around a trajectory: for every trajectory
/// time, the lateral interval free of road edges and obstacles (on the side
/// the trajectory passes them) and the longitudinal interval free of
/// obstacles in the occupied lateral band.
struct SpaceTimeCorridor {
  std::vector<double> t;
  std::vector<double> d_lo, d_hi;  ///< bounds of the ego center's lateral offset
  std::vector<double> s_lo, s_hi;  ///< bounds of the ego center's arc length
};

struct SmootherConfig {
  bool enabled = false;
  double margin_lat = 0.3;    ///< clearance kept laterally [m]
  double margin_lon = 1.0;    ///< clearance kept longitudinally [m]
  double w_lat_ref = 0.5;     ///< stay close to the sampled lateral motion
  double w_lat_target = 1.0;  ///< pull towards the lane center
  double w_lat_accel = 1.0;
  double w_lat_jerk = 2.0;
  double w_lon_ref = 0.05;       ///< stay close to the sampled positions
  double w_lon_speed_ref = 1.0;  ///< stay close to the sampled speed profile (keeps its timing)
  double w_lon_speed = 0.1;      ///< (s_d - target speed)^2
  double w_lon_accel = 1.0;
  double w_lon_jerk = 2.0;
};

/// Lateral bounds are computed with the longitudinal motion of `traj`, and
/// longitudinal bounds with its lateral motion.
SpaceTimeCorridor buildSpaceTimeCorridor(const Trajectory& traj, const ReferenceLine& ref,
                                         const Corridor& corridor, const std::vector<Obstacle>& obstacles,
                                         const VehicleParams& vehicle, double margin_lat, double margin_lon);

/// Refine a sampled trajectory with two piecewise-jerk QPs (lateral, then
/// longitudinal) inside its space-time corridor. The result replaces `traj`
/// only if it passes the same constraint and collision checks as sampled
/// candidates; returns whether it did.
bool smoothTrajectory(Trajectory& traj, const PlanningInput& input, const PlannerConfig& planner,
                      const VehicleParams& vehicle, const SmootherConfig& config);

}  // namespace fiss_planner
