// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "fiss_planner/common/types.hpp"

namespace fiss_planner {

/// Breakdown of a trajectory's cost.
struct CostTerms {
  double time = 0.0;
  double speed = 0.0;
  double accel = 0.0;
  double jerk = 0.0;
  double lateral = 0.0;
  double lane_change = 0.0;
  double proximity = 0.0;  ///< headway, time-to-collision and lateral clearance to obstacles
  double road_edge = 0.0;  ///< footprint beyond the road edges
  double total() const { return time + speed + accel + jerk + lateral + lane_change + proximity + road_edge; }
};

/// Why a candidate was rejected (or kOk).
enum class Rejection : int {
  kOk = 0,
  kInvalid,         ///< non-finite values
  kSpeed,           ///< exceeds max speed or drives backwards
  kAccel,           ///< exceeds longitudinal acceleration/deceleration limits
  kCurvature,       ///< exceeds curvature limit
  kLatAccel,        ///< exceeds lateral acceleration limit
  kOffRoad,         ///< leaves the drivable corridor
  kCollision,       ///< collides with a predicted obstacle
  kSafetyDistance,  ///< closer to an obstacle ahead than the safety envelope allows
  kCount
};

/// A candidate or final trajectory. Frenet arrays are always filled; the
/// Cartesian arrays are filled once the trajectory has been converted.
struct Trajectory {
  EndState end;
  std::array<int, 3> idx{-1, -1, -1};  ///< grid index (width, speed, time); -1 if off-grid
  int lane = 0;                        ///< lane offset relative to ego lane (0 = keep)

  std::vector<double> t;
  std::vector<double> s, s_d, s_dd, s_ddd;
  std::vector<double> d, d_d, d_dd, d_ddd;

  std::vector<double> x, y, yaw, v, a, kappa;

  CostTerms cost;
  double cost_estimate = 0.0;  ///< cheap end-state estimate (incl. heuristic)
  bool is_cartesian = false;
  Rejection rejection = Rejection::kOk;
  int blocking_id = -1;     ///< obstacle behind a kCollision / kSafetyDistance rejection
  double blocking_t = 0.0;  ///< time of that first conflict [s]

  size_t size() const { return t.size(); }
  double totalCost() const { return cost.total(); }
  EgoState cartesianStateAt(size_t i) const { return {x[i], y[i], yaw[i], v[i], a[i], kappa[i]}; }
  FrenetState frenetStateAt(size_t i) const { return {s[i], s_d[i], s_dd[i], d[i], d_d[i], d_dd[i]}; }
};

}  // namespace fiss_planner
