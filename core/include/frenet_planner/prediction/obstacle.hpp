// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <optional>
#include <vector>

#include "frenet_planner/collision/box2d.hpp"

namespace frenet_planner {

/// One predicted pose of an obstacle. `t` is relative to the planning time.
struct ObstacleState {
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
};

/// An obstacle with a rectangular footprint and a (possibly single-state)
/// predicted trajectory. Predictions are an input to the planner, so any
/// predictor (constant velocity, learned, ground truth) can be plugged in.
struct Obstacle {
  int id = -1;
  double length = 0.0;
  double width = 0.0;
  bool is_static = false;
  std::vector<ObstacleState> prediction;  ///< sorted by t

  /// Footprint at time t, linearly interpolated between predicted states.
  /// Static obstacles and single-pose obstacles hold their pose; dynamic
  /// obstacles are unknown (nullopt) beyond their prediction horizon.
  std::optional<Box2d> boxAt(double t) const;
};

/// Predict an obstacle forward assuming constant speed and heading.
Obstacle predictConstantVelocity(int id, double x, double y, double yaw, double v, double length,
                                 double width, double horizon, double dt);

}  // namespace frenet_planner
