// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <array>
#include <cmath>

namespace frenet_planner {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

/// Oriented rectangle described by its center, heading and full extents.
struct Box2d {
  double cx = 0.0;
  double cy = 0.0;
  double yaw = 0.0;
  double length = 0.0;  ///< extent along the heading
  double width = 0.0;   ///< extent across the heading

  /// Corners in counter-clockwise order starting at front-left.
  std::array<Vec2, 4> corners() const;
  double circumradius() const { return 0.5 * std::hypot(length, width); }
};

/// Separating-axis test for two oriented boxes (touching counts as overlap).
bool overlaps(const Box2d& a, const Box2d& b);

}  // namespace frenet_planner
