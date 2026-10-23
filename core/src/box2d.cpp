// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/collision/box2d.hpp"

namespace frenet_planner {

std::array<Vec2, 4> Box2d::corners() const {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double hl = 0.5 * length, hw = 0.5 * width;
  const double lx = hl * c, ly = hl * s;   // half-length along heading
  const double wx = -hw * s, wy = hw * c;  // half-width to the left
  return {{{cx + lx + wx, cy + ly + wy},
           {cx - lx + wx, cy - ly + wy},
           {cx - lx - wx, cy - ly - wy},
           {cx + lx - wx, cy + ly - wy}}};
}

bool overlaps(const Box2d& a, const Box2d& b) {
  // Cheap bounding-circle rejection first.
  const double dx = b.cx - a.cx, dy = b.cy - a.cy;
  const double r = a.circumradius() + b.circumradius();
  if (dx * dx + dy * dy > r * r) return false;

  // For two rectangles only the four face normals are candidate separating
  // axes. Project the center distance and both half-extents onto each axis.
  const double ca = std::cos(a.yaw), sa = std::sin(a.yaw);
  const double cb = std::cos(b.yaw), sb = std::sin(b.yaw);
  const double axes[4][2] = {{ca, sa}, {-sa, ca}, {cb, sb}, {-sb, cb}};
  for (const auto& ax : axes) {
    const double dist = std::abs(dx * ax[0] + dy * ax[1]);
    const double ra = 0.5 * a.length * std::abs(ca * ax[0] + sa * ax[1]) +
                      0.5 * a.width * std::abs(-sa * ax[0] + ca * ax[1]);
    const double rb = 0.5 * b.length * std::abs(cb * ax[0] + sb * ax[1]) +
                      0.5 * b.width * std::abs(-sb * ax[0] + cb * ax[1]);
    if (dist > ra + rb) return false;
  }
  return true;
}

}  // namespace frenet_planner
