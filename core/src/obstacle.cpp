// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/prediction/obstacle.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/common/math.hpp"

namespace frenet_planner {

std::optional<Box2d> Obstacle::boxAt(double t) const {
  if (prediction.empty()) return std::nullopt;
  const auto make = [this](double x, double y, double yaw) { return Box2d{x, y, yaw, length, width}; };

  constexpr double kEps = 1e-6;
  // A single observed pose carries no motion information; hold it (conservative).
  if (prediction.size() == 1 || t <= prediction.front().t) {
    const auto& p = prediction.front();
    return make(p.x, p.y, p.yaw);
  }
  if (t >= prediction.back().t) {
    const auto& p = prediction.back();
    if (!is_static && t > p.t + kEps) return std::nullopt;
    return make(p.x, p.y, p.yaw);
  }
  const auto it = std::upper_bound(prediction.begin(), prediction.end(), t,
                                   [](double v, const ObstacleState& s) { return v < s.t; });
  const auto& b = *it;
  const auto& a = *(it - 1);
  const double w = (t - a.t) / (b.t - a.t);
  return make(a.x + w * (b.x - a.x), a.y + w * (b.y - a.y), a.yaw + w * normalizeAngle(b.yaw - a.yaw));
}

Obstacle predictConstantVelocity(int id, double x, double y, double yaw, double v, double length,
                                 double width, double horizon, double dt) {
  Obstacle ob;
  ob.id = id;
  ob.length = length;
  ob.width = width;
  ob.is_static = std::abs(v) < 0.1;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (double t : timeSamples(horizon, dt)) {
    ob.prediction.push_back({t, x + v * t * c, y + v * t * s, yaw, v});
  }
  return ob;
}

}  // namespace frenet_planner
