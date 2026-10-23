// SPDX-License-Identifier: Apache-2.0
// Shared fixtures for the core unit tests.
#pragma once

#include <cmath>
#include <vector>

#include "frenet_planner/common/math.hpp"
#include "frenet_planner/geometry/reference_line.hpp"

namespace frenet_planner::test {

inline ReferenceLine straightLine(double length, double heading = 0.0) {
  std::vector<double> x, y;
  for (double s = 0.0; s <= length + 1e-9; s += 5.0) {
    x.push_back(s * std::cos(heading));
    y.push_back(s * std::sin(heading));
  }
  return ReferenceLine(x, y);
}

/// Counter-clockwise arc of radius R starting at the origin heading +x.
inline ReferenceLine arc(double R, double angle) {
  std::vector<double> x, y;
  const int n = std::max(8, static_cast<int>(R * angle / 2.0));
  for (int i = 0; i <= n; ++i) {
    const double a = angle * i / n;
    x.push_back(R * std::sin(a));
    y.push_back(R * (1.0 - std::cos(a)));
  }
  return ReferenceLine(x, y);
}

/// Straight leg, half circle of radius r, straight leg back.
inline ReferenceLine uTurn(double r, double leg) {
  std::vector<double> x, y;
  for (double s = 0.0; s < leg; s += 2.0) {
    x.push_back(s);
    y.push_back(0.0);
  }
  for (int i = 0; i <= 30; ++i) {
    const double a = kPi * i / 30.0;
    x.push_back(leg + r * std::sin(a));
    y.push_back(r * (1.0 - std::cos(a)));
  }
  for (double s = leg - 2.0; s >= 0.0; s -= 2.0) {
    x.push_back(s);
    y.push_back(2.0 * r);
  }
  return ReferenceLine(x, y);
}

/// Gentle S-curve about 200 m long.
inline ReferenceLine sCurve() {
  std::vector<double> x, y;
  for (double s = 0.0; s <= 200.0; s += 4.0) {
    x.push_back(s);
    y.push_back(8.0 * std::sin(s / 40.0));
  }
  return ReferenceLine(x, y);
}

}  // namespace frenet_planner::test
