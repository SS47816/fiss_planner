// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <limits>
#include <vector>

#include "frenet_planner/common/types.hpp"
#include "frenet_planner/geometry/spline.hpp"

namespace frenet_planner {

struct RefPoint {
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;
};

/// Result of projecting a Cartesian point onto the reference line.
struct Projection {
  double s = 0.0;  ///< arc length of the foot point
  double d = 0.0;  ///< signed lateral offset, positive to the left
};

/// Arc-length parameterized reference line (Frenet frame) built from a polyline.
///
/// The input polyline is fitted with a chord-length spline, densely resampled
/// and refitted so that the parameter is close to true arc length. Beyond its
/// ends the line continues straight, so samples running past the end of the
/// route stay well defined.
class ReferenceLine {
 public:
  ReferenceLine() = default;
  ReferenceLine(const std::vector<double>& x, const std::vector<double>& y, double resample_ds = 1.0);

  bool empty() const { return spline_.empty(); }
  double length() const { return spline_.length(); }

  RefPoint at(double s) const;
  std::vector<RefPoint> sample(double ds) const;

  /// Project (x, y) onto the line. With a finite `s_hint` only a window around
  /// the hint is searched, which keeps projection unambiguous on looping roads.
  Projection project(double x, double y, double s_hint = std::numeric_limits<double>::quiet_NaN()) const;

  /// Exact conversion of a Cartesian kinematic state into time-domain Frenet
  /// coordinates (s, s_d, s_dd, d, d_d, d_dd).
  FrenetState toFrenet(const EgoState& state, double s_hint = std::numeric_limits<double>::quiet_NaN()) const;

  /// Inverse of toFrenet. Heading and curvature come from the analytic
  /// velocity/acceleration vectors, not from finite differences.
  EgoState toCartesian(const FrenetState& fs) const;

 private:
  CubicSpline2D spline_;
  std::vector<double> table_s_, table_x_, table_y_;  ///< dense lookup table
  double table_ds_ = 0.5;
};

}  // namespace frenet_planner
