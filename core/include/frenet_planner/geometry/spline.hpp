// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <cstddef>
#include <vector>

namespace frenet_planner {

/// Cubic spline y(x) through an arbitrary number (>= 2) of knots with
/// strictly increasing x. Outside the knot range the spline is continued
/// linearly with the end slope, so evaluation never returns garbage.
class CubicSpline1D {
 public:
  enum class Boundary {
    kNatural,    ///< zero second derivative at both ends
    kParabolic,  ///< end segments are parabolas (second derivative continued)
  };

  CubicSpline1D() = default;
  CubicSpline1D(std::vector<double> x, std::vector<double> y, Boundary boundary = Boundary::kNatural);

  double operator()(double t) const;
  double d1(double t) const;
  double d2(double t) const;
  double d3(double t) const;

  bool empty() const { return x_.size() < 2; }
  double front() const { return x_.front(); }
  double back() const { return x_.back(); }
  const std::vector<double>& knots() const { return x_; }

 private:
  /// Index of the segment containing t (clamped to valid segments).
  size_t segment(double t) const;

  std::vector<double> x_, a_, b_, c_, d_;
  double end_slope_ = 0.0;
};

/// Planar parametric spline (x(s), y(s)) parameterized by cumulative chord
/// length, with parabolic end conditions so that curvature does not artificially
/// drop to zero at the ends. Consecutive duplicate points are dropped.
class CubicSpline2D {
 public:
  CubicSpline2D() = default;
  CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y);

  bool empty() const { return s_.size() < 2; }
  double length() const { return s_.empty() ? 0.0 : s_.back(); }
  const std::vector<double>& s() const { return s_; }

  void position(double s, double& x, double& y) const;
  double yaw(double s) const;
  double curvature(double s) const;
  double curvatureRate(double s) const;

 private:
  std::vector<double> s_;
  CubicSpline1D sx_, sy_;
};

}  // namespace frenet_planner
