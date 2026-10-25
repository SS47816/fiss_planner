// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/geometry/spline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fiss_planner {

CubicSpline1D::CubicSpline1D(std::vector<double> x, std::vector<double> y, Boundary boundary)
    : x_(std::move(x)), a_(std::move(y)) {
  const size_t n = x_.size();
  if (n < 2 || a_.size() != n) {
    throw std::invalid_argument("CubicSpline1D: need >= 2 knots and matching x/y sizes");
  }
  std::vector<double> h(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    h[i] = x_[i + 1] - x_[i];
    if (!(h[i] > 0.0)) throw std::invalid_argument("CubicSpline1D: x must be strictly increasing");
  }

  // Solve the interior tridiagonal system for c (half the second derivative)
  // with the Thomas algorithm. Natural ends: c_0 = c_{n-1} = 0. Parabolic
  // ends: c_0 = c_1 and c_{n-1} = c_{n-2}, folded into the first/last rows.
  c_.assign(n, 0.0);
  if (n > 2) {
    const size_t m = n - 2;
    std::vector<double> diag(m), upper(m), rhs(m);
    for (size_t k = 0; k < m; ++k) {
      const size_t i = k + 1;
      diag[k] = 2.0 * (h[i - 1] + h[i]);
      upper[k] = h[i];
      rhs[k] = 3.0 * ((a_[i + 1] - a_[i]) / h[i] - (a_[i] - a_[i - 1]) / h[i - 1]);
    }
    if (boundary == Boundary::kParabolic) {
      diag.front() += h.front();
      diag.back() += h.back();
    }
    for (size_t k = 1; k < m; ++k) {
      const double w = h[k] / diag[k - 1];  // sub-diagonal entry of row k is h[k]
      diag[k] -= w * upper[k - 1];
      rhs[k] -= w * rhs[k - 1];
    }
    c_[m] = rhs[m - 1] / diag[m - 1];
    for (size_t k = m - 1; k-- > 0;) c_[k + 1] = (rhs[k] - upper[k] * c_[k + 2]) / diag[k];
    if (boundary == Boundary::kParabolic) {
      c_.front() = c_[1];
      c_.back() = c_[n - 2];
    }
  }

  b_.resize(n - 1);
  d_.resize(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    b_[i] = (a_[i + 1] - a_[i]) / h[i] - h[i] * (2.0 * c_[i] + c_[i + 1]) / 3.0;
    d_[i] = (c_[i + 1] - c_[i]) / (3.0 * h[i]);
  }
  const double hl = h.back();
  end_slope_ = b_.back() + 2.0 * c_[n - 2] * hl + 3.0 * d_.back() * hl * hl;
}

size_t CubicSpline1D::segment(double t) const {
  const auto it = std::upper_bound(x_.begin(), x_.end(), t);
  const size_t idx = static_cast<size_t>(std::max<std::ptrdiff_t>(0, it - x_.begin() - 1));
  return std::min(idx, x_.size() - 2);
}

double CubicSpline1D::operator()(double t) const {
  if (t < x_.front()) return a_.front() + b_.front() * (t - x_.front());
  if (t > x_.back()) return a_.back() + end_slope_ * (t - x_.back());
  const size_t i = segment(t);
  const double dx = t - x_[i];
  return a_[i] + dx * (b_[i] + dx * (c_[i] + dx * d_[i]));
}

double CubicSpline1D::d1(double t) const {
  if (t < x_.front()) return b_.front();
  if (t > x_.back()) return end_slope_;
  const size_t i = segment(t);
  const double dx = t - x_[i];
  return b_[i] + dx * (2.0 * c_[i] + 3.0 * d_[i] * dx);
}

double CubicSpline1D::d2(double t) const {
  if (t < x_.front() || t > x_.back()) return 0.0;
  const size_t i = segment(t);
  return 2.0 * c_[i] + 6.0 * d_[i] * (t - x_[i]);
}

double CubicSpline1D::d3(double t) const {
  if (t < x_.front() || t > x_.back()) return 0.0;
  return 6.0 * d_[segment(t)];
}

CubicSpline2D::CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y) {
  if (x.size() != y.size()) throw std::invalid_argument("CubicSpline2D: x/y size mismatch");
  std::vector<double> xs, ys;
  xs.reserve(x.size());
  ys.reserve(y.size());
  s_.reserve(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    if (!std::isfinite(x[i]) || !std::isfinite(y[i])) continue;
    if (xs.empty()) {
      xs.push_back(x[i]);
      ys.push_back(y[i]);
      s_.push_back(0.0);
      continue;
    }
    const double ds = std::hypot(x[i] - xs.back(), y[i] - ys.back());
    if (ds < 1e-6) continue;
    s_.push_back(s_.back() + ds);
    xs.push_back(x[i]);
    ys.push_back(y[i]);
  }
  if (s_.size() < 2) throw std::invalid_argument("CubicSpline2D: need >= 2 distinct points");
  sx_ = CubicSpline1D(s_, xs, CubicSpline1D::Boundary::kParabolic);
  sy_ = CubicSpline1D(s_, ys, CubicSpline1D::Boundary::kParabolic);
}

void CubicSpline2D::position(double s, double& x, double& y) const {
  x = sx_(s);
  y = sy_(s);
}

double CubicSpline2D::yaw(double s) const { return std::atan2(sy_.d1(s), sx_.d1(s)); }

double CubicSpline2D::curvature(double s) const {
  const double dx = sx_.d1(s), dy = sy_.d1(s);
  const double ddx = sx_.d2(s), ddy = sy_.d2(s);
  const double n2 = dx * dx + dy * dy;
  return (dx * ddy - dy * ddx) / (n2 * std::sqrt(n2));
}

double CubicSpline2D::curvatureRate(double s) const {
  const double dx = sx_.d1(s), dy = sy_.d1(s);
  const double ddx = sx_.d2(s), ddy = sy_.d2(s);
  const double dddx = sx_.d3(s), dddy = sy_.d3(s);
  const double n2 = dx * dx + dy * dy;
  const double cross = dx * ddy - dy * ddx;
  const double num = (dx * dddy - dy * dddx) * n2 - 3.0 * cross * (dx * ddx + dy * ddy);
  return num / (n2 * n2 * std::sqrt(n2));
}

}  // namespace fiss_planner
