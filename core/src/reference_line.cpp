// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/geometry/reference_line.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "fiss_planner/common/math.hpp"

namespace fiss_planner {

namespace {
constexpr double kMinSpeed = 1e-3;
constexpr double kHintWindow = 30.0;  // [m] search window around a projection hint
}  // namespace

ReferenceLine::ReferenceLine(const std::vector<double>& x, const std::vector<double>& y, double resample_ds) {
  if (!(resample_ds > 0.0)) throw std::invalid_argument("ReferenceLine: resample_ds must be > 0");
  const CubicSpline2D raw(x, y);

  // Resample densely, measure true arc length, then refit on ~resample_ds knots.
  const double fine = std::min(0.1, resample_ds / 4.0);
  const int n_fine = std::max(2, static_cast<int>(std::ceil(raw.length() / fine)) + 1);
  std::vector<double> fx(n_fine), fy(n_fine), fs(n_fine, 0.0);
  for (int i = 0; i < n_fine; ++i) {
    raw.position(raw.length() * i / (n_fine - 1), fx[i], fy[i]);
    if (i > 0) fs[i] = fs[i - 1] + std::hypot(fx[i] - fx[i - 1], fy[i] - fy[i - 1]);
  }
  const int stride = std::max(1, static_cast<int>(std::round(resample_ds / fine)));
  std::vector<double> kx, ky;
  for (int i = 0; i < n_fine; i += stride) {
    kx.push_back(fx[i]);
    ky.push_back(fy[i]);
  }
  if (fs.back() - fs[(n_fine - 1) / stride * stride] > 1e-6) {
    kx.push_back(fx.back());
    ky.push_back(fy.back());
  }
  spline_ = CubicSpline2D(kx, ky);

  const int n_table = std::max(2, static_cast<int>(std::ceil(length() / table_ds_)) + 1);
  table_s_.resize(n_table);
  table_x_.resize(n_table);
  table_y_.resize(n_table);
  for (int i = 0; i < n_table; ++i) {
    table_s_[i] = length() * i / (n_table - 1);
    spline_.position(table_s_[i], table_x_[i], table_y_[i]);
  }
}

RefPoint ReferenceLine::at(double s) const {
  RefPoint p;
  p.s = s;
  spline_.position(s, p.x, p.y);
  p.yaw = spline_.yaw(s);
  p.kappa = spline_.curvature(s);
  p.dkappa = spline_.curvatureRate(s);
  return p;
}

std::vector<RefPoint> ReferenceLine::sample(double ds) const {
  std::vector<RefPoint> out;
  if (empty() || !(ds > 0.0)) return out;
  for (double s : linspace(0.0, length(), static_cast<int>(std::ceil(length() / ds)) + 1)) {
    out.push_back(at(s));
  }
  return out;
}

Projection ReferenceLine::project(double x, double y, double s_hint) const {
  if (empty()) throw std::runtime_error("ReferenceLine::project on empty line");

  size_t lo = 0, hi = table_s_.size();
  if (std::isfinite(s_hint)) {
    lo = static_cast<size_t>(std::lower_bound(table_s_.begin(), table_s_.end(), s_hint - kHintWindow) -
                             table_s_.begin());
    hi = static_cast<size_t>(std::upper_bound(table_s_.begin(), table_s_.end(), s_hint + kHintWindow) -
                             table_s_.begin());
    if (lo >= hi) {  // hint outside the line: fall back to the nearest end
      lo = s_hint < 0.0 ? 0 : table_s_.size() - 1;
      hi = lo + 1;
    }
  }
  size_t best = lo;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = lo; i < hi; ++i) {
    const double d2 = (table_x_[i] - x) * (table_x_[i] - x) + (table_y_[i] - y) * (table_y_[i] - y);
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }

  // Initial guess: project onto the tangent at the nearest table point. At the
  // ends the line continues straight, so this is exact behind/beyond it.
  double s = table_s_[best];
  {
    const double yaw = spline_.yaw(s);
    double correction = (x - table_x_[best]) * std::cos(yaw) + (y - table_y_[best]) * std::sin(yaw);
    const bool at_end = best == 0 || best + 1 == table_s_.size();
    if (!at_end) correction = std::clamp(correction, -table_ds_, table_ds_);
    s += correction;
  }
  // Newton iterations on g(s) = (q - P(s)) . T(s) = 0.
  for (int it = 0; it < 10; ++it) {
    const RefPoint r = at(s);
    const double c = std::cos(r.yaw), sn = std::sin(r.yaw);
    const double ex = x - r.x, ey = y - r.y;
    const double g = ex * c + ey * sn;
    const double dg = -1.0 + r.kappa * (-ex * sn + ey * c);
    if (std::abs(dg) < 1e-9) break;
    const double step = std::clamp(g / -dg, -table_ds_ * 2.0, table_ds_ * 2.0);
    s += step;
    if (std::abs(step) < 1e-9) break;
  }
  const RefPoint r = at(s);
  Projection p;
  p.s = s;
  p.d = -(x - r.x) * std::sin(r.yaw) + (y - r.y) * std::cos(r.yaw);
  return p;
}

// Kinematics in the moving (T, N) frame of the reference point:
//   vel = [s_d (1 - kr d), d_d]
//   acc = [s_dd (1 - kr d) - s_d^2 kr' d - 2 kr s_d d_d,  kr s_d^2 (1 - kr d) + d_dd]
FrenetState ReferenceLine::toFrenet(const EgoState& st, double s_hint) const {
  const Projection p = project(st.x, st.y, s_hint);
  const RefPoint r = at(p.s);
  const double one_minus = 1.0 - r.kappa * p.d;
  const double dtheta = normalizeAngle(st.yaw - r.yaw);
  const double ct = std::cos(dtheta), sn = std::sin(dtheta);

  FrenetState fs;
  fs.s = p.s;
  fs.d = p.d;
  fs.s_d = st.v * ct / one_minus;
  fs.d_d = st.v * sn;
  // Acceleration of the ego in the (T, N) basis: tangential a, normal v^2 kappa.
  const double acc_t = st.a * ct - st.v * st.v * st.kappa * sn;
  const double acc_n = st.a * sn + st.v * st.v * st.kappa * ct;
  fs.s_dd = (acc_t + fs.s_d * fs.s_d * r.dkappa * p.d + 2.0 * r.kappa * fs.s_d * fs.d_d) / one_minus;
  fs.d_dd = acc_n - r.kappa * fs.s_d * fs.s_d * one_minus;
  return fs;
}

EgoState ReferenceLine::toCartesian(const FrenetState& fs) const {
  const RefPoint r = at(fs.s);
  const double c = std::cos(r.yaw), sn = std::sin(r.yaw);
  const double one_minus = 1.0 - r.kappa * fs.d;

  const double vt = fs.s_d * one_minus;
  const double vn = fs.d_d;
  const double at_ =
      fs.s_dd * one_minus - fs.s_d * fs.s_d * r.dkappa * fs.d - 2.0 * r.kappa * fs.s_d * fs.d_d;
  const double an = r.kappa * fs.s_d * fs.s_d * one_minus + fs.d_dd;

  EgoState out;
  out.x = r.x - sn * fs.d;
  out.y = r.y + c * fs.d;
  out.v = std::hypot(vt, vn);
  if (out.v > kMinSpeed) {
    out.yaw = normalizeAngle(r.yaw + std::atan2(vn, vt));
    out.a = (vt * at_ + vn * an) / out.v;
    out.kappa = (vt * an - vn * at_) / (out.v * out.v * out.v);
  } else {
    // At standstill heading/curvature follow the offset reference curve.
    out.yaw = r.yaw;
    out.a = at_;
    out.kappa = r.kappa / one_minus;
  }
  return out;
}

namespace {

/// Solve (diag(w) + lambda D'D) p = w .* q for both coordinates, where D takes
/// second differences: a symmetric positive definite pentadiagonal system,
/// factored by a banded Cholesky decomposition.
void whittakerSolve(const std::vector<double>& w, double lambda, const std::vector<double>& qx,
                    const std::vector<double>& qy, std::vector<double>& px, std::vector<double>& py) {
  const size_t n = w.size();
  std::vector<double> a0(w), a1(n, 0.0), a2(n, 0.0);  // A(i,i), A(i,i-1), A(i,i-2)
  for (size_t i = 0; i + 2 < n; ++i) {                // rows [1 -2 1] of D at i, i+1, i+2
    a0[i] += lambda;
    a0[i + 1] += 4.0 * lambda;
    a0[i + 2] += lambda;
    a1[i + 1] -= 2.0 * lambda;
    a1[i + 2] -= 2.0 * lambda;
    a2[i + 2] += lambda;
  }
  std::vector<double> l0(n), l1(n, 0.0), l2(n, 0.0);  // L(i,i), L(i,i-1), L(i,i-2)
  for (size_t i = 0; i < n; ++i) {
    if (i >= 2) l2[i] = a2[i] / l0[i - 2];
    if (i >= 1) l1[i] = (a1[i] - (i >= 2 ? l2[i] * l1[i - 1] : 0.0)) / l0[i - 1];
    l0[i] = std::sqrt(a0[i] - l1[i] * l1[i] - l2[i] * l2[i]);
  }
  const auto solve = [&](const std::vector<double>& q, std::vector<double>& p) {
    p.resize(n);
    for (size_t i = 0; i < n; ++i) {  // L z = w .* q
      double v = w[i] * q[i];
      if (i >= 1) v -= l1[i] * p[i - 1];
      if (i >= 2) v -= l2[i] * p[i - 2];
      p[i] = v / l0[i];
    }
    for (size_t i = n; i-- > 0;) {  // L' p = z
      double v = p[i];
      if (i + 1 < n) v -= l1[i + 1] * p[i + 1];
      if (i + 2 < n) v -= l2[i + 2] * p[i + 2];
      p[i] = v / l0[i];
    }
  };
  solve(qx, px);
  solve(qy, py);
}

}  // namespace

void smoothPolyline(std::vector<double>& x, std::vector<double>& y, double length, double tolerance,
                    double ds) {
  if (x.size() != y.size()) throw std::invalid_argument("smoothPolyline: x/y size mismatch");
  if (!(ds > 0.0)) throw std::invalid_argument("smoothPolyline: ds must be > 0");
  // Piecewise-linear resampling: unlike a spline it cannot overshoot between
  // vertices that are close together.
  std::vector<double> cum = {0.0};
  std::vector<double> vx, vy;
  for (size_t i = 0; i < x.size(); ++i) {
    if (!vx.empty()) {
      const double d = std::hypot(x[i] - vx.back(), y[i] - vy.back());
      if (d < 1e-6) continue;
      cum.push_back(cum.back() + d);
    }
    vx.push_back(x[i]);
    vy.push_back(y[i]);
  }
  if (vx.size() < 2) return;
  const size_t n = std::max<size_t>(2, static_cast<size_t>(std::ceil(cum.back() / ds)) + 1);
  const double h = cum.back() / static_cast<double>(n - 1);
  std::vector<double> qx(n), qy(n);
  size_t seg = 0;
  for (size_t i = 0; i < n; ++i) {
    const double s = std::min(cum.back(), h * static_cast<double>(i));
    while (seg + 2 < cum.size() && cum[seg + 1] < s) ++seg;
    const double u = (s - cum[seg]) / (cum[seg + 1] - cum[seg]);
    qx[i] = vx[seg] + u * (vx[seg + 1] - vx[seg]);
    qy[i] = vy[seg] + u * (vy[seg + 1] - vy[seg]);
  }
  x = qx;
  y = qy;
  if (n < 5 || !(length > 0.0)) return;

  // Second-difference penalty with a cutoff wavelength of `length`: the
  // smoother passes wavelengths w with lambda (2 pi h / w)^4 << 1.
  const double lambda = std::pow(length / (2.0 * kPi * h), 4);
  std::vector<double> w(n, 1.0);
  constexpr int kMaxIterations = 12;
  for (int it = 0; it < kMaxIterations; ++it) {
    whittakerSolve(w, lambda, qx, qy, x, y);
    if (!(tolerance > 0.0)) return;
    bool within = true;
    for (size_t i = 0; i < n; ++i) {
      const double e = std::hypot(x[i] - qx[i], y[i] - qy[i]);
      if (e > tolerance) {
        w[i] *= (e / tolerance) * (e / tolerance);
        within = false;
      }
    }
    if (within) return;
  }
}

}  // namespace fiss_planner
