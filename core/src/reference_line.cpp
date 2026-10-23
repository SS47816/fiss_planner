// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/geometry/reference_line.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "frenet_planner/common/math.hpp"

namespace frenet_planner {

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

}  // namespace frenet_planner
