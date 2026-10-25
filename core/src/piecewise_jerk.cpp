// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/smoothing/piecewise_jerk.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "qp.hpp"

namespace fiss_planner {

bool hasOptimizers() {
#ifdef FISS_PLANNER_HAS_OSQP
  return true;
#else
  return false;
#endif
}

#ifndef FISS_PLANNER_HAS_OSQP

std::optional<PiecewiseJerkSolution> solvePiecewiseJerk(const PiecewiseJerkProblem&) { return std::nullopt; }

#else

std::optional<PiecewiseJerkSolution> solvePiecewiseJerk(const PiecewiseJerkProblem& p) {
  const int n = static_cast<int>(p.x_ref.size());
  if (n < 2 || !(p.dt > 0.0)) return std::nullopt;
  const double dt = p.dt, dt2 = dt * dt;
  const auto X = [](int i) { return i; };
  const auto DX = [n](int i) { return n + i; };
  const auto DDX = [n](int i) { return 2 * n + i; };
  detail::SparseQp qp(3 * n);  // [x_0..x_{n-1}, dx_0..dx_{n-1}, ddx_0..ddx_{n-1}]

  // Objective (P holds the upper triangle of the Hessian, i.e. 2 * weights).
  const double wj = p.w_dddx / dt2;
  for (int i = 0; i < n; ++i) {
    qp.addQuadratic(X(i), X(i), 2.0 * p.w_x);
    qp.q(X(i)) = -2.0 * p.w_x * p.x_ref[i];
    qp.addQuadratic(DX(i), DX(i), 2.0 * (p.w_dx + (i == n - 1 ? p.w_end_dx : 0.0)));
    if (!p.dx_ref.empty()) qp.q(DX(i)) = -2.0 * p.w_dx * p.dx_ref[i];
    const double jerk_diag = (i > 0 ? wj : 0.0) + (i < n - 1 ? wj : 0.0);
    qp.addQuadratic(DDX(i), DDX(i), 2.0 * (p.w_ddx + (i == n - 1 ? p.w_end_ddx : 0.0) + jerk_diag));
    if (i < n - 1) qp.addQuadratic(DDX(i), DDX(i + 1), -2.0 * wj);
  }

  // Box constraints; knot 0 is widened to contain the fixed initial state.
  const auto bound = [](const std::vector<double>& v, int i, double dflt) { return v.empty() ? dflt : v[i]; };
  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    double lo = bound(p.x_lo, i, -kInf), hi = bound(p.x_hi, i, kInf);
    if (i == 0) lo = std::min(lo, p.x0[0]), hi = std::max(hi, p.x0[0]);
    qp.addRow({{X(i), 1.0}}, lo, hi);
    qp.addRow({{DX(i), 1.0}}, i == 0 ? std::min(p.dx_lo, p.x0[1]) : p.dx_lo,
              i == 0 ? std::max(p.dx_hi, p.x0[1]) : p.dx_hi);
    qp.addRow({{DDX(i), 1.0}}, i == 0 ? std::min(p.ddx_lo, p.x0[2]) : p.ddx_lo,
              i == 0 ? std::max(p.ddx_hi, p.x0[2]) : p.ddx_hi);
  }
  for (int i = 0; i + 1 < n; ++i) {
    qp.addRow({{DDX(i + 1), 1.0}, {DDX(i), -1.0}}, -p.dddx_max * dt, p.dddx_max * dt);
    // dx_{i+1} = dx_i + (ddx_i + ddx_{i+1}) dt / 2
    qp.addRow({{DX(i + 1), 1.0}, {DX(i), -1.0}, {DDX(i), -0.5 * dt}, {DDX(i + 1), -0.5 * dt}}, 0.0, 0.0);
    // x_{i+1} = x_i + dx_i dt + ddx_i dt^2 / 3 + ddx_{i+1} dt^2 / 6
    qp.addRow({{X(i + 1), 1.0}, {X(i), -1.0}, {DX(i), -dt}, {DDX(i), -dt2 / 3.0}, {DDX(i + 1), -dt2 / 6.0}},
              0.0, 0.0);
  }
  qp.addRow({{X(0), 1.0}}, p.x0[0], p.x0[0]);
  qp.addRow({{DX(0), 1.0}}, p.x0[1], p.x0[1]);
  qp.addRow({{DDX(0), 1.0}}, p.x0[2], p.x0[2]);

  const auto z = qp.solve();
  if (!z) return std::nullopt;
  PiecewiseJerkSolution sol;
  sol.x.resize(n);
  sol.dx.resize(n);
  sol.ddx.resize(n);
  sol.dddx.assign(n, 0.0);
  for (int i = 0; i < n; ++i) {
    sol.x[i] = (*z)(X(i));
    sol.dx[i] = (*z)(DX(i));
    sol.ddx[i] = (*z)(DDX(i));
    if (i + 1 < n) sol.dddx[i] = ((*z)(DDX(i + 1)) - (*z)(DDX(i))) / dt;
  }
  sol.dddx[n - 1] = sol.dddx[n - 2];
  return sol;
}

#endif

}  // namespace fiss_planner
