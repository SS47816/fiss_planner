// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <array>
#include <limits>
#include <optional>
#include <vector>

namespace fiss_planner {

/// 1-D trajectory optimization on a uniform time grid with piecewise-constant
/// jerk (the formulation of Apollo's piecewise-jerk path/speed optimizers):
///
///   min  sum_i  w_x (x_i - x_ref_i)^2 + w_dx (dx_i - dx_ref_i)^2 + w_ddx ddx_i^2
///             + w_dddx ((ddx_{i+1} - ddx_i) / dt)^2
///        + w_end_dx dx_N^2 + w_end_ddx ddx_N^2
///   s.t. x_i in [x_lo_i, x_hi_i], dx_i in [dx_lo, dx_hi], ddx_i in [ddx_lo, ddx_hi],
///        |jerk| <= dddx_max, kinematic consistency between knots, fixed x_0.
struct PiecewiseJerkProblem {
  double dt = 0.1;
  std::array<double, 3> x0{0.0, 0.0, 0.0};  ///< (x, dx, ddx) at i = 0
  std::vector<double> x_ref;                ///< defines the number of knots
  std::vector<double> dx_ref;               ///< optional (same size as x_ref)
  std::vector<double> x_lo, x_hi;           ///< optional per-knot bounds
  double dx_lo = -std::numeric_limits<double>::infinity();
  double dx_hi = std::numeric_limits<double>::infinity();
  double ddx_lo = -std::numeric_limits<double>::infinity();
  double ddx_hi = std::numeric_limits<double>::infinity();
  double dddx_max = std::numeric_limits<double>::infinity();
  double w_x = 1.0, w_dx = 0.0, w_ddx = 1.0, w_dddx = 1.0;
  double w_end_dx = 0.0, w_end_ddx = 0.0;
};

struct PiecewiseJerkSolution {
  std::vector<double> x, dx, ddx, dddx;
};

/// Solve with OSQP. Returns nullopt if the problem is infeasible, the solver
/// fails, or the library was built without OSQP.
std::optional<PiecewiseJerkSolution> solvePiecewiseJerk(const PiecewiseJerkProblem& problem);

/// Whether the library was built with the OSQP-based optimizers.
bool hasOptimizers();

}  // namespace fiss_planner
