// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// Minimal sparse QP helper on the OSQP 1.x C API (internal):
//   min 0.5 z'Pz + q'z  s.t.  lo <= A z <= hi
#pragma once

#ifdef FRENET_PLANNER_HAS_OSQP

#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace frenet_planner::detail {

/// Dense vector with call-style indexing.
struct Vec {
  std::vector<double> v;
  double& operator()(int i) { return v[static_cast<size_t>(i)]; }
  double operator()(int i) const { return v[static_cast<size_t>(i)]; }
};

class SparseQp {
 public:
  explicit SparseQp(int n) : num_vars(n) { q.v.assign(static_cast<size_t>(n), 0.0); }

  /// Append a constraint row lo <= sum(coef * z[idx]) <= hi; returns its index.
  int addRow(std::initializer_list<std::pair<int, double>> terms, double l, double h) {
    const int r = static_cast<int>(lo_.size());
    for (const auto& [i, c] : terms) a_.emplace_back(r, i, c);
    lo_.push_back(finite(l));
    hi_.push_back(finite(h));
    return r;
  }
  /// Add an entry to the upper triangle of P (duplicates are summed).
  void addQuadratic(int i, int j, double w) {
    if (i > j) std::swap(i, j);
    p_.emplace_back(i, j, w);
  }

  std::optional<Vec> solve(int max_iter = 4000, double eps = 1e-5) const {
    const auto n = static_cast<OSQPInt>(num_vars);
    const auto m = static_cast<OSQPInt>(lo_.size());
    Csc P = toCsc(p_, n, n), A = toCsc(a_, m, n);
    std::vector<OSQPFloat> q_(q.v.begin(), q.v.end()), l(lo_.begin(), lo_.end()), u(hi_.begin(), hi_.end());
    OSQPCscMatrix Pm{n, n, P.p.data(), P.i.data(), P.x.data(), static_cast<OSQPInt>(P.x.size()), -1};
    OSQPCscMatrix Am{m, n, A.p.data(), A.i.data(), A.x.data(), static_cast<OSQPInt>(A.x.size()), -1};
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_starting = 1;
    settings.polishing = 1;
    settings.max_iter = max_iter;
    settings.eps_abs = eps;
    settings.eps_rel = eps;
    OSQPSolver* solver = nullptr;
    if (osqp_setup(&solver, &Pm, q_.data(), &Am, l.data(), u.data(), m, n, &settings) != 0 || !solver) {
      if (solver) osqp_cleanup(solver);
      return std::nullopt;
    }
    osqp_solve(solver);
    const OSQPInt status = solver->info->status_val;
    std::optional<Vec> out;
    if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE) {
      out = Vec{std::vector<double>(solver->solution->x, solver->solution->x + n)};
    }
    osqp_cleanup(solver);
    return out;
  }

  int num_vars;
  Vec q;

 private:
  using Triplet = std::tuple<int, int, double>;
  struct Csc {
    std::vector<OSQPInt> p, i;
    std::vector<OSQPFloat> x;
  };

  static double finite(double v) { return std::isfinite(v) ? v : (v > 0 ? OSQP_INFTY : -OSQP_INFTY); }

  /// Column-compressed form with duplicate entries summed.
  static Csc toCsc(std::vector<Triplet> t, OSQPInt rows, OSQPInt cols) {
    (void)rows;
    std::sort(t.begin(), t.end(), [](const Triplet& a, const Triplet& b) {
      return std::get<1>(a) != std::get<1>(b) ? std::get<1>(a) < std::get<1>(b)
                                              : std::get<0>(a) < std::get<0>(b);
    });
    Csc c;
    c.p.assign(static_cast<size_t>(cols) + 1, 0);
    for (size_t k = 0; k < t.size(); ++k) {
      const auto [r, col, v] = t[k];
      if (!c.i.empty() && k > 0 && std::get<0>(t[k - 1]) == r && std::get<1>(t[k - 1]) == col) {
        c.x.back() += v;
        continue;
      }
      c.i.push_back(r);
      c.x.push_back(v);
      ++c.p[static_cast<size_t>(col) + 1];
    }
    for (size_t j = 0; j < static_cast<size_t>(cols); ++j) c.p[j + 1] += c.p[j];
    return c;
  }

  std::vector<Triplet> p_, a_;
  std::vector<double> lo_, hi_;
};

}  // namespace frenet_planner::detail

#endif
