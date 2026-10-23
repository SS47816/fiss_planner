// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner/geometry/polynomial.hpp"

#include <stdexcept>

namespace frenet_planner {

// Closed-form solutions of the boundary-value problems. With
// u_k = c_k * T^k for the unknown coefficients, the linear systems reduce to
// small integer systems that are solved symbolically below.

QuinticPolynomial::QuinticPolynomial(double x0, double v0, double a0, double x1, double v1, double a1,
                                     double T) {
  if (!(T > 0.0)) throw std::invalid_argument("QuinticPolynomial: T must be positive");
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double h = x1 - (x0 + v0 * T + 0.5 * a0 * T2);
  const double V = (v1 - (v0 + a0 * T)) * T;
  const double A = (a1 - a0) * T2;
  c_[0] = x0;
  c_[1] = v0;
  c_[2] = 0.5 * a0;
  c_[3] = (10.0 * h - 4.0 * V + 0.5 * A) / T3;
  c_[4] = (-15.0 * h + 7.0 * V - A) / (T3 * T);
  c_[5] = (6.0 * h - 3.0 * V + 0.5 * A) / (T3 * T2);
}

double QuinticPolynomial::x(double t) const {
  return c_[0] + t * (c_[1] + t * (c_[2] + t * (c_[3] + t * (c_[4] + t * c_[5]))));
}

double QuinticPolynomial::dx(double t) const {
  return c_[1] + t * (2.0 * c_[2] + t * (3.0 * c_[3] + t * (4.0 * c_[4] + t * 5.0 * c_[5])));
}

double QuinticPolynomial::ddx(double t) const {
  return 2.0 * c_[2] + t * (6.0 * c_[3] + t * (12.0 * c_[4] + t * 20.0 * c_[5]));
}

double QuinticPolynomial::dddx(double t) const { return 6.0 * c_[3] + t * (24.0 * c_[4] + t * 60.0 * c_[5]); }

QuarticPolynomial::QuarticPolynomial(double x0, double v0, double a0, double v1, double a1, double T) {
  if (!(T > 0.0)) throw std::invalid_argument("QuarticPolynomial: T must be positive");
  const double dv = v1 - (v0 + a0 * T);
  const double da = a1 - a0;
  c_[0] = x0;
  c_[1] = v0;
  c_[2] = 0.5 * a0;
  c_[3] = dv / (T * T) - da / (3.0 * T);
  c_[4] = da / (4.0 * T * T) - dv / (2.0 * T * T * T);
}

double QuarticPolynomial::x(double t) const {
  return c_[0] + t * (c_[1] + t * (c_[2] + t * (c_[3] + t * c_[4])));
}

double QuarticPolynomial::dx(double t) const {
  return c_[1] + t * (2.0 * c_[2] + t * (3.0 * c_[3] + t * 4.0 * c_[4]));
}

double QuarticPolynomial::ddx(double t) const { return 2.0 * c_[2] + t * (6.0 * c_[3] + t * 12.0 * c_[4]); }

double QuarticPolynomial::dddx(double t) const { return 6.0 * c_[3] + t * 24.0 * c_[4]; }

}  // namespace frenet_planner
