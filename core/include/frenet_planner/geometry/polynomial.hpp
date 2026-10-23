// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <array>

namespace frenet_planner {

/// Quintic polynomial x(t) matching position, velocity and acceleration at
/// t = 0 and t = T. Used for lateral motion.
class QuinticPolynomial {
 public:
  QuinticPolynomial() = default;
  QuinticPolynomial(double x0, double v0, double a0, double x1, double v1, double a1, double T);

  double x(double t) const;
  double dx(double t) const;
  double ddx(double t) const;
  double dddx(double t) const;
  const std::array<double, 6>& coefficients() const { return c_; }

 private:
  std::array<double, 6> c_{};
};

/// Quartic polynomial x(t) matching position, velocity and acceleration at
/// t = 0 and velocity and acceleration at t = T. Used for velocity keeping.
class QuarticPolynomial {
 public:
  QuarticPolynomial() = default;
  QuarticPolynomial(double x0, double v0, double a0, double v1, double a1, double T);

  double x(double t) const;
  double dx(double t) const;
  double ddx(double t) const;
  double dddx(double t) const;
  const std::array<double, 5>& coefficients() const { return c_; }

 private:
  std::array<double, 5> c_{};
};

}  // namespace frenet_planner
