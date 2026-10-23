// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "frenet_planner/geometry/polynomial.hpp"

using frenet_planner::QuarticPolynomial;
using frenet_planner::QuinticPolynomial;

namespace {
template <typename P>
void expectDerivativesConsistent(const P& p, double T) {
  constexpr double h = 1e-5;
  for (double t = 0.1; t < T; t += 0.37) {
    EXPECT_NEAR(p.dx(t), (p.x(t + h) - p.x(t - h)) / (2 * h), 1e-5) << "t=" << t;
    EXPECT_NEAR(p.ddx(t), (p.dx(t + h) - p.dx(t - h)) / (2 * h), 1e-5) << "t=" << t;
    EXPECT_NEAR(p.dddx(t), (p.ddx(t + h) - p.ddx(t - h)) / (2 * h), 1e-5) << "t=" << t;
  }
}
}  // namespace

TEST(QuinticPolynomial, MatchesBoundaryConditions) {
  const double T = 4.2;
  const QuinticPolynomial p(1.0, -0.5, 0.3, 3.5, 0.2, -0.1, T);
  EXPECT_NEAR(p.x(0), 1.0, 1e-12);
  EXPECT_NEAR(p.dx(0), -0.5, 1e-12);
  EXPECT_NEAR(p.ddx(0), 0.3, 1e-12);
  EXPECT_NEAR(p.x(T), 3.5, 1e-9);
  EXPECT_NEAR(p.dx(T), 0.2, 1e-9);
  EXPECT_NEAR(p.ddx(T), -0.1, 1e-9);
}

TEST(QuinticPolynomial, DerivativesMatchFiniteDifferences) {
  expectDerivativesConsistent(QuinticPolynomial(0.0, 1.0, -0.4, 2.0, 0.0, 0.0, 5.0), 5.0);
}

TEST(QuarticPolynomial, MatchesBoundaryConditions) {
  const double T = 3.0;
  const QuarticPolynomial p(10.0, 5.0, -1.0, 12.0, 0.5, T);
  EXPECT_NEAR(p.x(0), 10.0, 1e-12);
  EXPECT_NEAR(p.dx(0), 5.0, 1e-12);
  EXPECT_NEAR(p.ddx(0), -1.0, 1e-12);
  EXPECT_NEAR(p.dx(T), 12.0, 1e-9);
  EXPECT_NEAR(p.ddx(T), 0.5, 1e-9);
}

TEST(QuarticPolynomial, DerivativesMatchFiniteDifferences) {
  expectDerivativesConsistent(QuarticPolynomial(0.0, 8.0, 0.5, 13.0, 0.0, 4.0), 4.0);
}

// The second derivative at t = 0 must equal the initial acceleration; every
// acceleration constraint check relies on it.
TEST(Polynomial, SecondDerivativeAtStartEqualsInitialAcceleration) {
  EXPECT_NEAR(QuinticPolynomial(0, 0, 2.5, 1, 0, 0, 3).ddx(0.0), 2.5, 1e-12);
  EXPECT_NEAR(QuarticPolynomial(0, 0, -1.5, 5, 0, 3).ddx(0.0), -1.5, 1e-12);
}

TEST(Polynomial, RejectsNonPositiveDuration) {
  EXPECT_THROW(QuinticPolynomial(0, 0, 0, 1, 0, 0, 0.0), std::invalid_argument);
  EXPECT_THROW(QuarticPolynomial(0, 0, 0, 1, 0, -1.0), std::invalid_argument);
}
