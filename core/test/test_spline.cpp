// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>

#include "frenet_planner/common/math.hpp"
#include "frenet_planner/geometry/spline.hpp"

using namespace frenet_planner;

TEST(CubicSpline1D, InterpolatesKnots) {
  const std::vector<double> x = {0.0, 1.0, 2.5, 4.0, 7.0, 8.0};
  const std::vector<double> y = {1.0, 3.0, -2.0, 0.5, 4.0, 4.5};
  const CubicSpline1D sp(x, y);
  for (size_t i = 0; i < x.size(); ++i) EXPECT_NEAR(sp(x[i]), y[i], 1e-12);
}

TEST(CubicSpline1D, ReproducesLinearData) {
  std::vector<double> x, y;
  for (int i = 0; i < 10; ++i) {
    x.push_back(i * 1.3);
    y.push_back(2.0 * x.back() - 1.0);
  }
  const CubicSpline1D sp(x, y);
  for (double t = 0.0; t < 11.0; t += 0.21) {
    EXPECT_NEAR(sp(t), 2.0 * t - 1.0, 1e-9);
    EXPECT_NEAR(sp.d1(t), 2.0, 1e-9);
  }
}

TEST(CubicSpline1D, ApproximatesSmoothFunctionWithDerivatives) {
  std::vector<double> x, y;
  for (int i = 0; i <= 60; ++i) {
    x.push_back(i * 0.1);
    y.push_back(std::sin(x.back()));
  }
  const CubicSpline1D sp(x, y);
  for (double t = 0.5; t < 5.5; t += 0.13) {
    EXPECT_NEAR(sp(t), std::sin(t), 1e-5);
    EXPECT_NEAR(sp.d1(t), std::cos(t), 1e-3);
    EXPECT_NEAR(sp.d2(t), -std::sin(t), 2e-2);
  }
}

class SplineKnotCount : public ::testing::TestWithParam<int> {};

TEST_P(SplineKnotCount, WorksForAnyKnotCount) {
  const int n = GetParam();
  std::vector<double> x, y;
  for (int i = 0; i < n; ++i) {
    x.push_back(i * 2.0);
    y.push_back(std::cos(i * 0.4));
  }
  const CubicSpline1D sp(x, y);
  for (int i = 0; i < n; ++i) EXPECT_NEAR(sp(x[i]), y[i], 1e-12);
}

INSTANTIATE_TEST_SUITE_P(KnotCounts, SplineKnotCount, ::testing::Values(2, 3, 4, 5, 6, 17, 200));

TEST(CubicSpline1D, ExtrapolatesLinearlyBeyondTheKnots) {
  const CubicSpline1D sp({0.0, 1.0, 2.0, 3.0}, {10.0, 11.0, 12.0, 13.0});
  EXPECT_NEAR(sp(5.0), 15.0, 1e-9);
  EXPECT_NEAR(sp(-1.0), 9.0, 1e-9);
  EXPECT_NEAR(sp.d1(10.0), 1.0, 1e-9);
}

TEST(CubicSpline1D, RejectsBadInput) {
  EXPECT_THROW(CubicSpline1D({0.0}, {1.0}), std::invalid_argument);
  EXPECT_THROW(CubicSpline1D({0.0, 1.0, 1.0}, {1.0, 2.0, 3.0}), std::invalid_argument);
  EXPECT_THROW(CubicSpline1D({0.0, 1.0}, {1.0}), std::invalid_argument);
}

TEST(CubicSpline2D, CircleCurvature) {
  const double R = 25.0;
  std::vector<double> x, y;
  for (int i = 0; i <= 90; ++i) {
    const double a = deg2rad(i * 2.0);
    x.push_back(R * std::cos(a));
    y.push_back(R * std::sin(a));
  }
  const CubicSpline2D sp(x, y);
  EXPECT_NEAR(sp.length(), kPi * R, 0.05);
  for (double s = 5.0; s < sp.length() - 5.0; s += 3.0) {
    EXPECT_NEAR(sp.curvature(s), 1.0 / R, 1e-3);
    EXPECT_NEAR(sp.curvatureRate(s), 0.0, 1e-3);
  }
}

TEST(CubicSpline2D, DropsDuplicatePoints) {
  const CubicSpline2D sp({0.0, 1.0, 1.0, 2.0}, {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(sp.s().size(), 3u);
  EXPECT_NEAR(sp.length(), 2.0, 1e-12);
}
