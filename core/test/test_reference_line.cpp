// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "fiss_planner/common/math.hpp"
#include "fiss_planner/geometry/reference_line.hpp"
#include "test_utils.hpp"

using namespace fiss_planner;

TEST(ReferenceLine, StraightLineProjection) {
  const auto ref = test::straightLine(100.0);
  const auto p = ref.project(30.0, 2.0);
  EXPECT_NEAR(p.s, 30.0, 1e-6);
  EXPECT_NEAR(p.d, 2.0, 1e-6);
  const auto q = ref.project(30.0, -1.5);
  EXPECT_NEAR(q.d, -1.5, 1e-6);  // right is negative
}

// Projection is well defined everywhere, including before and past the line.
TEST(ReferenceLine, ProjectionBehindStartAndBeyondEnd) {
  const auto ref = test::straightLine(50.0);
  const auto behind = ref.project(-5.0, 1.0);
  EXPECT_NEAR(behind.s, -5.0, 1e-6);
  EXPECT_NEAR(behind.d, 1.0, 1e-6);
  const auto beyond = ref.project(60.0, -2.0);
  EXPECT_NEAR(beyond.s, 60.0, 1e-6);
  EXPECT_NEAR(beyond.d, -2.0, 1e-6);
}

TEST(ReferenceLine, ArcLengthIsAccurate) {
  const auto ref = test::arc(40.0, kPi / 2);
  EXPECT_NEAR(ref.length(), 40.0 * kPi / 2, 0.02);
  for (double s = 2.0; s < ref.length() - 2.0; s += 5.0) {
    EXPECT_NEAR(ref.at(s).kappa, 1.0 / 40.0, 2e-4);
  }
}

TEST(ReferenceLine, ProjectionOnCurveIsOrthogonal) {
  const auto ref = test::arc(30.0, kPi);
  for (double s = 3.0; s < ref.length() - 3.0; s += 4.1) {
    for (double d : {-2.0, 0.0, 1.5}) {
      const auto r = ref.at(s);
      const double x = r.x - std::sin(r.yaw) * d;
      const double y = r.y + std::cos(r.yaw) * d;
      const auto p = ref.project(x, y);
      EXPECT_NEAR(p.s, s, 1e-5);
      EXPECT_NEAR(p.d, d, 1e-5);
    }
  }
}

TEST(ReferenceLine, ProjectionHintDisambiguatesLoops) {
  // A U-turn whose two legs are 10 m apart.
  const auto ref = test::uTurn(5.0, 40.0);
  const auto r = ref.at(10.0);
  const auto p = ref.project(r.x, r.y, 10.0);
  EXPECT_NEAR(p.s, 10.0, 1e-4);
}

TEST(ReferenceLine, CartesianFrenetRoundTrip) {
  const auto ref = test::sCurve();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> us(10.0, ref.length() - 10.0), ud(-3.0, 3.0), uv(0.5, 25.0),
      ua(-3.0, 3.0), uyaw(-0.4, 0.4), uk(-0.05, 0.05);
  for (int i = 0; i < 300; ++i) {
    const auto r = ref.at(us(rng));
    const double d = ud(rng);
    EgoState st;
    st.x = r.x - std::sin(r.yaw) * d;
    st.y = r.y + std::cos(r.yaw) * d;
    st.yaw = normalizeAngle(r.yaw + uyaw(rng));
    st.v = uv(rng);
    st.a = ua(rng);
    st.kappa = uk(rng);
    const FrenetState fs = ref.toFrenet(st);
    const EgoState back = ref.toCartesian(fs);
    EXPECT_NEAR(back.x, st.x, 1e-6);
    EXPECT_NEAR(back.y, st.y, 1e-6);
    EXPECT_NEAR(normalizeAngle(back.yaw - st.yaw), 0.0, 1e-8);
    EXPECT_NEAR(back.v, st.v, 1e-8);
    EXPECT_NEAR(back.a, st.a, 1e-7);
    EXPECT_NEAR(back.kappa, st.kappa, 1e-8);
  }
}

TEST(ReferenceLine, FrenetMotionMatchesCartesianFiniteDifferences) {
  // Integrate a Frenet motion, convert each sample, and compare the analytic
  // Cartesian speed/heading/curvature with finite differences of positions.
  const auto ref = test::sCurve();
  const double dt = 1e-3;
  auto fs_at = [](double t) {
    FrenetState fs;
    fs.s = 20.0 + 8.0 * t + 0.3 * t * t;
    fs.s_d = 8.0 + 0.6 * t;
    fs.s_dd = 0.6;
    fs.d = 0.5 * std::sin(t);
    fs.d_d = 0.5 * std::cos(t);
    fs.d_dd = -0.5 * std::sin(t);
    return fs;
  };
  for (double t = 0.5; t < 5.0; t += 0.5) {
    const auto p0 = ref.toCartesian(fs_at(t - dt));
    const auto p1 = ref.toCartesian(fs_at(t));
    const auto p2 = ref.toCartesian(fs_at(t + dt));
    const double vx = (p2.x - p0.x) / (2 * dt), vy = (p2.y - p0.y) / (2 * dt);
    const double ax = (p2.x - 2 * p1.x + p0.x) / (dt * dt), ay = (p2.y - 2 * p1.y + p0.y) / (dt * dt);
    const double v = std::hypot(vx, vy);
    EXPECT_NEAR(p1.v, v, 1e-4);
    EXPECT_NEAR(normalizeAngle(p1.yaw - std::atan2(vy, vx)), 0.0, 1e-5);
    EXPECT_NEAR(p1.kappa, (vx * ay - vy * ax) / (v * v * v), 1e-3);
    EXPECT_NEAR(p1.a, (vx * ax + vy * ay) / v, 1e-2);
  }
}

namespace {

double maxAbsCurvature(const ReferenceLine& ref, double margin) {
  double k = 0.0;
  for (double s = margin; s <= ref.length() - margin; s += 0.5) k = std::max(k, std::abs(ref.at(s).kappa));
  return k;
}

}  // namespace

// Digitized lanes: a straight lane whose vertices jitter by a few centimetres
// and sometimes nearly coincide. Interpolated, the jitter becomes curvature
// spikes; smoothed, the line is straight again and stays within tolerance.
TEST(SmoothPolyline, RemovesVertexNoiseWithinTolerance) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> jitter(-0.05, 0.05);
  std::vector<double> x, y;
  for (double s = 0.0; s <= 200.0; s += 2.5) {
    x.push_back(s);
    y.push_back(jitter(rng));
    if (static_cast<int>(s) % 20 == 0) {  // near-duplicate vertex
      x.push_back(s + 0.01);
      y.push_back(jitter(rng));
    }
  }
  const ReferenceLine raw(x, y);
  std::vector<double> sx = x, sy = y;
  smoothPolyline(sx, sy, 20.0, 0.1);
  const ReferenceLine smooth(sx, sy);
  EXPECT_GT(maxAbsCurvature(raw, 10.0), 0.02);
  EXPECT_LT(maxAbsCurvature(smooth, 10.0), 0.003);
  for (size_t i = 0; i < sx.size(); ++i) EXPECT_LT(std::abs(sy[i]), 0.1 + 1e-6);
  EXPECT_NEAR(sx.back(), x.back(), 0.01);
}

// Real curves keep their shape: a 15 m radius quarter turn between straights
// moves by about the tolerance at most and keeps its curvature.
TEST(SmoothPolyline, KeepsRealCurvesWithinTolerance) {
  const double R = 15.0;
  std::vector<double> x, y;
  for (double s = -30.0; s < 0.0; s += 3.0) {
    x.push_back(s);
    y.push_back(0.0);
  }
  for (double a = 0.0; a < 0.5 * kPi; a += 0.1) {
    x.push_back(R * std::sin(a));
    y.push_back(R * (1.0 - std::cos(a)));
  }
  for (double s = 0.0; s <= 30.0; s += 3.0) {
    x.push_back(R);
    y.push_back(R + s);
  }
  std::vector<double> sx = x, sy = y;
  smoothPolyline(sx, sy, 20.0, 0.1);
  const ReferenceLine original(x, y);
  for (size_t i = 0; i < sx.size(); ++i) EXPECT_LT(std::abs(original.project(sx[i], sy[i]).d), 0.105);
  const ReferenceLine smooth(sx, sy);
  const double s_mid = smooth.project(R * std::sin(0.25 * kPi), R * (1.0 - std::cos(0.25 * kPi))).s;
  EXPECT_NEAR(smooth.at(s_mid).kappa, 1.0 / R, 0.15 / R);
}

TEST(SmoothPolyline, ShortOrDisabledOnlyResamples) {
  std::vector<double> x = {0.0, 3.0}, y = {0.0, 0.0};
  smoothPolyline(x, y, 20.0, 0.1);
  ASSERT_EQ(x.size(), 4u);  // resampled every metre
  EXPECT_NEAR(x[1], 1.0, 1e-9);
  std::vector<double> zx = {0.0, 10.0, 20.0}, zy = {0.0, 1.0, 0.0};
  smoothPolyline(zx, zy, 0.0, 0.1);  // no smoothing: the corner is kept
  EXPECT_GT(zx.size(), 20u);
  EXPECT_GT(*std::max_element(zy.begin(), zy.end()), 0.9);
}
