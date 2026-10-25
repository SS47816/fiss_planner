// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <random>

#include "fiss_planner/collision/box2d.hpp"
#include "fiss_planner/common/math.hpp"
#include "fiss_planner/prediction/obstacle.hpp"

using namespace fiss_planner;

namespace {
// Brute-force reference: two convex quads overlap iff a corner of one lies in
// the other or two edges intersect.
bool pointInBox(const Box2d& b, const Vec2& p) {
  const double c = std::cos(b.yaw), s = std::sin(b.yaw);
  const double dx = p.x - b.cx, dy = p.y - b.cy;
  return std::abs(dx * c + dy * s) <= 0.5 * b.length + 1e-12 &&
         std::abs(-dx * s + dy * c) <= 0.5 * b.width + 1e-12;
}
double cross(const Vec2& o, const Vec2& a, const Vec2& b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}
bool segmentsIntersect(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2) {
  const double d1 = cross(q1, q2, p1), d2 = cross(q1, q2, p2);
  const double d3 = cross(p1, p2, q1), d4 = cross(p1, p2, q2);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}
bool bruteForceOverlap(const Box2d& a, const Box2d& b) {
  const auto ca = a.corners(), cb = b.corners();
  for (const auto& p : ca)
    if (pointInBox(b, p)) return true;
  for (const auto& p : cb)
    if (pointInBox(a, p)) return true;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (segmentsIntersect(ca[i], ca[(i + 1) % 4], cb[j], cb[(j + 1) % 4])) return true;
  return false;
}
}  // namespace

TEST(Box2d, CornersAreCounterClockwise) {
  const Box2d b{0.0, 0.0, 0.0, 4.0, 2.0};
  const auto c = b.corners();
  EXPECT_NEAR(c[0].x, 2.0, 1e-12);
  EXPECT_NEAR(c[0].y, 1.0, 1e-12);
  EXPECT_NEAR(c[2].x, -2.0, 1e-12);
  EXPECT_NEAR(c[2].y, -1.0, 1e-12);
}

TEST(Box2d, SatAgreesWithBruteForce) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> pos(-6.0, 6.0), yaw(-kPi, kPi), ext(0.5, 5.0);
  int overlaps_seen = 0;
  for (int i = 0; i < 20000; ++i) {
    const Box2d a{pos(rng), pos(rng), yaw(rng), ext(rng), ext(rng)};
    const Box2d b{pos(rng), pos(rng), yaw(rng), ext(rng), ext(rng)};
    const bool expected = bruteForceOverlap(a, b);
    overlaps_seen += expected;
    ASSERT_EQ(overlaps(a, b), expected) << "case " << i;
  }
  EXPECT_GT(overlaps_seen, 1000);  // the sample covers both outcomes
}

TEST(ObstaclePrediction, ConstantVelocityAdvancesBothAxes) {
  const auto ob = predictConstantVelocity(3, 1.0, 2.0, kPi / 4, std::sqrt(2.0), 4.0, 2.0, 5.0, 0.1);
  ASSERT_EQ(ob.prediction.size(), 51u);
  EXPECT_NEAR(ob.prediction.back().x, 6.0, 1e-9);
  EXPECT_NEAR(ob.prediction.back().y, 7.0, 1e-9);
  const auto box = ob.boxAt(2.55);
  ASSERT_TRUE(box);
  EXPECT_NEAR(box->cx, 3.55, 1e-9);
  EXPECT_NEAR(box->cy, 4.55, 1e-9);
}

TEST(Obstacle, HorizonSemantics) {
  auto dynamic = predictConstantVelocity(1, 0, 0, 0, 10.0, 4, 2, 2.0, 0.5);
  EXPECT_TRUE(dynamic.boxAt(2.0));
  EXPECT_FALSE(dynamic.boxAt(3.0));  // unknown beyond the prediction horizon

  auto parked = predictConstantVelocity(2, 5, 5, 0, 0.0, 4, 2, 2.0, 0.5);
  EXPECT_TRUE(parked.is_static);
  ASSERT_TRUE(parked.boxAt(100.0));
  EXPECT_NEAR(parked.boxAt(100.0)->cx, 5.0, 1e-12);

  Obstacle single;
  single.length = 1;
  single.width = 1;
  single.prediction.push_back({0.0, 1.0, 1.0, 0.0, 3.0});
  EXPECT_TRUE(single.boxAt(4.0));  // single pose is held
}
