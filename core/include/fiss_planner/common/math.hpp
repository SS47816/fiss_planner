// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace fiss_planner {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

/// Wrap an angle into [-pi, pi).
inline double normalizeAngle(double angle) {
  double a = std::fmod(angle + kPi, kTwoPi);
  if (a < 0.0) a += kTwoPi;
  return a - kPi;
}

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }
inline double kph2mps(double kph) { return kph / 3.6; }
inline double mps2kph(double mps) { return mps * 3.6; }

inline bool isFinite(double x) { return std::isfinite(x); }

/// Evenly spaced samples over [lo, hi], endpoints included (like numpy.linspace).
/// A single sample is placed at the midpoint; n <= 0 yields no samples.
inline std::vector<double> linspace(double lo, double hi, int n) {
  std::vector<double> out;
  if (n <= 0) return out;
  out.reserve(static_cast<size_t>(n));
  if (n == 1) {
    out.push_back(0.5 * (lo + hi));
    return out;
  }
  const double step = (hi - lo) / static_cast<double>(n - 1);
  for (int i = 0; i < n; ++i) out.push_back(lo + step * static_cast<double>(i));
  out.back() = hi;  // exact endpoint, no accumulated rounding
  return out;
}

/// Sample times 0, dt, 2dt, ... up to and including T (within rounding).
inline std::vector<double> timeSamples(double T, double dt) {
  std::vector<double> out;
  if (T < 0.0 || dt <= 0.0) return out;
  const int n = static_cast<int>(std::floor(T / dt + 1e-9)) + 1;
  out.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) out.push_back(dt * static_cast<double>(i));
  return out;
}

}  // namespace fiss_planner
