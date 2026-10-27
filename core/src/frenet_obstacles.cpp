// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/planning/frenet_obstacles.hpp"

#include <algorithm>
#include <cmath>

#include "fiss_planner/common/math.hpp"

namespace fiss_planner {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

double sq(double x) { return x * x; }

const double kInLineCos = std::cos(kPi / 4.0);  // within 45 degrees of the reference heading

/// (1 - actual / desired)^2 when actual < desired, else 0.
double shortfall(double actual, double desired) { return sq(std::max(0.0, 1.0 - actual / desired)); }
}  // namespace

FrenetBox toFrenetBox(const Box2d& box, const ReferenceLine& ref, double s_hint) {
  FrenetBox fb{kInf, -kInf, kInf, -kInf};
  for (const auto& c : box.corners()) {
    const auto p = ref.project(c.x, c.y, s_hint);
    fb.s_min = std::min(fb.s_min, p.s);
    fb.s_max = std::max(fb.s_max, p.s);
    fb.d_min = std::min(fb.d_min, p.d);
    fb.d_max = std::max(fb.d_max, p.d);
  }
  return fb;
}

bool FrenetTrack::at(double t, FrenetBox& box, double& speed, bool* in_line_out) const {
  if (boxes.empty()) return false;
  const double u = std::max(0.0, t) / dt;
  const size_t i0 = static_cast<size_t>(u);
  if (i0 + 1 >= boxes.size()) {
    // Within rounding of the last sample (or a held pose) only.
    if (u > static_cast<double>(boxes.size() - 1) + 1e-6 && !holds_last) return false;
    box = boxes.back();
    speed = s_d.back();
    if (in_line_out != nullptr) *in_line_out = in_line.back();
    return true;
  }
  const double w = u - static_cast<double>(i0);
  if (in_line_out != nullptr) *in_line_out = in_line[w < 0.5 ? i0 : i0 + 1];
  const auto& a = boxes[i0];
  const auto& b = boxes[i0 + 1];
  box = {a.s_min + w * (b.s_min - a.s_min), a.s_max + w * (b.s_max - a.s_max),
         a.d_min + w * (b.d_min - a.d_min), a.d_max + w * (b.d_max - a.d_max)};
  speed = s_d[i0] + w * (s_d[i0 + 1] - s_d[i0]);
  return true;
}

FrenetObstacles::FrenetObstacles(const std::vector<Obstacle>& obstacles, const ReferenceLine& ref,
                                 double s_lo, double s_hi, double d_lo, double d_hi, double horizon,
                                 double dt) {
  const int n = static_cast<int>(std::floor(horizon / dt + 1e-9)) + 1;
  std::vector<double> centers;
  for (size_t k = 0; k < obstacles.size(); ++k) {
    const auto& ob = obstacles[k];
    FrenetTrack track;
    track.id = ob.id;
    track.index = k;
    track.dt = dt;
    track.holds_last = ob.is_static || ob.prediction.size() == 1;
    centers.clear();
    bool relevant = false;
    double hint = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < n; ++i) {
      const auto box = ob.boxAt(i * dt);
      if (!box) break;
      // Project the center only; the extent follows from the heading relative
      // to the reference line (exact on straight references).
      const Projection p = ref.project(box->cx, box->cy, hint);
      hint = p.s;
      const double rel = box->yaw - ref.at(p.s).yaw;
      const double c = std::abs(std::cos(rel)), sn = std::abs(std::sin(rel));
      const double hs = 0.5 * (box->length * c + box->width * sn);
      const double hd = 0.5 * (box->length * sn + box->width * c);
      const FrenetBox fb{p.s - hs, p.s + hs, p.d - hd, p.d + hd};
      relevant = relevant || (fb.s_max >= s_lo && fb.s_min <= s_hi && fb.d_max >= d_lo && fb.d_min <= d_hi);
      track.boxes.push_back(fb);
      track.in_line.push_back(track.holds_last || std::cos(rel) > kInLineCos);
      centers.push_back(p.s);
    }
    if (!relevant) continue;
    const size_t m = centers.size();
    track.s_d.assign(m, 0.0);
    for (size_t i = 0; m > 1 && i < m; ++i) {
      const size_t a = i == 0 ? 0 : i - 1;
      const size_t b = std::min(i + 1, m - 1);
      track.s_d[i] = (centers[b] - centers[a]) / (static_cast<double>(b - a) * dt);
    }
    tracks_.push_back(std::move(track));
  }
}

GapEvaluator::GapEvaluator(const CostConfig& cost, const SafetyConfig& safety, const VehicleParams& vehicle,
                           double margin_lat)
    : w_(cost),
      safety_(safety),
      half_length_(0.5 * vehicle.length),
      half_width_(0.5 * vehicle.width),
      half_band_(0.5 * vehicle.width + margin_lat) {}

double GapEvaluator::cost(const Trajectory& traj, const FrenetObstacles& obstacles) const {
  const size_t n = traj.size();
  if (obstacles.empty() || n == 0 || (w_.w_headway <= 0.0 && w_.w_ttc <= 0.0 && w_.w_clearance <= 0.0)) {
    return 0.0;
  }
  // Every ~0.2 s is plenty for terms this smooth.
  const double dt = n > 1 ? traj.t[1] - traj.t[0] : 0.1;
  const size_t stride = static_cast<size_t>(std::max(1.0, std::round(0.2 / dt)));
  double headway = 0.0, ttc = 0.0, clearance = 0.0;
  int count = 0;
  FrenetBox fb;
  double v_ob = 0.0;
  for (size_t i = 0; i < n; i += stride, ++count) {
    const double s = traj.s[i], d = traj.d[i], v = std::max(0.0, traj.s_d[i]);
    const double desired_gap = w_.desired_gap + w_.desired_time_headway * v;
    double h = 0.0, c = 0.0, l = 0.0;
    for (const auto& track : obstacles.tracks()) {
      bool in_line = false;
      if (!track.at(traj.t[i], fb, v_ob, &in_line)) continue;
      if (fb.d_max > d - half_band_ && fb.d_min < d + half_band_) {
        if (!in_line || fb.s_min + fb.s_max <= 2.0 * s) continue;  // crossing, oncoming or behind
        const double gap = std::max(0.0, fb.s_min - s - half_length_);
        h = std::max(h, shortfall(gap, desired_gap));
        const double closing = v - v_ob;
        if (closing > 1e-3) c = std::max(c, shortfall(gap / closing, w_.desired_ttc));
      } else if (fb.s_max > s - half_length_ - 1.0 && fb.s_min < s + half_length_ + 1.0) {
        const double lateral = fb.d_min >= d ? fb.d_min - d - half_width_ : d - half_width_ - fb.d_max;
        l = std::max(l, shortfall(std::max(0.0, lateral), w_.desired_clearance));
      }
    }
    headway += h;
    ttc += c;
    clearance += l;
  }
  return (w_.w_headway * headway + w_.w_ttc * ttc + w_.w_clearance * clearance) / count;
}

namespace {

/// Walks the safety envelope of a trajectory. `on_breach(k, t, severity)` is
/// called for every sample and obstacle in breach (severity in (0, 1]) and
/// returns whether to stop.
template <typename OnBreach>
void walkEnvelope(const Trajectory& traj, const FrenetObstacles& obstacles, double max_t,
                  const SafetyConfig& safety, double half_length, double half_band, OnBreach&& on_breach) {
  const size_t n = traj.size();
  if (!safety.enabled || obstacles.empty() || n == 0) return;
  const auto& tracks = obstacles.tracks();
  const auto in_band = [half_band](const FrenetBox& b, double d) {
    return b.d_max > d - half_band && b.d_min < d + half_band;
  };
  // An obstacle that is in the ego's path from the start, or that moves into
  // the lane band the ego starts in (a cut-in), grants a reaction time from
  // when it appears and may not be approached closer than it appeared.
  // Obstacles the ego moves towards itself (by changing lanes) grant neither.
  std::vector<double> check_from(tracks.size(), safety.reaction_time);
  std::vector<double> allowance(tracks.size(), std::numeric_limits<double>::infinity());
  std::vector<char> seen(tracks.size(), 0);
  FrenetBox fb;
  double v_ob = 0.0;
  for (size_t i = 0; i < n && traj.t[i] <= max_t; ++i) {
    const double t = traj.t[i], s = traj.s[i], d = traj.d[i], v = std::max(0.0, traj.s_d[i]);
    const double required = safety.min_gap + safety.min_time_headway * v;
    for (size_t k = 0; k < tracks.size(); ++k) {
      bool in_line = false;
      if (!tracks[k].at(t, fb, v_ob, &in_line) || !in_line || !in_band(fb, d) ||
          fb.s_min + fb.s_max <= 2.0 * s) {
        continue;
      }
      const double gap = fb.s_min - s - half_length;
      if (!seen[k]) {
        seen[k] = 1;
        if (i == 0 || in_band(fb, traj.d[0])) {
          allowance[k] = gap;
          check_from[k] = t + safety.reaction_time;
        }
      }
      if (t < check_from[k] - 1e-9) continue;
      // Pulling out (moving laterally away from it) is not following it, so the
      // gap floor is waived; the time to collision must still hold in case the
      // lane change cannot be completed. Standing still never closes a gap.
      constexpr double kStandstill = 0.1;  // [m/s]
      const bool pulling_out =
          traj.d_d[i] * (d - 0.5 * (fb.d_min + fb.d_max)) > 0.0 && std::abs(traj.d_d[i]) > 0.2;
      const double floor = std::min(required, allowance[k]);
      double severity = 0.0;
      if (!pulling_out && v > kStandstill && gap < floor) {
        severity = std::clamp((floor - gap) / std::max(floor, 1e-3), 1e-6, 1.0);
      }
      const double closing = v - v_ob;
      if (closing > 1e-3 && gap < safety.min_ttc * closing) {
        severity = std::max(severity, std::clamp(1.0 - gap / (safety.min_ttc * closing), 1e-6, 1.0));
      }
      if (severity > 0.0 && on_breach(tracks[k].id, t, severity)) return;
    }
  }
}

}  // namespace

bool GapEvaluator::isSafe(Trajectory& traj, const FrenetObstacles& obstacles, double max_t) const {
  bool safe = true;
  walkEnvelope(traj, obstacles, max_t, safety_, half_length_, half_band_, [&](int id, double t, double) {
    traj.blocking_id = id;
    traj.blocking_t = t;
    safe = false;
    return true;
  });
  return safe;
}

double GapEvaluator::breach(const Trajectory& traj, const FrenetObstacles& obstacles, double max_t) const {
  double worst = 0.0;
  walkEnvelope(traj, obstacles, max_t, safety_, half_length_, half_band_, [&](int, double, double severity) {
    worst = std::max(worst, severity);
    return worst >= 1.0;
  });
  return worst;
}

bool GapEvaluator::breachesEnvelope(double s, double s_d, const FrenetBox& box, double v_ob) const {
  if (!safety_.enabled || box.s_min + box.s_max <= 2.0 * s) return false;
  const double v = std::max(0.0, s_d);
  const double gap = box.s_min - s - half_length_;
  const double closing = v - v_ob;
  return gap < safety_.min_gap + safety_.min_time_headway * v ||
         (closing > 1e-3 && gap < safety_.min_ttc * closing);
}

LeadGap GapEvaluator::leadGap(double s, double d, double s_d, const FrenetObstacles& obstacles, double t,
                              double band_lo, double band_hi) const {
  LeadGap lead;
  FrenetBox fb;
  double v_ob = 0.0;
  const double v = std::max(0.0, s_d);
  const bool lane_mode = band_lo < band_hi;
  for (const auto& track : obstacles.tracks()) {
    bool in_line = false;
    if (!track.at(t, fb, v_ob, &in_line) || !in_line || fb.s_min + fb.s_max <= 2.0 * s) continue;
    if (lane_mode) {
      const double center = 0.5 * (fb.d_min + fb.d_max);
      if (center < band_lo || center > band_hi) continue;
      // An obstacle at the lane's edge (a cone, a parked car's mirror) that leaves
      // room to pass within the lane does not set its pace.
      if (std::max(band_hi - fb.d_max, fb.d_min - band_lo) >= 2.0 * half_band_) continue;
    } else if (fb.d_max <= d - half_band_ || fb.d_min >= d + half_band_) {
      continue;
    }
    const double gap = std::max(0.0, fb.s_min - s - half_length_);
    if (gap >= lead.gap) continue;
    lead.id = track.id;
    lead.speed = v_ob;
    lead.gap = gap;
    lead.time_headway = v > 0.1 ? gap / v : kInf;
    lead.ttc = v - v_ob > 1e-3 ? gap / (v - v_ob) : kInf;
  }
  return lead;
}

std::vector<char> rearInLane(const FrenetObstacles& obstacles, size_t num_obstacles, double s,
                             double half_length, double lane_right, double lane_left) {
  std::vector<char> flags(num_obstacles, 0);
  FrenetBox fb;
  double v_ob = 0.0;
  for (const auto& track : obstacles.tracks()) {
    if (!track.at(0.0, fb, v_ob)) continue;
    const double d_center = 0.5 * (fb.d_min + fb.d_max);
    if (fb.s_max < s - half_length && d_center > lane_right && d_center < lane_left) flags[track.index] = 1;
  }
  return flags;
}

}  // namespace fiss_planner
