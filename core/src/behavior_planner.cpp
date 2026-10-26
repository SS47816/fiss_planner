// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "fiss_planner/behavior/behavior_planner.hpp"

#include <algorithm>
#include <cmath>

#include "fiss_planner/common/math.hpp"

namespace fiss_planner {

using behavior::Status;
using Board = BehaviorPlanner::Board;

namespace {
constexpr double kStandstill = 0.1;            // [m/s]
constexpr double kMovingSpeed = 0.5;           // [m/s]
constexpr double kStopGap = 3.0;               // [m] standstill gap the planner keeps to a stop
constexpr double kSignReach = kStopGap + 2.0;  // [m] stopped this close before a stop sign counts
constexpr double kSameDirection =
    0.5;  // [rad] traffic within this heading of the ego lane is followed, not yielded to
}  // namespace

BehaviorPlanner::BehaviorPlanner(const BehaviorConfig& config, const VehicleParams& vehicle)
    : cfg_(config), vehicle_(vehicle) {
  using namespace behavior;
  const double half_length = 0.5 * vehicle_.length;

  auto yellow =
      sequence<Board>("pass yellow light",
                      condition<Board>("yellow light ahead", [](const Board& b) { return b.yellow >= 0; }),
                      condition<Board>("stopping is uncomfortable",
                                       [this](const Board& b) {
                                         const auto& sl = (*b.in->stop_lines)[b.yellow];
                                         return !canStopBefore(*b.in, sl.s, cfg_.comfort_decel);
                                       }),
                      action<Board>("release the stop line", [](Board& b) {
                        b.out.released_stop_lines.push_back((*b.in->stop_lines)[b.yellow].id);
                        b.out.active.push_back("pass_yellow");
                        return Status::kSuccess;
                      }));

  auto sign = sequence<Board>(
      "stop sign", condition<Board>("stop sign ahead", [](const Board& b) { return b.sign >= 0; }),
      action<Board>("stop, then go", [this, half_length](Board& b) {
        const auto& sl = (*b.in->stop_lines)[b.sign];
        const bool at_line = sl.s - (b.in->start.s + half_length) < kSignReach;
        standing_ = at_line && b.in->start.s_d < kStandstill ? standing_ + b.in->dt : 0.0;
        if (standing_ >= cfg_.stop_sign_wait) {
          served_signs_.insert(sl.id);
          standing_ = 0.0;
        }
        if (served_signs_.count(sl.id)) {
          b.out.released_stop_lines.push_back(sl.id);
          b.out.active.push_back("stop_sign_go");
        } else {
          b.out.active.push_back("stop_sign_stop");
        }
        return Status::kSuccess;
      }));

  auto junction = sequence<Board>(
      "yield at junction",
      condition<Board>("junction ahead",
                       [this](const Board& b) { return cfg_.junction_yield && b.junction >= 0; }),
      condition<Board>("can still stop before it",
                       [this](const Board& b) {
                         const double s_in = (*b.in->junctions)[b.junction].first;
                         return canStopBefore(*b.in, s_in - cfg_.stop_margin, cfg_.comfort_decel);
                       }),
      condition<Board>("traffic in the path",
                       [this](const Board& b) {
                         const auto& [s_in, s_out] = (*b.in->junctions)[b.junction];
                         return crossingTraffic(*b.in, s_in, s_out);
                       }),
      action<Board>("stop at the entry", [this](Board& b) {
        b.out.stops.push_back((*b.in->junctions)[b.junction].first - cfg_.stop_margin);
        b.out.active.push_back("yield_junction");
        return Status::kSuccess;
      }));

  tree_ = sequence<Board>(
      "behaviour", optional<Board>("yellow", std::move(yellow)), optional<Board>("sign", std::move(sign)),
      optional<Board>("junction", std::move(junction)), action<Board>("proceed", [](Board& b) {
        if (b.out.active.empty()) b.out.active.push_back("proceed");
        return Status::kSuccess;
      }));
}

void BehaviorPlanner::reset() {
  served_signs_.clear();
  standing_ = 0.0;
}

BehaviorDirective BehaviorPlanner::plan(const BehaviorInput& in) {
  Board b;
  b.in = &in;
  const double front = in.start.s + 0.5 * vehicle_.length;
  if (in.stop_lines != nullptr) {
    for (size_t i = 0; i < in.stop_lines->size(); ++i) {
      const auto& sl = (*in.stop_lines)[i];
      const auto nearest = [&](int k) { return k < 0 || sl.s < (*in.stop_lines)[k].s; };
      if (sl.yellow && nearest(b.yellow)) b.yellow = static_cast<int>(i);
      if (sl.stop_sign && nearest(b.sign)) b.sign = static_cast<int>(i);
    }
  }
  if (b.sign < 0) standing_ = 0.0;
  if (in.junctions != nullptr) {
    const double reach = std::max(cfg_.yield_lookahead, 6.0 * in.start.s_d);
    for (size_t i = 0; i < in.junctions->size(); ++i) {
      const auto& [s_in, s_out] = (*in.junctions)[i];
      if (s_out <= in.start.s) continue;
      // Committed once the front is in; too far away to matter yet.
      if (front < s_in && s_in - front < reach) b.junction = static_cast<int>(i);
      break;
    }
  }
  tree_->tick(b);
  return b.out;
}

bool BehaviorPlanner::canStopBefore(const BehaviorInput& in, double s, double decel) const {
  // The trajectory planner stops a gap short of a stop line or virtual stop.
  const double dist = s - kStopGap - (in.start.s + 0.5 * vehicle_.length);
  const double v = std::max(0.0, in.start.s_d);
  if (dist <= 0.0) return v < kStandstill;
  return v * v / (2.0 * dist) <= decel;
}

double BehaviorPlanner::arrival(const BehaviorInput& in, double s) const {
  const double d = s - in.start.s;
  if (d <= 0.0) return 0.0;
  const double v0 = std::max(0.0, in.start.s_d), vmax = std::max(cfg_.turn_speed, v0), a = cfg_.go_accel;
  const double t_acc = (vmax - v0) / a, d_acc = v0 * t_acc + 0.5 * a * t_acc * t_acc;
  if (d <= d_acc) return (-v0 + std::sqrt(v0 * v0 + 2.0 * a * d)) / a;
  return t_acc + (d - d_acc) / vmax;
}

bool BehaviorPlanner::crossingTraffic(const BehaviorInput& in, double s_in, double s_out) const {
  const ReferenceLine& ref = *in.reference;
  const double L = vehicle_.length, W = vehicle_.width;
  for (const auto& ob : *in.obstacles) {
    if (ob.id < 0 || ob.is_static || ob.prediction.empty()) continue;
    // Standing vehicles are obstacles for the trajectory planner, not traffic to yield to.
    const bool moving = std::any_of(ob.prediction.begin(), ob.prediction.end(),
                                    [](const ObstacleState& p) { return p.v >= kMovingSpeed; });
    if (!moving) continue;
    // Traffic along the ego's lane (ahead or behind) is followed, not yielded to.
    const auto& p0 = ob.prediction.front();
    const auto f0 = ref.project(p0.x, p0.y, s_in);
    const double rel = std::abs(normalizeAngle(p0.yaw - ref.at(f0.s).yaw));
    if (rel < kSameDirection && std::abs(f0.d) < W) continue;
    const double reach = 0.5 * (L + ob.length);
    double hint = s_in;
    for (const auto& p : ob.prediction) {
      const auto f = ref.project(p.x, p.y, hint);
      hint = f.s;
      if (f.s < s_in - reach || f.s > s_out + reach) continue;
      if (std::abs(f.d) > 0.5 * (W + ob.width) + 0.5) continue;
      // The ego occupies station f.s from when its front gets there until its rear has passed.
      const double t_from = arrival(in, f.s - reach) - cfg_.crossing_gap;
      const double t_to = arrival(in, f.s + reach) + cfg_.crossing_gap;
      if (p.t >= t_from && p.t <= t_to) return true;
    }
  }
  return false;
}

}  // namespace fiss_planner
