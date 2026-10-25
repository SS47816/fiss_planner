// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <limits>
#include <utility>
#include <vector>

namespace fiss_planner {

/// A lane expressed as a lateral band [right(s), left(s)] in the Frenet frame
/// of the reference line, sampled at the corridor stations.
struct LaneBand {
  int offset = 0;              ///< relative to the ego lane: 0 keep, +1 left, -1 right
  int lane_id = -1;            ///< id of the lane in the source map (informational)
  bool change_allowed = true;  ///< whether the planner may sample/enter this lane
  double extra_cost = 0.0;     ///< added to trajectories ending in this lane (e.g. route urgency)
  double speed_limit = std::numeric_limits<double>::infinity();  ///< [m/s]
  std::vector<double> center, left, right;                       ///< lateral offsets per station [m]
};

/// Drivable corridor along the reference line: a set of lane bands sampled at
/// common stations. The union of allowed lanes forms the road bounds used by
/// the off-road check; the bands drive lateral sampling.
class Corridor {
 public:
  Corridor() = default;
  explicit Corridor(std::vector<double> stations) : s_(std::move(stations)) {}

  /// Single lane with constant bounds [d_right, d_left] between s_start and s_end.
  static Corridor uniform(double s_start, double s_end, double d_right, double d_left, double ds = 1.0);
  /// `num_left` + 1 + `num_right` parallel lanes of `lane_width`; the ego lane
  /// is centered on the reference line.
  static Corridor parallelLanes(double s_start, double s_end, double lane_width, int num_left, int num_right,
                                double ds = 1.0);

  void addLane(LaneBand lane);

  bool empty() const { return s_.empty() || lanes_.empty(); }
  const std::vector<double>& stations() const { return s_; }
  const std::vector<LaneBand>& lanes() const { return lanes_; }
  std::vector<LaneBand>& lanes() { return lanes_; }

  /// Lane band of the ego lane (offset 0), or nullptr.
  const LaneBand* egoLane() const;

  /// Interpolated (right, center, left) of a lane at station s (clamped at the ends).
  double center(const LaneBand& lane, double s) const { return interp(lane.center, s); }
  double left(const LaneBand& lane, double s) const { return interp(lane.left, s); }
  double right(const LaneBand& lane, double s) const { return interp(lane.right, s); }

  /// Union of allowed lanes at station s as (d_min, d_max).
  std::pair<double, double> roadBounds(double s) const;

 private:
  double interp(const std::vector<double>& values, double s) const;

  std::vector<double> s_;
  std::vector<LaneBand> lanes_;
};

}  // namespace fiss_planner
