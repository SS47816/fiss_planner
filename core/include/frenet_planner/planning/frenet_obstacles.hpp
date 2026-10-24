// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// Obstacles projected into the Frenet frame of the reference line once per
// planning cycle, so that gap costs and the safety envelope of every candidate
// are cheap interval tests instead of polygon checks.
#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "frenet_planner/geometry/reference_line.hpp"
#include "frenet_planner/planning/config.hpp"
#include "frenet_planner/planning/trajectory.hpp"
#include "frenet_planner/prediction/obstacle.hpp"

namespace frenet_planner {

/// Extent of a footprint along (s) and across (d) the reference line.
struct FrenetBox {
  double s_min = 0.0, s_max = 0.0, d_min = 0.0, d_max = 0.0;
};

/// Exact Frenet extent of a box (projects its four corners).
FrenetBox toFrenetBox(const Box2d& box, const ReferenceLine& ref, double s_hint);

/// Predicted Frenet extent and along-track speed of one obstacle on a coarse
/// time grid (t = 0, dt, 2 dt, ...), interpolated linearly in between.
struct FrenetTrack {
  int id = -1;
  size_t index = 0;  ///< position in the obstacle list it was built from
  double dt = 0.5;
  bool holds_last = false;  ///< static or single-pose obstacles keep their last extent
  std::vector<FrenetBox> boxes;
  std::vector<double> s_d;    ///< [m/s]
  std::vector<char> in_line;  ///< heading along the reference line (within 45 deg) or static

  /// Extent and speed at time t; false beyond the prediction. `in_line_out`
  /// (optional) tells whether the obstacle can be followed at that time; gap
  /// and time-to-collision rules apply only to those, while crossing and
  /// oncoming traffic is left to the collision check.
  bool at(double t, FrenetBox& box, double& speed, bool* in_line_out = nullptr) const;
};

/// Gap to the closest obstacle ahead in the ego's path.
struct LeadGap {
  int id = -1;                                           ///< -1 if there is none
  double speed = 0.0;                                    ///< its speed along the reference line [m/s]
  double gap = std::numeric_limits<double>::infinity();  ///< bumper to bumper [m]
  double time_headway = std::numeric_limits<double>::infinity();  ///< gap / ego speed [s]
  double ttc = std::numeric_limits<double>::infinity();           ///< gap / closing speed [s]
};

class FrenetObstacles {
 public:
  FrenetObstacles() = default;
  /// Keeps obstacles whose predicted extent comes within [s_lo, s_hi] x
  /// [d_lo, d_hi] at some time up to `horizon`.
  FrenetObstacles(const std::vector<Obstacle>& obstacles, const ReferenceLine& ref, double s_lo, double s_hi,
                  double d_lo, double d_hi, double horizon, double dt = 0.5);

  bool empty() const { return tracks_.empty(); }
  const std::vector<FrenetTrack>& tracks() const { return tracks_; }

 private:
  std::vector<FrenetTrack> tracks_;
};

/// Proximity cost terms, the safety envelope and gap measurements, all for an
/// ego footprint of the vehicle's length and (width + 2 margin_lat).
class GapEvaluator {
 public:
  GapEvaluator() = default;
  GapEvaluator(const CostConfig& cost, const SafetyConfig& safety, const VehicleParams& vehicle,
               double margin_lat);

  /// Weighted proximity cost (>= 0) of a trajectory's Frenet arrays.
  double cost(const Trajectory& traj, const FrenetObstacles& obstacles) const;
  /// Safety envelope (see SafetyConfig) over samples with t <= max_t. On a
  /// violation, writes the obstacle id and time to the trajectory's blocking
  /// fields and returns false.
  bool isSafe(Trajectory& traj, const FrenetObstacles& obstacles,
              double max_t = std::numeric_limits<double>::infinity()) const;
  /// Whether an ego at station s moving at s_d would breach the safety envelope
  /// of an obstacle ahead with extent `box` and speed `v_ob` if it shared its lane band.
  bool breachesEnvelope(double s, double s_d, const FrenetBox& box, double v_ob) const;
  /// Gap to the closest obstacle ahead of an ego at (s, d) moving at s_d, at
  /// time t. By default "ahead" means overlapping the ego's lateral band; with
  /// band_lo < band_hi it means having its center in [band_lo, band_hi] (a lane).
  LeadGap leadGap(double s, double d, double s_d, const FrenetObstacles& obstacles, double t = 0.0,
                  double band_lo = 0.0, double band_hi = 0.0) const;

 private:
  CostConfig w_;
  SafetyConfig safety_;
  double half_length_ = 2.25;
  double half_width_ = 0.9;
  double half_band_ = 1.2;  ///< half width plus lateral margin
};

/// Flags (indexed like the original obstacle list) of obstacles that start
/// entirely behind an ego at station s with their center inside the ego lane
/// [lane_right, lane_left]. Obstacles without a track are not flagged.
std::vector<char> rearInLane(const FrenetObstacles& obstacles, size_t num_obstacles, double s,
                             double half_length, double lane_right, double lane_left);

}  // namespace frenet_planner
