// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "fiss_planner/behavior/behavior_tree.hpp"
#include "fiss_planner/common/types.hpp"
#include "fiss_planner/geometry/reference_line.hpp"
#include "fiss_planner/map/route_corridor.hpp"
#include "fiss_planner/prediction/obstacle.hpp"

namespace fiss_planner {

/// Decisions taken before trajectory sampling (the behaviour layer). Off by
/// default: the planner then treats every active stop line as a stop.
struct BehaviorConfig {
  bool enabled = false;
  /// Junctions: stop before the entry while crossing or oncoming traffic would
  /// be in the ego's path through the junction; once inside, go on.
  bool junction_yield = true;
  double yield_lookahead = 40.0;  ///< [m] junctions closer than this (or 6 s of driving) are checked
  double crossing_gap = 1.5;      ///< [s] time margin to traffic in the ego's path through a junction
  double go_accel = 1.5;          ///< [m/s^2] acceleration assumed for the ego through a junction ...
  double turn_speed = 6.0;        ///< [m/s] ... up to this speed (or its current one, if faster)
  double stop_margin = 1.0;       ///< [m] stop this far before a junction entry
  /// Yellow lights are passed, and junctions committed to, when stopping
  /// before them would take more than this deceleration [m/s^2].
  double comfort_decel = 3.0;
  double stop_sign_wait = 0.5;  ///< [s] standing at a stop sign before going on
};

struct BehaviorInput {
  const ReferenceLine* reference = nullptr;
  FrenetState start;  ///< ego on the reference
  const std::vector<RouteCorridor::StopLineAhead>* stop_lines = nullptr;
  const std::vector<std::pair<double, double>>* junctions = nullptr;
  const std::vector<Obstacle>* obstacles = nullptr;  ///< predictions relative to now
  double dt = 0.1;                                   ///< planning period [s]
};

/// What the behaviours ask of the trajectory planner.
struct BehaviorDirective {
  std::vector<double> stops;             ///< stations on the reference to stop before
  std::vector<int> released_stop_lines;  ///< active stop lines that may be passed
  std::vector<std::string> active;       ///< behaviours that applied, in tree order
};

/// The behaviour tree, ticked once per cycle:
///
///   Sequence
///     Optional  pass yellow light   (yellow ahead, stopping uncomfortable -> release it)
///     Optional  stop sign           (stop at the line; release it after standing still)
///     Optional  yield at junction   (junction ahead, not committed, traffic in the path -> stop at entry)
///     Action    proceed
class BehaviorPlanner {
 public:
  BehaviorPlanner(const BehaviorConfig& config, const VehicleParams& vehicle);
  BehaviorDirective plan(const BehaviorInput& input);
  void reset();

  /// Board shared by the tree's nodes (exposed for tests).
  struct Board {
    const BehaviorInput* in = nullptr;
    BehaviorDirective out;
    int sign = -1;  ///< nearest unserved stop sign ahead (index into stop_lines)
    int yellow = -1;
    int junction = -1;  ///< next junction not yet entered (index into junctions)
  };

 private:
  bool canStopBefore(const BehaviorInput& in, double s, double decel) const;
  /// Time for the ego to reach station s (accelerating as configured).
  double arrival(const BehaviorInput& in, double s) const;
  bool crossingTraffic(const BehaviorInput& in, double s_in, double s_out) const;

  BehaviorConfig cfg_;
  VehicleParams vehicle_;
  behavior::NodePtr<Board> tree_;
  std::unordered_set<int> served_signs_;
  double standing_ = 0.0;  ///< time standing at the current stop sign [s]
};

}  // namespace fiss_planner
