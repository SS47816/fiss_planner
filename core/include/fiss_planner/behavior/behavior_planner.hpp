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
#include "fiss_planner/planning/corridor.hpp"
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
  /// Blocked lane: standing obstacles in the ego lane, closer than
  /// blocked_lookahead and leaving no room to pass within it, that are not a
  /// queue (no junction or active stop line just beyond). The ego lane then
  /// costs blocked_cost more, and an oncoming band (RouteCorridorConfig::
  /// oncoming_lane) may be used at oncoming_cost while no oncoming traffic
  /// would meet the ego before it has passed the blocked stretch (crossing_gap
  /// to spare), and until it has.
  bool pass_blocked = true;
  double blocked_lookahead = 50.0;  ///< [m]
  double blocked_cost = 50.0;
  double oncoming_cost = 10.0;
  /// While blocked, staying in the ego lane ends this far [m] before the
  /// nearest blocker: the ego waits there with room to pull out.
  double pass_standoff = 8.0;
};

struct BehaviorInput {
  const ReferenceLine* reference = nullptr;
  FrenetState start;  ///< ego on the reference
  const std::vector<RouteCorridor::StopLineAhead>* stop_lines = nullptr;
  const std::vector<std::pair<double, double>>* junctions = nullptr;
  const std::vector<Obstacle>* obstacles = nullptr;  ///< predictions relative to now
  const Corridor* corridor = nullptr;                ///< lane bands (for blocked lanes)
  double dt = 0.1;                                   ///< planning period [s]
};

/// What the behaviours ask of the trajectory planner.
struct BehaviorDirective {
  std::vector<double> stops;             ///< stations on the reference to stop before
  std::vector<int> released_stop_lines;  ///< active stop lines that may be passed
  bool pass_blocked = false;             ///< the ego lane is blocked: prefer other lanes, oncoming ones too
  /// The blocked stretch on the reference: from the nearest blocker's rear to the farthest's front.
  double blocked_s = 0.0, blocked_end = 0.0;
  /// No oncoming traffic before the ego can pass the stretch (or it is passing already).
  bool oncoming_allowed = false;
  std::vector<std::string> active;  ///< behaviours that applied, in tree order
};

/// The behaviour tree, ticked once per cycle:
///
///   Sequence
///     Optional  pass yellow light   (yellow ahead, stopping uncomfortable -> release it)
///     Optional  stop sign           (stop at the line; release it after standing still)
///     Optional  yield at junction   (junction ahead, not committed, traffic in the path -> stop at entry)
///     Optional  pass blocked lane   (standing obstacles block the ego lane -> leave it, oncoming too)
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
    int junction = -1;     ///< next junction not yet entered (index into junctions)
    bool blocked = false;  ///< the ego lane is blocked ahead
    double blocked_s = 0.0, blocked_end = 0.0;
  };

 private:
  bool canStopBefore(const BehaviorInput& in, double s, double decel) const;
  /// Time for the ego to reach station s (accelerating as configured).
  double arrival(const BehaviorInput& in, double s) const;
  bool crossingTraffic(const BehaviorInput& in, double s_in, double s_out) const;
  /// The nearest obstacle blocking the ego lane, if any (fills b.blocked_*).
  bool laneBlocked(const BehaviorInput& in, Board& b) const;
  bool oncomingClear(const BehaviorInput& in, const Board& b) const;

  BehaviorConfig cfg_;
  VehicleParams vehicle_;
  behavior::NodePtr<Board> tree_;
  std::unordered_set<int> served_signs_;
  double standing_ = 0.0;  ///< time standing at the current stop sign [s]
};

}  // namespace fiss_planner
