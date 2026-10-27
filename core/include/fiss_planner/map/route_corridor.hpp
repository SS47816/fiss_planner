// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <optional>
#include <vector>

#include "fiss_planner/common/types.hpp"
#include "fiss_planner/geometry/reference_line.hpp"
#include "fiss_planner/map/road_map.hpp"
#include "fiss_planner/planning/corridor.hpp"
#include "fiss_planner/prediction/obstacle.hpp"

namespace fiss_planner {

struct RouteCorridorConfig {
  double lookahead = 150.0;          ///< reference length ahead of the ego [m]
  double lookbehind = 20.0;          ///< reference length behind the ego [m]
  double station_ds = 1.0;           ///< corridor station spacing [m]
  double smoothing_length = 20.0;    ///< reference smoothing removes wiggles shorter than this [m]; 0 = off
  double smoothing_tolerance = 0.1;  ///< largest deviation of the reference from the centerlines [m]
  int max_adjacent_lanes = 1;        ///< lanes considered on each side of the ego lane
  double w_route_urgency = 10.0;     ///< cost per lane change still needed with half its ramp distance left
  double urgency_distance = 80.0;    ///< distance over which the urgency of one lane change ramps up [m] ...
  double urgency_time = 10.0;        ///< ... or the distance driven in this time, if longer [s]
  /// Stop lines become obstacles this far beyond the line [m], so that the
  /// vehicle, which stops a gap short of obstacles, stops close to the line.
  double stop_line_offset = 0.0;
};

/// Everything the planner needs from the map for one cycle.
struct RouteCorridor {
  ReferenceLine reference;           ///< centerline of the ego lane chain
  Corridor corridor;                 ///< ego lane and adjacent lanes as lateral bands
  std::vector<int> ego_chain;        ///< lane ids along the reference
  int ego_lane = -1;                 ///< lane containing the ego
  double ego_s = 0.0;                ///< ego arc length on the reference
  double speed_limit = 0.0;          ///< of the ego lane [m/s] (inf if unknown)
  std::vector<Obstacle> stop_lines;  ///< active stop lines as static virtual obstacles
  /// Per entry of stop_lines: the map's stop line and its station on the reference.
  struct StopLineAhead {
    int id = -1;
    double s = 0.0;
    bool yellow = false;
    bool stop_sign = false;
  };
  std::vector<StopLineAhead> stop_line_info;
  /// Stretches [s_in, s_out] of the reference through junction lanes, in order.
  std::vector<std::pair<double, double>> junctions;
};

/// Build the reference line and corridor around the ego from a lane map and
/// an (optional) route given as lane ids in driving order, ending in the
/// destination lane. The ego lane chain follows route lanes where possible;
/// adjacent lanes become bands whose accessibility comes from the map's
/// lane-change permissions (or from the route passing through them).
///
/// Route urgency: along each band, lane changes fall due wherever the band
/// cannot go on as it is: where a route lane leaves the route (the changes the
/// route makes from there before it continues lengthwise), where a band off
/// the route ends or can no longer reach it sideways (counted over permitted
/// changes), at the end of a dead-end road, and at `goal` (a point in the
/// destination lane) if given, also beyond the reference line. For n changes
/// due in a distance D, with U the larger of urgency_distance and
/// urgency_time times the ego speed, the urgency is u = clamp(1 - D / (n U),
/// 0, 1) and the cost w n u / (1 - u), capped at 10 w n: zero while the
/// changes fit comfortably, w n with U / 2 per change left, and growing
/// steeply towards the deadline, so the more changes are needed, the earlier
/// they are made. A band takes its most urgent deadline. Lanes off the route
/// cost another w, so that leaving the route (to pass slower traffic) has to
/// be worth it.
std::optional<RouteCorridor> buildRouteCorridor(const RoadMap& map, const std::vector<int>& route,
                                                const EgoState& ego, const RouteCorridorConfig& cfg,
                                                int previous_ego_lane = -1,
                                                const std::optional<Vec2>& goal = std::nullopt);

}  // namespace fiss_planner
