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
  double lookahead = 150.0;        ///< reference length ahead of the ego [m]
  double lookbehind = 20.0;        ///< reference length behind the ego [m]
  double station_ds = 1.0;         ///< corridor station spacing [m]
  int max_adjacent_lanes = 1;      ///< lanes considered on each side of the ego lane
  double w_route_urgency = 10.0;   ///< cost of a lane that leaves the route right away
  double urgency_distance = 80.0;  ///< distance over which route urgency ramps up [m]
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
};

/// Build the reference line and corridor around the ego from a lane map and
/// an (optional) route given as lane ids. The ego lane chain follows route
/// lanes where possible; adjacent lanes become bands whose accessibility comes
/// from the map's lane-change permissions (or from the route passing through
/// them) and whose cost grows as they approach the point where they leave the
/// route.
std::optional<RouteCorridor> buildRouteCorridor(const RoadMap& map, const std::vector<int>& route,
                                                const EgoState& ego, const RouteCorridorConfig& cfg,
                                                int previous_ego_lane = -1);

}  // namespace fiss_planner
