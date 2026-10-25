// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <string>
#include <vector>

#include "fiss_planner/map/road_map.hpp"
#include "fiss_planner/prediction/obstacle.hpp"

namespace fiss_planner {

struct LaneFollowingConfig {
  double max_heading_diff = 0.52;      ///< beyond this the obstacle is not following its lane [rad]
  double max_lane_distance = 2.5;      ///< max distance to the lane centerline [m]
  double min_speed = 0.5;              ///< slower obstacles keep a constant-velocity prediction [m/s]
  double lateral_time_constant = 1.5;  ///< decay of the offset to the centerline [s]
};

/// Predict an obstacle that follows its lane: it keeps its speed along the
/// lane centerline (continuing into the straightest successor at forks) while
/// its lateral offset decays to the centerline. Falls back to a constant
/// velocity prediction when the obstacle is (nearly) stopped, off the map, or
/// not heading along its lane.
Obstacle predictLaneFollowing(const RoadMap& map, int id, double x, double y, double yaw, double v,
                              double length, double width, double horizon, double dt,
                              const LaneFollowingConfig& cfg = {});

enum class PredictionMode {
  kAsGiven,           ///< use the obstacles' predictions unchanged
  kConstantVelocity,  ///< extend single-state obstacles at constant velocity
  kLaneFollowing,     ///< extend single-state obstacles along their lane (constant velocity off the map)
};
/// Parses "as_given", "constant_velocity" and "lane_following".
PredictionMode predictionModeFromString(const std::string& name);
std::string toString(PredictionMode mode);

/// Give every non-static obstacle that carries only its current state (as
/// tracked objects from perception usually do) a prediction over `horizon`.
/// Otherwise such obstacles would be held in place.
void completePredictions(std::vector<Obstacle>& obstacles, PredictionMode mode, const RoadMap* map,
                         double horizon, double dt, const LaneFollowingConfig& cfg = {});

}  // namespace fiss_planner
