// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// Lanelet2 -> fiss_planner::RoadMap conversion, shared by the ROS 1 and
// ROS 2 wrappers (no ROS dependency).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "fiss_planner/map/road_map.hpp"

namespace fiss_planner::ros_common {

struct Lanelet2Options {
  std::string path;               ///< .osm file
  std::string projector = "utm";  ///< "utm" or "local" (local Cartesian at the origin)
  double origin_lat = 49.0;
  double origin_lon = 8.4;
  std::string location = "de";  ///< traffic rules location (lanelet::Locations)
};

/// A loaded Lanelet2 map with its routing graph, converted to a RoadMap.
class Lanelet2Map {
 public:
  explicit Lanelet2Map(const Lanelet2Options& options);
  ~Lanelet2Map();

  const RoadMap& roadMap() const { return road_map_; }
  /// Shortest legal route (lane ids, lane changes included) between the
  /// lanelets nearest to two positions; empty if none exists.
  std::vector<int> route(double from_x, double from_y, double to_x, double to_y) const;
  size_t numLanes() const { return road_map_.lanes().size(); }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  RoadMap road_map_;
};

}  // namespace fiss_planner::ros_common
