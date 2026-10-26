// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "fiss_planner/collision/box2d.hpp"

namespace fiss_planner {

enum class LineType { kUnknown, kDashed, kSolid, kRoadEdge, kVirtual };

/// A lane (lanelet) of a lane-level road map. Polylines run in driving
/// direction. Adapters (Lanelet2, CommonRoad, CARLA) fill these in.
struct MapLane {
  int id = -1;
  std::vector<Vec2> centerline;
  std::vector<Vec2> left;   ///< left boundary
  std::vector<Vec2> right;  ///< right boundary
  LineType left_type = LineType::kUnknown;
  LineType right_type = LineType::kUnknown;
  int left_neighbor = -1;  ///< adjacent lane in the same direction, or -1
  int right_neighbor = -1;
  bool can_change_left = false;  ///< traffic rules allow changing into left_neighbor
  bool can_change_right = false;
  std::vector<int> successors;
  std::vector<int> predecessors;                                 ///< filled by RoadMap::finalize() if empty
  double speed_limit = std::numeric_limits<double>::infinity();  ///< [m/s]
  bool junction = false;  ///< inside an intersection (e.g. a lane connector)

  double length() const;
};

/// A stop line on a lane at arc length `s` along its centerline. Active stop
/// lines (red light, stop sign) become virtual obstacles for the planner. A
/// yellow light is active but may be passed if stopping is uncomfortable, and
/// the vehicle may go on from a stop sign once it has stopped there (both
/// decided by the behaviour layer, see BehaviorConfig).
struct StopLine {
  int id = -1;
  int lane_id = -1;
  double s = 0.0;
  bool active = true;
  bool yellow = false;
  bool stop_sign = false;
};

class RoadMap {
 public:
  void addLane(MapLane lane);
  void addStopLine(const StopLine& stop_line) { stop_lines_.push_back(stop_line); }
  bool setStopLineActive(int id, bool active, bool yellow = false);
  /// Derive predecessors from successors and validate neighbour references.
  void finalize();

  const MapLane* lane(int id) const;
  const std::unordered_map<int, MapLane>& lanes() const { return lanes_; }
  const std::vector<StopLine>& stopLines() const { return stop_lines_; }

  /// Lane containing (x, y) with a compatible heading. Lanes in `preferred`
  /// win ties (e.g. the route or the previous ego lane). If no lane contains
  /// the point, the nearest centerline within `max_dist` is used. -1 if none.
  int locate(double x, double y, double yaw, const std::unordered_set<int>* preferred = nullptr,
             double max_dist = 5.0) const;

 private:
  std::unordered_map<int, MapLane> lanes_;
  std::vector<StopLine> stop_lines_;
};

/// Arc-length position and lateral distance of a point on a polyline.
struct PolylineProjection {
  double s = 0.0;
  double distance = std::numeric_limits<double>::infinity();
  double heading = 0.0;  ///< heading of the segment at the foot point
};
PolylineProjection projectOnPolyline(const std::vector<Vec2>& line, double x, double y);
double polylineLength(const std::vector<Vec2>& line);
/// Point and heading at arc length s along a polyline (clamped).
Vec2 polylinePoint(const std::vector<Vec2>& line, double s, double* heading = nullptr);

}  // namespace fiss_planner
