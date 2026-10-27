// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "fiss_planner/collision/box2d.hpp"
#include "fiss_planner/common/types.hpp"
#include "fiss_planner/planning/trajectory.hpp"

namespace fiss_planner {

/// Free-space planning (parking in or out, leaving the lanes): a Hybrid A*
/// search over vehicle poses on an occupancy grid.
struct FreeSpaceConfig {
  bool enabled = false;
  double grid_size = 50.0;    ///< [m] side of the square grid around the ego and the goal
  double resolution = 0.1;    ///< [m] grid cell size
  int yaw_bins = 72;          ///< heading resolution of the search (360 deg / bins)
  double step = 1.0;          ///< [m] arc length of a motion primitive
  int steer_samples = 5;      ///< steering angles per direction, from full left to full right
  bool reverse = true;        ///< allow driving backwards
  double reverse_cost = 2.0;  ///< multiplier on the distance driven backwards
  double switch_cost = 5.0;   ///< [m] per change of direction
  double steer_cost = 0.5;    ///< [m] per primitive at full steering
  double margin = 0.2;        ///< [m] clearance kept to obstacles and the edge of the free area
  int max_expansions = 40000;
  double speed = 1.5;        ///< [m/s] along the free-space path
  double accel = 1.0;        ///< [m/s^2] speeding up and slowing down
  double goal_ahead = 12.0;  ///< [m] the goal is on the target lane this far ahead of the ego
};

/// Pose of the vehicle center; `reverse` marks poses reached driving backwards.
struct FreeSpacePose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool reverse = false;
};

/// Square occupancy grid. Cells start occupied; markFree() frees polygons
/// (lanes, parking areas), then markOccupied() blocks obstacle boxes. Cell
/// edges lie on multiples of the resolution, so grids built around nearby
/// centers agree where they overlap.
class OccupancyGrid {
 public:
  OccupancyGrid(double cx, double cy, double size, double resolution);
  void markFree(const std::vector<Vec2>& polygon);
  void markOccupied(const Box2d& box, double inflation = 0.0);
  /// Distance [m] from (x, y) to the nearest occupied cell (0 inside one or
  /// outside the grid); valid after computeClearance().
  double clearance(double x, double y) const;
  void computeClearance();
  bool inside(double x, double y) const;

  int size() const { return n_; }
  double resolution() const { return res_; }
  bool occupiedCell(int i, int j) const { return cells_[index(i, j)] != 0; }
  void cell(double x, double y, int& i, int& j) const;
  Vec2 center(int i, int j) const;

 private:
  int index(int i, int j) const { return j * n_ + i; }
  double x0_, y0_, res_;
  int n_;
  std::vector<std::uint8_t> cells_;
  std::vector<float> clearance_;
};

/// Hybrid A* from `start` to `goal` (vehicle center poses) for a kinematic
/// bicycle at its minimum turning radius, with a Dubins shot to the goal once
/// it is near and the way is free. The path keeps config.margin from occupied
/// cells, or as much as the start pose has if less. Returns the path (about
/// every 0.2 m), or nullopt if the search fails.
std::optional<std::vector<FreeSpacePose>> planHybridAStar(const OccupancyGrid& grid,
                                                          const FreeSpacePose& start,
                                                          const FreeSpacePose& goal,
                                                          const VehicleParams& vehicle,
                                                          const FreeSpaceConfig& config);

/// Whether the vehicle (center pose) keeps `margin` from occupied cells.
bool freeSpaceCollisionFree(const OccupancyGrid& grid, const FreeSpacePose& pose,
                            const VehicleParams& vehicle, double margin);

/// Time-parameterized trajectory along a free-space path: each stretch
/// between changes of direction starts and ends at rest (the first starts at
/// `v0`), at most `speed`; reverse stretches have negative speed.
Trajectory freeSpaceTrajectory(const std::vector<FreeSpacePose>& path, double v0,
                               const FreeSpaceConfig& config, double dt);

/// Shortest forward path of bounded curvature between two poses (Dubins),
/// sampled every `ds`; empty if none.
std::vector<FreeSpacePose> dubinsPath(const FreeSpacePose& from, const FreeSpacePose& to, double radius,
                                      double ds);

}  // namespace fiss_planner
