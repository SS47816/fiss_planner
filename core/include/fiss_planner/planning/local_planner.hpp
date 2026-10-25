// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "fiss_planner/map/road_map.hpp"
#include "fiss_planner/map/route_corridor.hpp"
#include "fiss_planner/planning/planner.hpp"
#include "fiss_planner/smoothing/smoother.hpp"

namespace fiss_planner {

/// Replanning from the previous trajectory (Apollo-style stitching).
struct StitchingConfig {
  bool enabled = true;
  double latency = 0.0;             ///< expected compute + actuation delay [s]
  double max_position_error = 1.0;  ///< replan from the measured state beyond this [m]
  double max_heading_error = 0.3;   ///< ... or beyond this heading error [rad]
};

/// Lane layout used when only a reference path (no map) is given.
struct PathModeConfig {
  double lane_width = 3.5;
  int num_left_lanes = 1;
  int num_right_lanes = 1;
  bool allow_left = true;
  bool allow_right = true;
};

struct LocalPlannerConfig {
  PlannerConfig planner;
  RouteCorridorConfig route;
  StitchingConfig stitching;
  PathModeConfig path;
  SmootherConfig smoother;       ///< optional QP refinement of the chosen trajectory
  double desired_speed = 13.9;   ///< [m/s] used when step() gets no speed
  double emergency_decel = 0.0;  ///< [m/s^2] fail-safe braking; 0 = vehicle max_decel
  /// When no candidate is valid and the previous plan is no longer collision
  /// free, the planner retries without the safety envelope and then checking
  /// collisions only this far ahead [s] before braking (long-horizon
  /// predictions are the least reliable). 0 = no short-horizon retry.
  double degraded_horizon = 2.5;
  /// After an emergency stop, the next plan starts from the measured state with
  /// at most this deceleration [m/s^2], so the vehicle resumes once the way is
  /// clear instead of inheriting full braking from the stop profile.
  double recovery_decel = 2.0;
  bool use_speed_limit = true;            ///< cap at the ego lane's speed limit
  bool use_curvature_speed_limit = true;  ///< cap at sqrt(max_lat_accel / kappa) ahead
};

struct LocalPlanOutput {
  bool success = false;    ///< a new trajectory was planned this cycle
  bool fallback = false;   ///< the remainder of the previous trajectory is returned instead
  bool emergency = false;  ///< no safe plan: maximum-deceleration stop in the current lane
  bool degraded = false;   ///< planned in a fallback retry (no envelope, or the shorter collision horizon)
  bool stitched = false;   ///< planned from the previous trajectory, not the measured state
  bool smoothed = false;   ///< the QP smoother refined the sampled trajectory
  Trajectory trajectory;   ///< t relative to the step() time
  PlanStats stats;
  std::vector<Trajectory> candidates;
  std::shared_ptr<const ReferenceLine> reference;
  Corridor corridor;
  FrenetState start;
  double target_speed = 0.0;
  int ego_lane = -1;
  std::vector<Obstacle> virtual_obstacles;  ///< e.g. active stop lines
};

/// One-call planning pipeline used by the ROS nodes and the benchmarks:
/// reference/corridor construction (from a lane map or a plain path), start
/// state selection with stitching, target speed policy and the planner.
/// step() runs on one thread; request*() may be called from any thread and
/// takes effect at the start of the next cycle.
class LocalPlanner {
 public:
  LocalPlanner(const LocalPlannerConfig& config, const VehicleParams& vehicle);

  void setMap(std::shared_ptr<const RoadMap> map, std::vector<int> route = {});
  void setReferencePath(const std::vector<double>& x, const std::vector<double>& y);
  void requestConfig(const LocalPlannerConfig& config);
  void requestPlannerType(PlannerType type);
  /// Change a stop line's state (e.g. traffic light) without resetting the planner.
  void setStopLineActive(int stop_line_id, bool active);
  void reset();

  /// Plan at time `t_now` [s]. Obstacle predictions are relative to t_now.
  /// A non-positive `desired_speed` falls back to the configured one.
  LocalPlanOutput step(double t_now, const EgoState& ego, const std::vector<Obstacle>& obstacles,
                       double desired_speed = -1.0);

  const LocalPlannerConfig& config() const { return config_; }
  const Planner& planner() const { return *planner_; }

 private:
  void applyPendingConfig();
  double targetSpeed(const ReferenceLine& ref, double s, double v, double desired, double lane_limit) const;
  Trajectory emergencyStop(const FrenetState& start, const ReferenceLine& ref) const;

  LocalPlannerConfig config_;
  VehicleParams vehicle_;
  std::unique_ptr<Planner> planner_;

  std::shared_ptr<RoadMap> map_;  ///< private copy (stop line states are mutable)
  std::vector<int> route_;
  std::shared_ptr<const ReferenceLine> path_ref_;
  Corridor path_corridor_;

  std::optional<Trajectory> previous_;  ///< last output, t relative to previous_t0_
  bool previous_emergency_ = false;     ///< previous_ is an emergency stop
  double previous_t0_ = 0.0;
  int previous_ego_lane_ = -1;

  std::mutex pending_mutex_;
  std::optional<LocalPlannerConfig> pending_config_;
  std::vector<std::pair<int, bool>> pending_stop_lines_;
};

/// Cartesian state of a trajectory at time t (linear interpolation, clamped).
EgoState sampleTrajectory(const Trajectory& traj, double t);

}  // namespace fiss_planner
