// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <string>

#include "fiss_planner/common/types.hpp"

namespace fiss_planner {

enum class PlannerType {
  kFop,       ///< Werling et al. 2010: generate, convert and check every candidate
  kFopPlus,   ///< generate all, validate lazily in cost order (Apollo-lattice style)
  kFiss,      ///< Sun et al., RA-L 2022: fast iterative search and sampling
  kFissPlus,  ///< Sun et al., IROS 2023: FISS + neighbour exploration + refinement
};

std::string toString(PlannerType type);
/// Parses "fop", "fop+", "fop_plus", "fiss", "fiss+", "fiss_plus" (case-insensitive).
PlannerType plannerTypeFromString(const std::string& name);

struct SamplingConfig {
  int num_width = 5;          ///< lateral samples per lane
  int num_speed = 5;          ///< target speed samples
  int num_t = 5;              ///< horizon samples
  double min_t = 3.0;         ///< [s]
  double max_t = 5.0;         ///< [s]
  double dt = 0.1;            ///< trajectory time resolution [s]
  double min_speed = 0.0;     ///< lowest sampled target speed [m/s]
  double lane_margin = 0.15;  ///< clearance kept to lane edges when sampling [m]
};

/// Weights of the trajectory cost. Speed, lateral and time terms are evaluated
/// at the end state; acceleration, jerk and proximity terms are per-sample
/// means, so the cost does not depend on the trajectory resolution.
///
/// The proximity terms penalize, per sample, the closest obstacle ahead in the
/// ego's path (gap below desired_gap + desired_time_headway * v, time to
/// collision below desired_ttc) and the closest obstacle alongside (lateral gap
/// below desired_clearance), each as (1 - actual / desired)^2 when positive.
/// Obstacles behind the ego are ignored.
///
/// Car following: candidates ending in a lane with a slower vehicle ahead are
/// scored against the speed that closes the gap to the desired one within
/// `follow_time`, instead of the desired speed, so the speed term does not pull
/// the ego into the vehicle ahead. The lane's deficit to the desired speed is
/// charged as a lane cost (w_speed * deficit^2), so free lanes stay attractive.
struct CostConfig {
  double w_time = 1.0;         ///< (max_t - T): prefer longer horizons
  double w_speed = 1.0;        ///< (s_d(T) - v_target)^2
  double w_accel = 0.1;        ///< mean(s_dd^2 + d_dd^2)
  double w_jerk = 0.1;         ///< mean(s_ddd^2 + d_ddd^2)
  double w_lateral = 2.0;      ///< (d(T) - lane center)^2
  double w_lane_change = 5.0;  ///< per lane changed
  double w_heuristic = 1.0;    ///< FISS history heuristic weight

  double w_headway = 10.0;            ///< gap to the obstacle ahead
  double desired_gap = 3.0;           ///< [m] at standstill
  double desired_time_headway = 1.5;  ///< [s]
  double w_ttc = 5.0;                 ///< time to collision with the obstacle ahead
  double desired_ttc = 4.0;           ///< [s]
  double w_clearance = 5.0;           ///< lateral gap to obstacles alongside
  double desired_clearance = 1.0;     ///< [m]
  double follow_time = 3.0;           ///< [s]; 0 = always score against the desired speed
};

struct ConstraintConfig {
  bool check_curvature = true;
  bool check_lat_accel = true;
  bool check_road_bounds = true;
  double curvature_check_min_speed = 1.0;  ///< curvature is ill-defined near standstill [m/s]
};

struct CollisionConfig {
  double margin_lon = 0.5;  ///< added to each end of the ego box [m]
  double margin_lat = 0.3;  ///< added to each side of the ego box [m]
  int stride = 1;           ///< check every n-th trajectory point
  /// Lane-keeping candidates ignore obstacles that start behind the ego in its
  /// own lane: with reactive traffic, the follower keeps its distance. Lane
  /// changes still check them. Leave off for non-reactive (replayed) traffic.
  bool ignore_rear_in_lane = false;
};

/// Hard safety envelope ("virtual bumper") towards obstacles ahead in the
/// ego's path: a candidate must keep a gap of at least
/// min_gap + min_time_headway * v and a time to collision of at least min_ttc.
/// An obstacle that is in the path from the start, or that cuts into the ego's
/// lane later, is checked only from `reaction_time` after it appears, and may
/// not be approached closer than it appeared; so after a close cut-in, braking
/// candidates stay feasible. Lane changes towards an obstacle get no grace,
/// while samples moving laterally away from an obstacle (pulling out to
/// overtake) are held to its time to collision only, not to the gap floor.
///
/// Towards crossing and oncoming vehicles (heading more than 45 deg from the
/// ego's), the envelope is a post-encroachment time: a candidate may not occupy
/// space such a vehicle occupies within crossing_time_gap before or after, so
/// turns across traffic keep clear of the vehicles they cross. Like the gap and
/// TTC rules, it is relaxed when no candidate satisfies it (see LocalPlanner).
struct SafetyConfig {
  bool enabled = true;
  double min_gap = 2.0;             ///< [m]
  double min_time_headway = 0.3;    ///< [s] (the comfortable headway is a cost term)
  double min_ttc = 1.5;             ///< [s]
  double reaction_time = 0.5;       ///< [s]
  double crossing_time_gap = 0.75;  ///< [s] post-encroachment time; 0 = off
};

struct FissPlusConfig {
  bool refine = true;
  int max_refine_iters = 3;
  double decay = 0.5;        ///< step-size decay per refinement iteration
  double time_budget = 0.0;  ///< wall-clock budget for the whole cycle [s]; 0 = unlimited
};

struct PlannerConfig {
  PlannerType type = PlannerType::kFissPlus;
  SamplingConfig sampling;
  CostConfig cost;
  ConstraintConfig constraints;
  CollisionConfig collision;
  SafetyConfig safety;
  FissPlusConfig fiss_plus;
  int num_threads = 1;           ///< worker threads for FOP/FOP+ validation
  bool keep_candidates = false;  ///< return generated candidates (for visualization)

  /// Throws std::invalid_argument describing the first invalid field.
  void validate() const;
};

/// Load a planner (and optionally vehicle) configuration from YAML. Missing
/// keys keep their defaults; unknown keys are rejected.
PlannerConfig loadPlannerConfig(const std::string& yaml_path, VehicleParams* vehicle = nullptr);
PlannerConfig parsePlannerConfig(const std::string& yaml_text, VehicleParams* vehicle = nullptr);

}  // namespace fiss_planner
