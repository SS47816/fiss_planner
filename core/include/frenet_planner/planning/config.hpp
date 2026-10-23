// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <string>

#include "frenet_planner/common/types.hpp"

namespace frenet_planner {

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
/// at the end state; acceleration and jerk are per-sample means, so the cost
/// does not depend on the trajectory resolution.
struct CostConfig {
  double w_time = 1.0;         ///< (max_t - T): prefer longer horizons
  double w_speed = 1.0;        ///< (s_d(T) - v_target)^2
  double w_accel = 0.1;        ///< mean(s_dd^2 + d_dd^2)
  double w_jerk = 0.1;         ///< mean(s_ddd^2 + d_ddd^2)
  double w_lateral = 2.0;      ///< (d(T) - lane center)^2
  double w_lane_change = 5.0;  ///< per lane changed
  double w_heuristic = 1.0;    ///< FISS history heuristic weight
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

}  // namespace frenet_planner
