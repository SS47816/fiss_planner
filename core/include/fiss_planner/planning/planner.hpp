// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "fiss_planner/planning/components.hpp"

namespace fiss_planner {

struct PlanningInput {
  FrenetState start;                                 ///< start state in the reference frame
  const ReferenceLine* reference = nullptr;          ///< required
  const Corridor* corridor = nullptr;                ///< required (see Corridor::uniform)
  const std::vector<Obstacle>* obstacles = nullptr;  ///< optional
  double target_speed = 0.0;                         ///< desired speed [m/s]
  /// Check collisions and the safety envelope only up to this time [s] (e.g.
  /// a degraded retry when long-horizon predictions block every candidate).
  double collision_horizon = std::numeric_limits<double>::infinity();
  /// Enforce the safety envelope (fallback retries may relax it; collisions are always checked).
  bool safety_envelope = true;
};

struct PlanStats {
  int num_generated = 0;         ///< trajectories generated in the Frenet frame
  int num_validated = 0;         ///< trajectories converted and checked
  int num_collision_checks = 0;  ///< ego/obstacle box-pair tests
  int num_iterations = 0;        ///< outer search iterations (FISS/FISS+)
  std::array<int, static_cast<int>(Rejection::kCount)> rejections{};
  double time_ms = 0.0;  ///< wall-clock planning time
  /// Obstacle that rejected the cheapest candidate lost to a collision or the
  /// safety envelope (-1 if none), when, and that candidate's cost.
  int blocking_id = -1;
  double blocking_t = 0.0;
  double blocking_cost = std::numeric_limits<double>::infinity();
  LeadGap lead;  ///< closest obstacle ahead in the ego's path at the start
};

struct PlanResult {
  bool success = false;
  Trajectory trajectory;
  PlanStats stats;
  std::vector<Trajectory> candidates;  ///< filled if PlannerConfig::keep_candidates
};

/// Common base of the sampling-based planners. All variants share sampling,
/// trajectory generation, costs and validation and differ only in how they
/// search the end-state grid.
class Planner {
 public:
  Planner(const PlannerConfig& config, const VehicleParams& vehicle);
  virtual ~Planner() = default;

  PlanResult plan(const PlanningInput& input);

  PlannerType type() const { return config_.type; }
  const PlannerConfig& config() const { return config_; }
  const VehicleParams& vehicle() const { return vehicle_; }
  /// Replace the configuration (takes effect on the next plan() call).
  virtual void setConfig(const PlannerConfig& config);
  /// Forget the previous solution (disables the history heuristic once).
  void reset() { prev_best_.reset(); }

 protected:
  using Clock = std::chrono::steady_clock;

  /// Per-call state handed to the search strategies.
  struct Context {
    const PlanningInput& input;
    const EndStateGrid& grid;
    Clock::time_point start_time;
    PlanStats& stats;
    const FrenetObstacles& obstacles;       ///< obstacles near the corridor, in the Frenet frame
    const std::vector<char>& rear_in_lane;  ///< obstacles lane-keeping candidates may ignore
    const std::vector<std::pair<int, double>>& lane_speeds;  ///< (lane offset, target speed)

    /// Target speed of candidates ending in the given lane (car following).
    double targetSpeed(int lane) const {
      for (const auto& [offset, speed] : lane_speeds) {
        if (offset == lane) return speed;
      }
      return input.target_speed;
    }
  };

  /// Lane information of a sample with the lane's car-following speed deficit
  /// (relative to the desired speed) added to its lane cost, so that slower
  /// lanes stay less attractive than free ones; returns the lane's target speed.
  double laneTerms(const Context& ctx, LateralSample& lat) const;

  /// Search the grid; on success write the chosen trajectory to `best`.
  virtual bool search(Context& ctx, Trajectory& best, std::vector<Trajectory>* candidates) = 0;

  /// Generate a trajectory (Frenet + cost) for a grid sample.
  Trajectory generate(Context& ctx, int i, int j, int k) const;
  /// Generate a trajectory (Frenet + cost) for an arbitrary end state.
  Trajectory generate(Context& ctx, const EndState& end) const;
  /// Estimated cost of an unexplored grid sample, including the history heuristic.
  double estimate(const Context& ctx, int i, int j, int k) const;
  /// Convert to Cartesian and run constraint and collision checks. Thread-safe;
  /// statistics are accumulated into `stats`.
  bool validate(const Context& ctx, Trajectory& traj, PlanStats& stats) const;
  /// Validate several trajectories (in parallel if configured); returns flags.
  std::vector<char> validateAll(const Context& ctx, std::vector<Trajectory>& trajs, PlanStats& stats) const;
  double elapsedSeconds(const Context& ctx) const;

  PlannerConfig config_;
  VehicleParams vehicle_;
  CostEvaluator cost_;
  ConstraintValidator constraints_;
  CollisionChecker collisions_;
  GapEvaluator gaps_;
  std::optional<EndState> prev_best_;
};

std::unique_ptr<Planner> makePlanner(const PlannerConfig& config, const VehicleParams& vehicle);

}  // namespace fiss_planner
