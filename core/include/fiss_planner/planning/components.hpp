// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// Building blocks shared by every sampling planner: the end-state grid, the
// trajectory generator, the cost evaluator and the validators.
#pragma once

#include <array>
#include <limits>
#include <optional>
#include <vector>

#include "fiss_planner/common/types.hpp"
#include "fiss_planner/geometry/reference_line.hpp"
#include "fiss_planner/planning/config.hpp"
#include "fiss_planner/planning/corridor.hpp"
#include "fiss_planner/planning/frenet_obstacles.hpp"
#include "fiss_planner/planning/trajectory.hpp"
#include "fiss_planner/prediction/obstacle.hpp"

namespace fiss_planner {

/// Lateral sample of the grid: an offset together with the lane it lies in.
struct LateralSample {
  double d = 0.0;
  int lane = 0;              ///< lane offset (0 = ego lane)
  double lane_center = 0.0;  ///< lateral offset of that lane's center
  double lane_cost = 0.0;    ///< extra cost of ending in that lane
};

/// Regular grid of end states over (lateral offset, target speed, horizon).
/// The lateral axis concatenates the samples of every allowed lane, ordered
/// right to left, so neighbouring indices are neighbouring offsets.
class EndStateGrid {
 public:
  EndStateGrid(const SamplingConfig& cfg, const Corridor& corridor, const VehicleParams& vehicle,
               double s_lookahead, double target_speed);

  std::array<int, 3> dims() const {
    return {static_cast<int>(lateral_.size()), static_cast<int>(speeds_.size()),
            static_cast<int>(times_.size())};
  }
  int numSamples() const { return dims()[0] * dims()[1] * dims()[2]; }
  int flat(int i, int j, int k) const { return (i * dims()[1] + j) * dims()[2] + k; }
  std::array<int, 3> unflat(int f) const;

  const std::vector<LateralSample>& lateral() const { return lateral_; }
  const std::vector<double>& speeds() const { return speeds_; }
  const std::vector<double>& times() const { return times_; }
  EndState endState(int i, int j, int k) const { return {lateral_[i].d, speeds_[j], times_[k]}; }

  /// Bounds and nominal step of the continuous end-state space (d, v, T).
  std::array<double, 3> lower() const { return lower_; }
  std::array<double, 3> upper() const { return upper_; }
  std::array<double, 3> resolution() const { return resolution_; }

  /// Lane information for an arbitrary (off-grid) lateral offset.
  LateralSample lateralInfo(double d) const;

 private:
  std::vector<LateralSample> lateral_;
  std::vector<double> speeds_, times_;
  std::array<double, 3> lower_{}, upper_{}, resolution_{};
  std::vector<LaneBand> lanes_;  ///< allowed lanes evaluated at the lookahead station
  std::vector<double> lane_left_, lane_right_;
};

/// Fill the Frenet arrays of `traj` from a quintic lateral and a quartic
/// longitudinal polynomial between `start` and `end` (t = 0, dt, ..., T).
/// Below a start speed of `low_speed` the lateral quintic is a function of
/// the distance travelled, d(s - s0), reaching end.d where the longitudinal
/// motion ends; a plan that (almost) stands keeps its lateral offset.
void generateFrenet(const FrenetState& start, const EndState& end, double dt, Trajectory& traj,
                    double low_speed = 0.0);

/// Fill the Cartesian arrays of `traj`. Returns false on non-finite output.
bool convertToCartesian(const ReferenceLine& ref, Trajectory& traj);

class CostEvaluator {
 public:
  CostEvaluator(const CostConfig& cost, const SamplingConfig& sampling, const VehicleParams& vehicle = {},
                double margin_lat = 0.3)
      : w_(cost),
        max_t_(sampling.max_t),
        vehicle_(vehicle),
        gaps_(cost, SafetyConfig{}, vehicle, margin_lat) {}

  /// Cost of the terminal terms only. Since the remaining (comfort) terms are
  /// non-negative this is a lower bound of the full cost.
  double estimate(const EndState& end, const LateralSample& lat, double target_speed) const;
  /// Full cost of a generated trajectory (Frenet arrays only), following
  /// Werling et al.: terminal speed/offset/horizon costs plus mean squared
  /// acceleration and jerk, plus the proximity terms if obstacles are given
  /// and the road-edge term if the corridor is.
  CostTerms evaluate(const Trajectory& traj, const LateralSample& lat, double target_speed,
                     const FrenetObstacles* obstacles = nullptr, const Corridor* corridor = nullptr) const;

 private:
  CostConfig w_;
  double max_t_;
  VehicleParams vehicle_;
  GapEvaluator gaps_;
};

/// Distance [m] the vehicle footprint extends beyond the corridor's road
/// bounds at each sample of a trajectory (0 inside), from its lateral offset
/// and its heading relative to the reference line. The heading follows the
/// Frenet velocity above 0.5 m/s and is held below (from rest: along the
/// reference), where the velocity no longer tells the vehicle's orientation.
std::vector<double> roadEdgeExcess(const Trajectory& traj, const Corridor& corridor,
                                   const VehicleParams& vehicle);

/// Kinematic and corridor constraints on a Cartesian trajectory.
class ConstraintValidator {
 public:
  ConstraintValidator(const ConstraintConfig& cfg, const VehicleParams& vehicle)
      : cfg_(cfg), vehicle_(vehicle) {}
  Rejection check(const Trajectory& traj, const Corridor* corridor) const;

 private:
  ConstraintConfig cfg_;
  VehicleParams vehicle_;
};

/// Checks the inflated ego footprint against predicted obstacle footprints
/// at every (strided) trajectory time.
class CollisionChecker {
 public:
  CollisionChecker(const CollisionConfig& cfg, const VehicleParams& vehicle) : cfg_(cfg), vehicle_(vehicle) {}
  /// Returns true if collision free; `num_checks` counts box-pair tests. On a
  /// collision, `hit_id` / `hit_t` (if given) receive the first obstacle hit
  /// and when. Only samples with t <= max_t are checked, and obstacles flagged
  /// in `skip` (indexed like `obstacles`) are ignored.
  bool isCollisionFree(const Trajectory& traj, const std::vector<Obstacle>& obstacles, int& num_checks,
                       int* hit_id = nullptr, double* hit_t = nullptr,
                       double max_t = std::numeric_limits<double>::infinity(),
                       const std::vector<char>* skip = nullptr) const;
  /// Post-encroachment time to crossing and oncoming vehicles (heading more
  /// than 45 deg from the ego's): false if the ego occupies space such a
  /// vehicle occupies within `gap` [s] before or after. Uses the collision
  /// margins; samples with t > max_t are not checked. `gap` <= 0 = always true.
  bool keepsTimeGap(const Trajectory& traj, const std::vector<Obstacle>& obstacles, double gap,
                    int& num_checks, int* hit_id = nullptr, double* hit_t = nullptr,
                    double max_t = std::numeric_limits<double>::infinity()) const;

 private:
  CollisionConfig cfg_;
  VehicleParams vehicle_;
};

}  // namespace fiss_planner
