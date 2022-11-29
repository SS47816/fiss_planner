/** fiss_planner.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of the core FISS Planner Algorithms
 */

#ifndef FISS_PLANNER_H_
#define FISS_PLANNER_H_

#include <cmath>
#include <vector>
#include <iostream>
#include <future>
#include <queue>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "frenet.h"
#include "math_utils.h"
#include "spline.h"
#include "vehicle_state.h"
#include "vehicle.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "sat_collision_checker.h"


namespace fiss
{

class FissPlanner
{
 public:
  struct Setting
  {
   public:
    Setting() {};
    virtual ~Setting() {};

    // General Settings
    double tick_t;              // time tick [s]

    // Sampling Parameters
    double center_offset;       // offset from the center of the lane [m]
    int num_width;              // number of road width samples
    double max_t;               // max prediction time [s]
    double min_t;               // min prediction time [s]
    int num_t;                  // number of time samples
    double highest_speed;       // highest target speed [m/s]
    double lowest_speed;        // lowest target speed [m/s]
    int num_speed;              // number of speed samples

    // Hard Constraints
    double max_speed;           // maximum speed [m/s]
    double max_accel;           // maximum acceleration [m/ss]
    double max_decel;           // maximum deceleration [m/ss]
    double max_curvature;       // maximum curvature [rad/m]
    double max_jerk_s;          // maximum longitudinal jerk [m/s^3]
    double max_jerk_d;          // maximum lateral jerk [m/s^3]
    // double steering_angle_rate; // [rad/s]

    // Cost Weights
    double k_jerk;              // jerk cost weight
    double k_time;              // time cost weight
    double k_diff;              // speed and lateral offset cost weight
    double k_lat;               // lateral overall cost weight
    double k_lon;               // longitudinal overall cost weight
    double k_obstacle;          // obstacle cost weight
    double k_heuristic;         // heuristic cost weight

    // Collision Parameters
    double safety_margin_lon;   // lon safety margin [ratio]
    double safety_margin_lat;   // lat safety margin [ratio]
    double safety_margin_soft;  // soft safety margin [ratio]
    double vehicle_width;       // vehicle width [m]
    double vehicle_length;      // vehicle length [m]
  };

  class TestResult
  {
   public:
    int length;
    int count;
    std::vector<int> numbers;
    std::vector<int> numbers_min;
    std::vector<int> numbers_max;
    std::vector<int> total_numbers;
    std::vector<double> time;
    std::vector<double> time_min;
    std::vector<double> time_max;
    std::vector<double> total_time;

    double total_fix_cost, total_dyn_cost;
    int total_dist;
    // std::vector<double> cost_history;
    // std::vector<double> dist_history;

    TestResult();
    TestResult(const int length);
    std::vector<double> updateCount(const std::vector<int> numbers, const std::vector<std::chrono::_V2::system_clock::time_point> timestamps,
                     const double fix_cost, const double dyn_cost, const int dist);
    void printSummary();
  };

  /* --------------------------------- Methods -------------------------------- */

  // Constructors
  FissPlanner();
  FissPlanner(const Setting& settings);

  // Destructor
  virtual ~FissPlanner() {};

  // Public Functions
  void updateSettings(const Setting& settings);
  // Generate reference curve as the frenet s coordinate
  std::pair<Path, Spline2D> generateReferenceCurve(const Lane& lane);

  // Plan for the optimal trajectory
  std::pair<std::vector<FrenetPath>, std::vector<double>> 
  frenetOptimalPlanning(Spline2D& cubic_spline, const FrenetState& frenet_state, const int lane_id,
                        const double left_width, const double right_width, const double current_speed, 
                        const autoware_msgs::DetectedObjectArray& obstacles, 
                        const bool check_collision, const bool use_async, const bool use_heuristic);
  
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> candidate_trajs_; 
  std::vector<FrenetPath> all_trajs_;
  FrenetPath best_traj_, prev_best_traj_;
  // Eigen::Vector3i prev_best_idx_;

 private:
  Setting settings_;
  TestResult test_result_;
  FrenetState start_state_;
  SATCollisionChecker sat_collision_checker_;
  
  std::vector<std::vector<std::vector<FrenetPath>>> sampleEndStates(const int lane_id, const double left_bound, 
                                                                    const double right_bound, const double current_speed, 
                                                                    const bool use_heuristic);
  // Find the best init guess based on end states
  bool findInitGuess(const std::vector<std::vector<std::vector<FrenetPath>>>& trajs, Eigen::Vector3i& idx);
  // Explore trajectories
  bool findNextBest(std::vector<std::vector<std::vector<FrenetPath>>>& trajs, Eigen::Vector3i& idx, int& num_traj);
  Eigen::Vector3d findGradients(std::vector<std::vector<std::vector<FrenetPath>>>& trajs, const Eigen::Vector3i& sizes, const Eigen::Vector3i& idx, int& num_traj);
  Eigen::Vector3i findDirection(const Eigen::Vector3i& sizes, const Eigen::Vector3i& idx);
  // Generate this candidate trajectory and compute the real(final) cost
  double getTrajAndRealCost(std::vector<std::vector<std::vector<FrenetPath>>>& trajs, const Eigen::Vector3i& idx, int& num_traj);

  // Convert paths from frenet frame to gobal map frame
  void convertToGlobalFrame(FrenetPath& traj, Spline2D& cubic_spline);
  // Check for vehicle kinematic constraints
  bool checkConstraints(FrenetPath& traj);

  // Check for collisions and calculate obstacle cost
  std::vector<Path> predictTrajectories(const autoware_msgs::DetectedObjectArray& obstacles);
  std::vector<Path> getPredictedTrajectories(const autoware_msgs::DetectedObjectArray& obstacles);
  bool checkCollisions(FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                       const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async, int& num_checks);
  std::pair<bool, int> checkTrajCollision(const FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                                          const autoware_msgs::DetectedObjectArray& obstacles, const double margin_lon, const double margin_lat);
};

}  // namespace fiss

#endif  // FISS_PLANNER_H_