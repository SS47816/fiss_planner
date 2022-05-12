/** frenet_optimal_trajectory_planner.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of Optimal trajectory planning in Frenet Coordinate Algorithm
 * Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
 */

#ifndef FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_
#define FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_

#include <cmath>
#include <vector>
#include <iostream>
#include <future>
#include <queue>

#include "setting.h"
#include "frenet.h"
#include "math_utils.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "spline.h"
#include "vehicle_state.h"
#include "vehicle.h"

// #include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "sat_collision_checker.h"

// #define TRUE_SIZE_LENGTH 3
// #define TRUE_SIZE_MARGIN 0.3

namespace fiss
{

class FrenetOptimalTrajectoryPlanner
{
 public:
  class TestResult
  {
   public:
    int count;
    std::vector<int> numbers;
    std::vector<int> total_numbers;
    std::vector<double> time;
    std::vector<double> total_time;

    double total_fix_cost, total_dyn_cost;
    double total_dist;
    // std::vector<double> cost_history;
    // std::vector<double> dist_history;

    TestResult();
    void updateCount(const std::vector<int> numbers, const std::vector<std::chrono::_V2::system_clock::time_point> timestamps,
                     const double fix_cost, const double dyn_cost, const double dist);
    void printSummary();
  };

  /* --------------------------------- Methods -------------------------------- */

  // Constructors
  FrenetOptimalTrajectoryPlanner();
  FrenetOptimalTrajectoryPlanner(Setting& settings);

  // Destructor
  virtual ~FrenetOptimalTrajectoryPlanner(){};

  void updateSettings(Setting& settings);

  /* Public Functions */
  // Generate reference curve as the frenet s coordinate
  std::pair<Path, Spline2D> generateReferenceCurve(const Lane& lane);

  // Plan for the optimal trajectory
  std::vector<FrenetPath> frenetOptimalPlanning(Spline2D& cubic_spline, const FrenetState& frenet_state, const int lane_id,
                                                     const double left_width, const double right_width, const double current_speed, 
                                                     const autoware_msgs::DetectedObjectArray& obstacles, const bool check_collision, const bool use_async);
  
  std::shared_ptr<std::vector<FrenetPath>> all_trajs_;
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> candidate_trajs_;
  FrenetPath best_traj_, prev_best_traj_;
  Eigen::Vector3i prev_best_idx_;

private:
  Setting settings_;
  TestResult test_result_;
  SATCollisionChecker sat_collision_checker_;

  // Sample candidate trajectories
  std::vector<FrenetPath> generateFrenetPaths(const FrenetState& frenet_state, const int lane_id,
                                                   const double left_bound, const double right_bound, const double current_speed);

  // Convert paths from frenet frame to gobal map frame
  int calculateGlobalPaths(std::vector<FrenetPath>& frenet_traj_list, Spline2D& cubic_spline);
  // Compute costs for candidate trajectories
  int computeCosts(std::vector<FrenetPath>& frenet_trajs, const double curr_speed);

  // Check for vehicle kinematic constraints
  bool checkConstraints(FrenetPath& traj);
  // Check for collisions and calculate obstacle cost
  std::vector<Path> predictTrajectories(const autoware_msgs::DetectedObjectArray& obstacles);
  bool checkCollisions(FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                       const autoware_msgs::DetectedObjectArray& obstacles, const bool use_async, int& num_checks);
  std::pair<bool, int> checkTrajCollision(const FrenetPath& ego_traj, const std::vector<Path>& obstacle_trajs, 
                                          const autoware_msgs::DetectedObjectArray& obstacles, const double margin_lon, const double margin_lat);
};

}  // namespace fiss

#endif  // FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_