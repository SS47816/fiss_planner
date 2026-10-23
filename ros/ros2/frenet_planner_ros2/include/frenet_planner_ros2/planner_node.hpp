// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <frenet_planner_msgs/msg/obstacle_array.hpp>
#include <frenet_planner_msgs/msg/planner_stats.hpp>
#include <frenet_planner_msgs/msg/stop_line_state_array.hpp>
#include <frenet_planner_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frenet_planner/control/controller.hpp"
#include "frenet_planner/planning/local_planner.hpp"
#include "frenet_planner/prediction/lane_following.hpp"

namespace frenet_planner::ros_common {
class Lanelet2Map;
}

namespace frenet_planner_ros2 {

/// Local planner node. Inputs: ~/odom, ~/obstacles, and either a Lanelet2
/// map (parameter map.lanelet2_path, route from ~/goal) or ~/reference_path.
/// Outputs: ~/trajectory, ~/path, ~/reference, ~/stats, ~/candidates and
/// ~/markers (ego, obstacles, predictions; blocking and lead obstacles highlighted).
/// Planning runs on a timer (or on every odometry message with
/// plan_on_odom); parameters can be changed at runtime and are applied
/// between cycles.
class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PlannerNode() override;

 private:
  void declareParameters();
  frenet_planner::LocalPlannerConfig configFromParameters() const;
  rcl_interfaces::msg::SetParametersResult onParameters(const std::vector<rclcpp::Parameter>& params);

  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void onPath(const nav_msgs::msg::Path::ConstSharedPtr& msg);
  void onObstacles(const frenet_planner_msgs::msg::ObstacleArray::ConstSharedPtr& msg);
  void onStopLines(const frenet_planner_msgs::msg::StopLineStateArray::ConstSharedPtr& msg);
  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void planCycle();

  bool toMapFrame(const std::string& frame, const rclcpp::Time& stamp, double& x, double& y,
                  double& yaw) const;
  std::vector<frenet_planner::Obstacle> obstaclesAt(const rclcpp::Time& now) const;
  void publish(const frenet_planner::LocalPlanOutput& out, const rclcpp::Time& stamp, double cycle_ms);
  void publishScene(const frenet_planner::LocalPlanOutput& out,
                    const std::vector<frenet_planner::Obstacle>& obstacles, const rclcpp::Time& stamp);

  frenet_planner::VehicleParams vehicle_;
  std::string map_frame_;
  double odom_to_center_ = 0.0;
  bool plan_on_odom_ = false;
  bool publish_candidates_ = false;
  frenet_planner::PredictionMode prediction_mode_ = frenet_planner::PredictionMode::kLaneFollowing;
  std::shared_ptr<const frenet_planner::RoadMap> road_map_;

  std::unique_ptr<frenet_planner::LocalPlanner> planner_;
  std::unique_ptr<frenet_planner::ros_common::Lanelet2Map> lanelet_map_;
  bool has_reference_ = false;

  std::optional<frenet_planner::EgoState> ego_;
  rclcpp::Time ego_stamp_;
  double prev_v_ = 0.0;
  std::optional<rclcpp::Time> prev_v_stamp_;
  frenet_planner_msgs::msg::ObstacleArray::ConstSharedPtr obstacles_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<frenet_planner_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<frenet_planner_msgs::msg::StopLineStateArray>::SharedPtr stop_lines_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<frenet_planner_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_pub_;
  rclcpp::Publisher<frenet_planner_msgs::msg::PlannerStats>::SharedPtr stats_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

/// Tracks ~/trajectory with an MPC or Stanley controller and publishes
/// ackermann_msgs/AckermannDriveStamped on ~/command.
class ControllerNode : public rclcpp::Node {
 public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void onTimer();
  frenet_planner::VehicleParams vehicle_;
  std::unique_ptr<frenet_planner::Controller> controller_;
  double odom_to_center_ = 0.0;
  std::optional<frenet_planner::EgoState> ego_;
  frenet_planner::Trajectory trajectory_;
  rclcpp::Time trajectory_stamp_;
  bool has_trajectory_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<frenet_planner_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace frenet_planner_ros2
