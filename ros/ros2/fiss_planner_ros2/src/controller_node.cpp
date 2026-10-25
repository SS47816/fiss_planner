// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include <tf2/utils.h>

#include <chrono>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fiss_planner/planning/local_planner.hpp"
#include "fiss_planner/smoothing/piecewise_jerk.hpp"
#include "fiss_planner_ros2/planner_node.hpp"

namespace fiss_planner_ros2 {

namespace fp = fiss_planner;

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("fiss_controller", options) {
  const std::string type = declare_parameter("controller", std::string("mpc"));
  const double rate = declare_parameter("rate_hz", 50.0);
  odom_to_center_ = declare_parameter("odom_to_center_offset", 0.0);
  vehicle_.length = declare_parameter("vehicle.length", vehicle_.length);
  vehicle_.width = declare_parameter("vehicle.width", vehicle_.width);
  vehicle_.wheelbase = declare_parameter("vehicle.wheelbase", vehicle_.wheelbase);
  vehicle_.rear_axle_to_center =
      declare_parameter("vehicle.rear_axle_to_center", vehicle_.rear_axle_to_center);
  vehicle_.max_accel = declare_parameter("vehicle.max_accel", vehicle_.max_accel);
  vehicle_.max_decel = declare_parameter("vehicle.max_decel", vehicle_.max_decel);
  vehicle_.max_steer = declare_parameter("vehicle.max_steer", vehicle_.max_steer);
  vehicle_.max_steer_rate = declare_parameter("vehicle.max_steer_rate", vehicle_.max_steer_rate);
  controller_ = fp::makeController(type, vehicle_);
  if (type == "mpc" && !fp::hasOptimizers()) {
    RCLCPP_WARN(get_logger(), "Built without OSQP: the MPC falls back to Stanley + PID");
  }

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::ConstSharedPtr& m) {
        const double yaw = tf2::getYaw(m->pose.pose.orientation);
        fp::EgoState e;
        e.x = m->pose.pose.position.x + odom_to_center_ * std::cos(yaw);
        e.y = m->pose.pose.position.y + odom_to_center_ * std::sin(yaw);
        e.yaw = yaw;
        e.v = m->twist.twist.linear.x;
        ego_ = e;
      });
  traj_sub_ = create_subscription<fiss_planner_msgs::msg::Trajectory>(
      "~/trajectory", rclcpp::QoS(1), [this](const fiss_planner_msgs::msg::Trajectory::ConstSharedPtr& m) {
        fp::Trajectory t;
        for (const auto& p : m->points) {
          t.t.push_back(p.t);
          t.x.push_back(p.x);
          t.y.push_back(p.y);
          t.yaw.push_back(p.yaw);
          t.v.push_back(p.v);
          t.a.push_back(p.a);
          t.kappa.push_back(p.kappa);
        }
        t.is_cartesian = true;
        trajectory_ = std::move(t);
        trajectory_stamp_ = rclcpp::Time(m->header.stamp, get_clock()->get_clock_type());
        has_trajectory_ = trajectory_.size() >= 2;
      });
  cmd_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("~/command", rclcpp::QoS(1));
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate)), [this] { onTimer(); });
}

void ControllerNode::onTimer() {
  if (!ego_ || !has_trajectory_) return;
  const double t = (now() - trajectory_stamp_).seconds();
  const auto cmd = controller_->compute(*ego_, trajectory_, t);
  ackermann_msgs::msg::AckermannDriveStamped msg;
  msg.header.stamp = now();
  msg.drive.steering_angle = static_cast<float>(cmd.steer);
  msg.drive.acceleration = static_cast<float>(cmd.accel);
  msg.drive.speed = static_cast<float>(fp::sampleTrajectory(trajectory_, t).v);
  cmd_pub_->publish(msg);
}

}  // namespace fiss_planner_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(fiss_planner_ros2::ControllerNode)
