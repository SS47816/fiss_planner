// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
// Tracks ~trajectory with MPC or Stanley+PID, publishes ackermann_msgs/AckermannDriveStamped.
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fiss_planner_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <optional>

#include "fiss_planner/control/controller.hpp"
#include "fiss_planner/planning/local_planner.hpp"

namespace fp = fiss_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fiss_controller");
  ros::NodeHandle pnh("~");
  std::string type;
  double rate, offset;
  pnh.param<std::string>("controller", type, "mpc");
  pnh.param("rate_hz", rate, 50.0);
  pnh.param("odom_to_center_offset", offset, 0.0);
  fp::VehicleParams v;
  pnh.param("vehicle/wheelbase", v.wheelbase, v.wheelbase);
  pnh.param("vehicle/rear_axle_to_center", v.rear_axle_to_center, v.rear_axle_to_center);
  pnh.param("vehicle/max_steer", v.max_steer, v.max_steer);
  pnh.param("vehicle/max_steer_rate", v.max_steer_rate, v.max_steer_rate);
  pnh.param("vehicle/max_accel", v.max_accel, v.max_accel);
  pnh.param("vehicle/max_decel", v.max_decel, v.max_decel);
  auto controller = fp::makeController(type, v);

  std::optional<fp::EgoState> ego;
  fp::Trajectory traj;
  ros::Time traj_stamp;
  auto odom_sub = pnh.subscribe<nav_msgs::Odometry>("odom", 1, [&](const nav_msgs::Odometry::ConstPtr& m) {
    const double yaw = tf2::getYaw(m->pose.pose.orientation);
    ego = fp::EgoState{m->pose.pose.position.x + offset * std::cos(yaw),
                       m->pose.pose.position.y + offset * std::sin(yaw),
                       yaw,
                       m->twist.twist.linear.x,
                       0.0,
                       0.0};
  });
  auto traj_sub = pnh.subscribe<fiss_planner_msgs::Trajectory>(
      "trajectory", 1, [&](const fiss_planner_msgs::Trajectory::ConstPtr& m) {
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
        traj = std::move(t);
        traj_stamp = m->header.stamp;
      });
  auto pub = pnh.advertise<ackermann_msgs::AckermannDriveStamped>("command", 1);
  auto timer = pnh.createTimer(ros::Duration(1.0 / rate), [&](const ros::TimerEvent&) {
    if (!ego || traj.size() < 2) return;
    const double t = (ros::Time::now() - traj_stamp).toSec();
    const auto cmd = controller->compute(*ego, traj, t);
    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.drive.steering_angle = static_cast<float>(cmd.steer);
    msg.drive.acceleration = static_cast<float>(cmd.accel);
    msg.drive.speed = static_cast<float>(fp::sampleTrajectory(traj, t).v);
    pub.publish(msg);
  });
  ros::spin();
  return 0;
}
