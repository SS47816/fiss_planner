// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include <rclcpp/rclcpp.hpp>

#include "frenet_planner_ros2/planner_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frenet_planner_ros2::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
