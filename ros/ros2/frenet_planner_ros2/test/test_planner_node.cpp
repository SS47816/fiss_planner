// SPDX-License-Identifier: Apache-2.0
// In-process integration tests of the ROS 2 planner node.
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "frenet_planner_ros2/planner_node.hpp"

using namespace std::chrono_literals;

namespace {

class Harness {
 public:
  explicit Harness(const rclcpp::NodeOptions& options) {
    planner = std::make_shared<frenet_planner_ros2::PlannerNode>(options);
    io = std::make_shared<rclcpp::Node>("test_io");
    odom_pub = io->create_publisher<nav_msgs::msg::Odometry>("/frenet_planner/odom", rclcpp::SensorDataQoS());
    path_pub = io->create_publisher<nav_msgs::msg::Path>("/frenet_planner/reference_path",
                                                         rclcpp::QoS(1).transient_local().reliable());
    obs_pub = io->create_publisher<frenet_planner_msgs::msg::ObstacleArray>("/frenet_planner/obstacles", 5);
    traj_sub = io->create_subscription<frenet_planner_msgs::msg::Trajectory>(
        "/frenet_planner/trajectory", 1,
        [this](const frenet_planner_msgs::msg::Trajectory::ConstSharedPtr& m) { last_traj = m; });
    stats_sub = io->create_subscription<frenet_planner_msgs::msg::PlannerStats>(
        "/frenet_planner/stats", 10,
        [this](const frenet_planner_msgs::msg::PlannerStats::ConstSharedPtr& m) { last_stats = m; });
    exec.add_node(planner);
    exec.add_node(io);
  }

  void publishWorld(double ego_x) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = io->now();
    odom.header.frame_id = "map";
    odom.pose.pose.position.x = ego_x;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = 10.0;
    odom_pub->publish(odom);
    frenet_planner_msgs::msg::ObstacleArray obs;
    obs.header = odom.header;
    frenet_planner_msgs::msg::Obstacle o;
    o.id = 1;
    o.length = 4.5;
    o.width = 1.9;
    o.is_static = true;
    frenet_planner_msgs::msg::ObstacleState st;
    st.x = ego_x + 35.0;
    o.prediction.push_back(st);
    obs.obstacles.push_back(o);
    obs_pub->publish(obs);
  }

  void publishPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (double x = -20.0; x <= 400.0; x += 10.0) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = x;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    path_pub->publish(path);
  }

  template <typename Pred>
  bool spinUntil(Pred pred, std::chrono::milliseconds timeout = 5000ms) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      exec.spin_some(20ms);
      if (pred()) return true;
    }
    return false;
  }

  std::shared_ptr<frenet_planner_ros2::PlannerNode> planner;
  std::shared_ptr<rclcpp::Node> io;
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<frenet_planner_msgs::msg::ObstacleArray>::SharedPtr obs_pub;
  rclcpp::Subscription<frenet_planner_msgs::msg::Trajectory>::SharedPtr traj_sub;
  rclcpp::Subscription<frenet_planner_msgs::msg::PlannerStats>::SharedPtr stats_sub;
  frenet_planner_msgs::msg::Trajectory::ConstSharedPtr last_traj;
  frenet_planner_msgs::msg::PlannerStats::ConstSharedPtr last_stats;
};

}  // namespace

TEST(PlannerNode, PathModeAvoidsObstacleAndSwitchesPlanner) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("odom_to_center_offset", 0.0);
  options.append_parameter_override("path.num_right_lanes", 0);
  options.append_parameter_override("rate_hz", 20.0);
  Harness h(options);
  h.publishPath();
  ASSERT_TRUE(h.spinUntil([&] {
    h.publishWorld(0.0);
    return h.last_traj != nullptr;
  }));
  double max_y = 0.0;
  for (const auto& p : h.last_traj->points) max_y = std::max(max_y, p.y);
  EXPECT_GT(max_y, 2.5) << "the planner should overtake the parked car on the left";
  EXPECT_EQ(h.last_traj->planner, "fiss_plus");

  // Runtime planner switch through the parameter interface.
  const auto res = h.planner->set_parameter(rclcpp::Parameter("planner.type", "fop"));
  ASSERT_TRUE(res.successful) << res.reason;
  ASSERT_TRUE(h.spinUntil([&] {
    h.publishWorld(1.0);
    return h.last_stats && h.last_stats->planner == "fop";
  }));
  EXPECT_TRUE(h.last_stats->success);

  // Invalid values are rejected and leave the planner untouched.
  EXPECT_FALSE(h.planner->set_parameter(rclcpp::Parameter("planner.type", "rrt")).successful);
  EXPECT_FALSE(h.planner->set_parameter(rclcpp::Parameter("sampling.min_t", 9.0)).successful);
  EXPECT_FALSE(h.planner->set_parameter(rclcpp::Parameter("vehicle.length", 3.0)).successful);
}

TEST(PlannerNode, LoadsLanelet2Map) {
  std::string map;
  try {
    map = ament_index_cpp::get_package_share_directory("lanelet2_maps") + "/res/mapping_example.osm";
  } catch (const std::exception&) {
    GTEST_SKIP() << "lanelet2_maps not installed";
  }
  if (!std::filesystem::exists(map)) GTEST_SKIP() << "sample map missing";
  rclcpp::NodeOptions options;
  options.append_parameter_override("map.lanelet2_path", map);
  options.append_parameter_override("map.origin_lat", 49.0);
  options.append_parameter_override("map.origin_lon", 8.4);
  EXPECT_NO_THROW({ Harness h(options); });
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
