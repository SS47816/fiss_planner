// SPDX-License-Identifier: Apache-2.0
// rostest: the ROS 1 node overtakes a parked car and switches planners via dynamic_reconfigure.
#include <dynamic_reconfigure/Reconfigure.h>
#include <fiss_planner_msgs/ObstacleArray.h>
#include <fiss_planner_msgs/PlannerStats.h>
#include <fiss_planner_msgs/Trajectory.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace {
fiss_planner_msgs::Trajectory::ConstPtr g_traj;
fiss_planner_msgs::PlannerStats::ConstPtr g_stats;
}  // namespace

TEST(PlannerNodeRos1, OvertakesAndReconfigures) {
  ros::NodeHandle nh;
  auto path_pub = nh.advertise<nav_msgs::Path>("/fiss_planner/reference_path", 1, true);
  auto odom_pub = nh.advertise<nav_msgs::Odometry>("/fiss_planner/odom", 1);
  auto obs_pub = nh.advertise<fiss_planner_msgs::ObstacleArray>("/fiss_planner/obstacles", 1);
  using TrajCb = boost::function<void(const fiss_planner_msgs::Trajectory::ConstPtr&)>;
  using StatsCb = boost::function<void(const fiss_planner_msgs::PlannerStats::ConstPtr&)>;
  auto traj_sub = nh.subscribe<fiss_planner_msgs::Trajectory>(
      "/fiss_planner/trajectory", 1,
      TrajCb([](const fiss_planner_msgs::Trajectory::ConstPtr& m) { g_traj = m; }));
  auto stats_sub = nh.subscribe<fiss_planner_msgs::PlannerStats>(
      "/fiss_planner/stats", 1,
      StatsCb([](const fiss_planner_msgs::PlannerStats::ConstPtr& m) { g_stats = m; }));

  nav_msgs::Path path;
  path.header.frame_id = "map";
  for (double x = -20.0; x <= 400.0; x += 10.0) {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = x;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  path_pub.publish(path);

  const auto publish_world = [&] {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = 10.0;
    odom_pub.publish(odom);
    fiss_planner_msgs::ObstacleArray obs;
    obs.header = odom.header;
    fiss_planner_msgs::Obstacle o;
    o.id = 1;
    o.length = 4.5;
    o.width = 1.9;
    o.is_static = true;
    fiss_planner_msgs::ObstacleState s;
    s.x = 35.0;
    o.prediction.push_back(s);
    obs.obstacles.push_back(o);
    obs_pub.publish(obs);
  };

  ros::Rate r(20);
  for (int i = 0; i < 200 && !g_traj; ++i) {
    publish_world();
    ros::spinOnce();
    r.sleep();
  }
  ASSERT_TRUE(g_traj);
  double max_y = 0.0;
  for (const auto& p : g_traj->points) max_y = std::max(max_y, p.y);
  EXPECT_GT(max_y, 2.5);

  dynamic_reconfigure::Reconfigure srv;
  dynamic_reconfigure::StrParameter type;
  type.name = "planner_type";
  type.value = "fop";
  srv.request.config.strs.push_back(type);
  ASSERT_TRUE(ros::service::waitForService("/fiss_planner/set_parameters", 5000));
  ASSERT_TRUE(ros::service::call("/fiss_planner/set_parameters", srv));
  bool switched = false;
  for (int i = 0; i < 100 && !switched; ++i) {
    publish_world();
    ros::spinOnce();
    switched = g_stats && g_stats->planner == "fop";
    r.sleep();
  }
  EXPECT_TRUE(switched);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_planner_node_ros1");
  return RUN_ALL_TESTS();
}
