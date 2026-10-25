// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
//
// ROS 1 (Noetic) wrapper of the fiss_planner LocalPlanner. Mirrors the
// ROS 2 node: timer-driven planning, Lanelet2 map + routing or reference-path
// mode, runtime-switchable planner via dynamic_reconfigure (applied between
// cycles, no shared globals).
#include <dynamic_reconfigure/server.h>
#include <fiss_planner_msgs/ObstacleArray.h>
#include <fiss_planner_msgs/PlannerStats.h>
#include <fiss_planner_msgs/StopLineStateArray.h>
#include <fiss_planner_msgs/Trajectory.h>
#include <fiss_planner_ros1/FissPlannerConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <memory>
#include <optional>

#include "fiss_planner/common/math.hpp"
#include "fiss_planner/planning/local_planner.hpp"
#include "fiss_planner/prediction/lane_following.hpp"
#include "fiss_planner_ros/lanelet2_adapter.hpp"

namespace fp = fiss_planner;

class PlannerNode {
 public:
  PlannerNode() : pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param("odom_to_center_offset", odom_to_center_, 0.0);
    pnh_.param("plan_on_odom", plan_on_odom_, false);
    pnh_.param("publish_candidates", publish_candidates_, false);
    std::string config_file;
    pnh_.param<std::string>("config_file", config_file, "");
    if (!config_file.empty()) base_.planner = fp::loadPlannerConfig(config_file, &vehicle_);
    pnh_.param("vehicle/length", vehicle_.length, vehicle_.length);
    pnh_.param("vehicle/width", vehicle_.width, vehicle_.width);
    pnh_.param("vehicle/wheelbase", vehicle_.wheelbase, vehicle_.wheelbase);
    pnh_.param("vehicle/rear_axle_to_center", vehicle_.rear_axle_to_center, vehicle_.rear_axle_to_center);
    pnh_.param("vehicle/max_speed", vehicle_.max_speed, vehicle_.max_speed);
    pnh_.param("vehicle/max_accel", vehicle_.max_accel, vehicle_.max_accel);
    pnh_.param("vehicle/max_decel", vehicle_.max_decel, vehicle_.max_decel);
    pnh_.param("vehicle/max_steer", vehicle_.max_steer, vehicle_.max_steer);
    pnh_.param("vehicle/max_lat_accel", vehicle_.max_lat_accel, vehicle_.max_lat_accel);
    base_.planner.keep_candidates = publish_candidates_;
    planner_ = std::make_unique<fp::LocalPlanner>(base_, vehicle_);

    std::string l2;
    pnh_.param<std::string>("map/lanelet2_path", l2, "");
    if (!l2.empty()) {
      fp::ros_common::Lanelet2Options lo;
      lo.path = l2;
      pnh_.param<std::string>("map/projector", lo.projector, "utm");
      pnh_.param("map/origin_lat", lo.origin_lat, 49.0);
      pnh_.param("map/origin_lon", lo.origin_lon, 8.4);
      lanelet_map_ = std::make_unique<fp::ros_common::Lanelet2Map>(lo);
      road_map_ = std::make_shared<fp::RoadMap>(lanelet_map_->roadMap());
      planner_->setMap(road_map_);
      has_reference_ = true;
      ROS_INFO("Loaded Lanelet2 map %s with %zu lanes", l2.c_str(), lanelet_map_->numLanes());
    }

    reconfigure_.setCallback([this](fiss_planner_ros1::FissPlannerConfig& c, uint32_t) { onReconfigure(c); });

    odom_sub_ = pnh_.subscribe("odom", 1, &PlannerNode::onOdom, this);
    path_sub_ = pnh_.subscribe("reference_path", 1, &PlannerNode::onPath, this);
    obstacles_sub_ = pnh_.subscribe("obstacles", 1, &PlannerNode::onObstacles, this);
    stop_sub_ = pnh_.subscribe("stop_lines", 5, &PlannerNode::onStopLines, this);
    goal_sub_ = pnh_.subscribe("goal", 1, &PlannerNode::onGoal, this);
    traj_pub_ = pnh_.advertise<fiss_planner_msgs::Trajectory>("trajectory", 1);
    path_pub_ = pnh_.advertise<nav_msgs::Path>("path", 1);
    stats_pub_ = pnh_.advertise<fiss_planner_msgs::PlannerStats>("stats", 10);
    candidates_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("candidates", 1);
    double rate = 10.0;
    pnh_.param("rate_hz", rate, rate);
    if (!plan_on_odom_)
      timer_ = pnh_.createTimer(ros::Duration(1.0 / std::max(0.1, rate)),
                                [this](const ros::TimerEvent&) { plan(); });
  }

 private:
  void onReconfigure(const fiss_planner_ros1::FissPlannerConfig& c) {
    fp::LocalPlannerConfig cfg = base_;
    try {
      cfg.planner.type = fp::plannerTypeFromString(c.planner_type);
    } catch (const std::exception& e) {
      ROS_ERROR("%s", e.what());
      return;
    }
    cfg.planner.num_threads = c.num_threads;
    auto& s = cfg.planner.sampling;
    s.num_width = c.num_width;
    s.num_speed = c.num_speed;
    s.num_t = c.num_t;
    s.min_t = c.min_t;
    s.max_t = c.max_t;
    s.min_speed = c.min_speed;
    auto& w = cfg.planner.cost;
    w.w_time = c.w_time;
    w.w_speed = c.w_speed;
    w.w_accel = c.w_accel;
    w.w_jerk = c.w_jerk;
    w.w_lateral = c.w_lateral;
    w.w_lane_change = c.w_lane_change;
    w.w_heuristic = c.w_heuristic;
    w.w_headway = c.w_headway;
    w.desired_gap = c.desired_gap;
    w.desired_time_headway = c.desired_time_headway;
    w.w_ttc = c.w_ttc;
    w.desired_ttc = c.desired_ttc;
    w.w_clearance = c.w_clearance;
    w.desired_clearance = c.desired_clearance;
    w.follow_time = c.follow_time;
    cfg.planner.collision.margin_lon = c.margin_lon;
    cfg.planner.collision.margin_lat = c.margin_lat;
    cfg.planner.collision.ignore_rear_in_lane = c.ignore_rear_in_lane;
    auto& sf = cfg.planner.safety;
    sf.enabled = c.safety_envelope;
    sf.min_gap = c.min_gap;
    sf.min_time_headway = c.min_time_headway;
    sf.min_ttc = c.min_ttc;
    sf.reaction_time = c.reaction_time;
    sf.crossing_time_gap = c.crossing_time_gap;
    cfg.degraded_horizon = c.degraded_horizon;
    cfg.recovery_decel = c.recovery_decel;
    try {
      prediction_mode_ = fp::predictionModeFromString(c.prediction_mode);
    } catch (const std::exception& e) {
      ROS_ERROR("%s", e.what());
    }
    cfg.smoother.enabled = c.smoother;
    cfg.stitching.enabled = c.stitching;
    cfg.stitching.latency = c.latency;
    cfg.path.lane_width = c.lane_width;
    cfg.path.num_left_lanes = c.num_left_lanes;
    cfg.path.num_right_lanes = c.num_right_lanes;
    cfg.route.max_adjacent_lanes = c.max_adjacent_lanes;
    cfg.desired_speed = c.desired_speed;
    try {
      planner_->requestConfig(cfg);  // thread-safe; applied before the next cycle
      base_ = cfg;
    } catch (const std::exception& e) {
      ROS_ERROR("Rejected configuration: %s", e.what());
    }
  }

  bool toMap(const std::string& frame, double& x, double& y, double& yaw) const {
    if (frame.empty() || frame == map_frame_) return true;
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, frame, ros::Time(0));
      const double ty = tf2::getYaw(tf.transform.rotation), c = std::cos(ty), s = std::sin(ty);
      const double nx = tf.transform.translation.x + c * x - s * y;
      y = tf.transform.translation.y + s * x + c * y;
      x = nx;
      yaw = fp::normalizeAngle(yaw + ty);
      return true;
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(2.0, "%s", e.what());
      return false;
    }
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& m) {
    double x = m->pose.pose.position.x, y = m->pose.pose.position.y,
           yaw = tf2::getYaw(m->pose.pose.orientation);
    if (!toMap(m->header.frame_id, x, y, yaw)) return;
    fp::EgoState e;
    e.x = x + odom_to_center_ * std::cos(yaw);
    e.y = y + odom_to_center_ * std::sin(yaw);
    e.yaw = yaw;
    e.v = m->twist.twist.linear.x;
    e.kappa = std::abs(e.v) > 0.5 ? m->twist.twist.angular.z / e.v : 0.0;
    if (ego_ && (m->header.stamp - ego_stamp_).toSec() > 1e-3) {
      e.a = 0.7 * ego_->a + 0.3 * (e.v - ego_->v) / (m->header.stamp - ego_stamp_).toSec();
    }
    ego_ = e;
    ego_stamp_ = m->header.stamp;
    if (plan_on_odom_) plan();
  }

  void onPath(const nav_msgs::Path::ConstPtr& m) {
    if (lanelet_map_) return;
    std::vector<double> xs, ys;
    for (const auto& p : m->poses) {
      double x = p.pose.position.x, y = p.pose.position.y, yaw = 0.0;
      if (!toMap(m->header.frame_id, x, y, yaw)) return;
      xs.push_back(x);
      ys.push_back(y);
    }
    try {
      planner_->setReferencePath(xs, ys);
      has_reference_ = true;
    } catch (const std::exception& e) {
      ROS_ERROR("Invalid reference path: %s", e.what());
    }
  }

  void onObstacles(const fiss_planner_msgs::ObstacleArray::ConstPtr& m) { obstacles_ = m; }

  void onStopLines(const fiss_planner_msgs::StopLineStateArray::ConstPtr& m) {
    for (const auto& s : m->states) planner_->setStopLineActive(s.id, s.active);
  }

  void onGoal(const geometry_msgs::PoseStamped::ConstPtr& m) {
    if (!lanelet_map_ || !ego_) {
      ROS_WARN("Goal ignored (needs a Lanelet2 map and odometry)");
      return;
    }
    double gx = m->pose.position.x, gy = m->pose.position.y, gyaw = 0.0;
    if (!toMap(m->header.frame_id, gx, gy, gyaw)) return;
    const auto route = lanelet_map_->route(ego_->x, ego_->y, gx, gy);
    if (route.empty()) {
      ROS_WARN("No route to the goal");
      return;
    }
    planner_->setMap(road_map_, route);
  }

  std::vector<fp::Obstacle> obstacles(const ros::Time& now) const {
    std::vector<fp::Obstacle> out;
    if (!obstacles_) return out;
    const double age = (now - obstacles_->header.stamp).toSec();
    for (const auto& o : obstacles_->obstacles) {
      fp::Obstacle ob;
      ob.id = o.id;
      ob.length = o.length;
      ob.width = o.width;
      ob.is_static = o.is_static;
      for (const auto& p : o.prediction) {
        double x = p.x, y = p.y, yaw = p.yaw;
        if (!toMap(obstacles_->header.frame_id, x, y, yaw)) return {};
        ob.prediction.push_back({p.t - age, x, y, yaw, p.v});
      }
      if (!ob.prediction.empty()) out.push_back(std::move(ob));
    }
    const double horizon = planner_->config().planner.sampling.max_t + 0.5;
    fp::completePredictions(out, prediction_mode_, road_map_.get(), horizon, 0.1);
    return out;
  }

  void plan() {
    if (!ego_ || !has_reference_) return;
    const auto t0 = std::chrono::steady_clock::now();
    const ros::Time now = plan_on_odom_ ? ego_stamp_ : ros::Time::now();
    fp::LocalPlanOutput out;
    try {
      out = planner_->step(now.toSec(), *ego_, obstacles(now));
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(2.0, "Planning failed: %s", e.what());
      return;
    }
    const double cycle_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const std::string planner = fp::toString(planner_->planner().type());

    fiss_planner_msgs::PlannerStats st;
    st.header.stamp = now;
    st.header.frame_id = map_frame_;
    st.planner = planner;
    st.success = out.success;
    st.fallback = out.fallback;
    st.emergency = out.emergency;
    st.stitched = out.stitched;
    st.smoothed = out.smoothed;
    st.time_ms = out.stats.time_ms;
    st.cycle_ms = cycle_ms;
    st.target_speed = out.target_speed;
    st.num_generated = out.stats.num_generated;
    st.num_validated = out.stats.num_validated;
    st.num_collision_checks = out.stats.num_collision_checks;
    st.ego_lane = out.ego_lane;
    st.degraded = out.degraded;
    st.blocking_obstacle_id = out.stats.blocking_id;
    st.lead_obstacle_id = out.stats.lead.id;
    st.lead_gap = out.stats.lead.gap;
    st.time_headway = out.stats.lead.time_headway;
    st.ttc = out.stats.lead.ttc;
    stats_pub_.publish(st);

    const auto& t = out.trajectory;
    if (t.size() < 2) return;
    fiss_planner_msgs::Trajectory traj;
    traj.header = st.header;
    traj.planner = planner;
    traj.emergency = out.emergency;
    traj.cost = t.totalCost();
    nav_msgs::Path path;
    path.header = st.header;
    for (size_t i = 0; i < t.size(); ++i) {
      fiss_planner_msgs::TrajectoryPoint p;
      p.t = t.t[i];
      p.x = t.x[i];
      p.y = t.y[i];
      p.yaw = t.yaw[i];
      p.v = t.v[i];
      p.a = t.a[i];
      p.kappa = t.kappa[i];
      p.s = i < t.s.size() ? t.s[i] : 0.0;
      p.d = i < t.d.size() ? t.d[i] : 0.0;
      traj.points.push_back(p);
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = t.x[i];
      ps.pose.position.y = t.y[i];
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, t.yaw[i]);
      ps.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(ps);
    }
    traj_pub_.publish(traj);
    path_pub_.publish(path);

    if (publish_candidates_ && candidates_pub_.getNumSubscribers() > 0) {
      visualization_msgs::MarkerArray ma;
      visualization_msgs::Marker clear;
      clear.action = visualization_msgs::Marker::DELETEALL;
      ma.markers.push_back(clear);
      int id = 0;
      for (const auto& c : out.candidates) {
        visualization_msgs::Marker m;
        m.header = st.header;
        m.ns = "candidates";
        m.id = id++;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.scale.x = 0.05;
        m.color.g = c.rejection == fp::Rejection::kOk ? 1.0f : 0.3f;
        m.color.r = c.rejection == fp::Rejection::kOk ? 0.2f : 1.0f;
        m.color.a = 0.6f;
        for (size_t i = 0; i < c.size(); i += 2) {
          geometry_msgs::Point p;
          p.x = c.x[i];
          p.y = c.y[i];
          m.points.push_back(p);
        }
        ma.markers.push_back(m);
      }
      candidates_pub_.publish(ma);
    }
  }

  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  dynamic_reconfigure::Server<fiss_planner_ros1::FissPlannerConfig> reconfigure_{pnh_};
  fp::VehicleParams vehicle_;
  fp::LocalPlannerConfig base_;
  std::unique_ptr<fp::LocalPlanner> planner_;
  std::unique_ptr<fp::ros_common::Lanelet2Map> lanelet_map_;
  std::shared_ptr<const fp::RoadMap> road_map_;
  fp::PredictionMode prediction_mode_ = fp::PredictionMode::kLaneFollowing;
  std::string map_frame_;
  double odom_to_center_ = 0.0;
  bool plan_on_odom_ = false, publish_candidates_ = false, has_reference_ = false;
  std::optional<fp::EgoState> ego_;
  ros::Time ego_stamp_;
  fiss_planner_msgs::ObstacleArray::ConstPtr obstacles_;
  ros::Subscriber odom_sub_, path_sub_, obstacles_sub_, stop_sub_, goal_sub_;
  ros::Publisher traj_pub_, path_pub_, stats_pub_, candidates_pub_;
  ros::Timer timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fiss_planner");
  PlannerNode node;
  ros::spin();
  return 0;
}
