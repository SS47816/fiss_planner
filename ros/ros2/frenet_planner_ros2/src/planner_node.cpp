// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#include "frenet_planner_ros2/planner_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "frenet_planner/common/math.hpp"
#include "frenet_planner/prediction/lane_following.hpp"
#include "frenet_planner_ros/lanelet2_adapter.hpp"

namespace frenet_planner_ros2 {

using frenet_planner::EgoState;
using frenet_planner::LocalPlannerConfig;
using frenet_planner::Obstacle;
namespace fp = frenet_planner;

namespace {

/// Runtime-tunable parameters, bound to fields of LocalPlannerConfig.
struct Binding {
  std::string name;
  std::function<rclcpp::ParameterValue(const LocalPlannerConfig&)> get;
  std::function<void(LocalPlannerConfig&, const rclcpp::Parameter&)> set;
  std::string description;
};

template <typename T>
Binding bind(const std::string& name, T LocalPlannerConfig::*section, double T::*field,
             const std::string& desc) {
  return {name, [=](const LocalPlannerConfig& c) { return rclcpp::ParameterValue((c.*section).*field); },
          [=](LocalPlannerConfig& c, const rclcpp::Parameter& p) { (c.*section).*field = p.as_double(); },
          desc};
}
template <typename T>
Binding bindInt(const std::string& name, T LocalPlannerConfig::*section, int T::*field,
                const std::string& desc) {
  return {name, [=](const LocalPlannerConfig& c) { return rclcpp::ParameterValue((c.*section).*field); },
          [=](LocalPlannerConfig& c, const rclcpp::Parameter& p) {
            (c.*section).*field = static_cast<int>(p.as_int());
          },
          desc};
}
template <typename T>
Binding bindBool(const std::string& name, T LocalPlannerConfig::*section, bool T::*field,
                 const std::string& desc) {
  return {name, [=](const LocalPlannerConfig& c) { return rclcpp::ParameterValue((c.*section).*field); },
          [=](LocalPlannerConfig& c, const rclcpp::Parameter& p) { (c.*section).*field = p.as_bool(); },
          desc};
}

/// Planner-config fields reached through LocalPlannerConfig::planner.
template <typename S, typename F>
Binding bindPlanner(const std::string& name, S fp::PlannerConfig::*section, F S::*field,
                    const std::string& desc) {
  return {name,
          [=](const LocalPlannerConfig& c) { return rclcpp::ParameterValue((c.planner.*section).*field); },
          [=](LocalPlannerConfig& c, const rclcpp::Parameter& p) {
            if constexpr (std::is_same_v<F, double>) (c.planner.*section).*field = p.as_double();
            if constexpr (std::is_same_v<F, int>) (c.planner.*section).*field = static_cast<int>(p.as_int());
            if constexpr (std::is_same_v<F, bool>) (c.planner.*section).*field = p.as_bool();
          },
          desc};
}

const std::vector<Binding>& bindings() {
  static const std::vector<Binding> b = [] {
    using fp::CollisionConfig;
    using fp::CostConfig;
    using fp::FissPlusConfig;
    using fp::PlannerConfig;
    using fp::SafetyConfig;
    using fp::SamplingConfig;
    std::vector<Binding> v;
    v.push_back(
        {"planner.type",
         [](const LocalPlannerConfig& c) { return rclcpp::ParameterValue(fp::toString(c.planner.type)); },
         [](LocalPlannerConfig& c, const rclcpp::Parameter& p) {
           c.planner.type = fp::plannerTypeFromString(p.as_string());
         },
         "fop | fop_plus | fiss | fiss_plus"});
    v.push_back({"planner.num_threads",
                 [](const LocalPlannerConfig& c) { return rclcpp::ParameterValue(c.planner.num_threads); },
                 [](LocalPlannerConfig& c, const rclcpp::Parameter& p) {
                   c.planner.num_threads = static_cast<int>(p.as_int());
                 },
                 "validation threads (FOP/FOP+)"});
    v.push_back(bindPlanner("sampling.num_width", &PlannerConfig::sampling, &SamplingConfig::num_width,
                            "lateral samples per lane"));
    v.push_back(bindPlanner("sampling.num_speed", &PlannerConfig::sampling, &SamplingConfig::num_speed,
                            "speed samples"));
    v.push_back(
        bindPlanner("sampling.num_t", &PlannerConfig::sampling, &SamplingConfig::num_t, "horizon samples"));
    v.push_back(bindPlanner("sampling.min_t", &PlannerConfig::sampling, &SamplingConfig::min_t, "[s]"));
    v.push_back(bindPlanner("sampling.max_t", &PlannerConfig::sampling, &SamplingConfig::max_t, "[s]"));
    v.push_back(
        bindPlanner("sampling.min_speed", &PlannerConfig::sampling, &SamplingConfig::min_speed, "[m/s]"));
    v.push_back(bindPlanner("cost.w_time", &PlannerConfig::cost, &CostConfig::w_time, ""));
    v.push_back(bindPlanner("cost.w_speed", &PlannerConfig::cost, &CostConfig::w_speed, ""));
    v.push_back(bindPlanner("cost.w_accel", &PlannerConfig::cost, &CostConfig::w_accel, ""));
    v.push_back(bindPlanner("cost.w_jerk", &PlannerConfig::cost, &CostConfig::w_jerk, ""));
    v.push_back(bindPlanner("cost.w_lateral", &PlannerConfig::cost, &CostConfig::w_lateral, ""));
    v.push_back(bindPlanner("cost.w_lane_change", &PlannerConfig::cost, &CostConfig::w_lane_change, ""));
    v.push_back(bindPlanner("cost.w_heuristic", &PlannerConfig::cost, &CostConfig::w_heuristic,
                            "FISS history heuristic"));
    v.push_back(bindPlanner("cost.w_headway", &PlannerConfig::cost, &CostConfig::w_headway,
                            "gap to the vehicle ahead"));
    v.push_back(
        bindPlanner("cost.desired_gap", &PlannerConfig::cost, &CostConfig::desired_gap, "[m] at standstill"));
    v.push_back(bindPlanner("cost.desired_time_headway", &PlannerConfig::cost,
                            &CostConfig::desired_time_headway, "[s]"));
    v.push_back(bindPlanner("cost.w_ttc", &PlannerConfig::cost, &CostConfig::w_ttc, "time to collision"));
    v.push_back(bindPlanner("cost.desired_ttc", &PlannerConfig::cost, &CostConfig::desired_ttc, "[s]"));
    v.push_back(
        bindPlanner("cost.w_clearance", &PlannerConfig::cost, &CostConfig::w_clearance, "lateral clearance"));
    v.push_back(
        bindPlanner("cost.desired_clearance", &PlannerConfig::cost, &CostConfig::desired_clearance, "[m]"));
    v.push_back(bindPlanner("cost.follow_time", &PlannerConfig::cost, &CostConfig::follow_time,
                            "[s] car following; 0 = off"));
    v.push_back(bindPlanner("safety.enabled", &PlannerConfig::safety, &SafetyConfig::enabled,
                            "hard gap / TTC envelope"));
    v.push_back(bindPlanner("safety.min_gap", &PlannerConfig::safety, &SafetyConfig::min_gap, "[m]"));
    v.push_back(bindPlanner("safety.min_time_headway", &PlannerConfig::safety,
                            &SafetyConfig::min_time_headway, "[s]"));
    v.push_back(bindPlanner("safety.min_ttc", &PlannerConfig::safety, &SafetyConfig::min_ttc, "[s]"));
    v.push_back(
        bindPlanner("safety.reaction_time", &PlannerConfig::safety, &SafetyConfig::reaction_time, "[s]"));
    v.push_back(
        bindPlanner("collision.margin_lon", &PlannerConfig::collision, &CollisionConfig::margin_lon, "[m]"));
    v.push_back(
        bindPlanner("collision.margin_lat", &PlannerConfig::collision, &CollisionConfig::margin_lat, "[m]"));
    v.push_back(bindPlanner("collision.ignore_rear_in_lane", &PlannerConfig::collision,
                            &CollisionConfig::ignore_rear_in_lane, "followers keep their distance"));
    v.push_back(bindPlanner("fiss_plus.refine", &PlannerConfig::fiss_plus, &FissPlusConfig::refine, ""));
    v.push_back(bindPlanner("fiss_plus.max_refine_iters", &PlannerConfig::fiss_plus,
                            &FissPlusConfig::max_refine_iters, ""));
    v.push_back(bindPlanner("fiss_plus.time_budget", &PlannerConfig::fiss_plus, &FissPlusConfig::time_budget,
                            "[s], 0 = none"));
    v.push_back(bindBool("smoother.enabled", &LocalPlannerConfig::smoother, &fp::SmootherConfig::enabled,
                         "QP refinement"));
    v.push_back(
        bindBool("stitching.enabled", &LocalPlannerConfig::stitching, &fp::StitchingConfig::enabled, ""));
    v.push_back(
        bind("stitching.latency", &LocalPlannerConfig::stitching, &fp::StitchingConfig::latency, "[s]"));
    v.push_back(bind("path.lane_width", &LocalPlannerConfig::path, &fp::PathModeConfig::lane_width,
                     "[m] (path mode)"));
    v.push_back(
        bindInt("path.num_left_lanes", &LocalPlannerConfig::path, &fp::PathModeConfig::num_left_lanes, ""));
    v.push_back(
        bindInt("path.num_right_lanes", &LocalPlannerConfig::path, &fp::PathModeConfig::num_right_lanes, ""));
    v.push_back(bindBool("path.allow_left", &LocalPlannerConfig::path, &fp::PathModeConfig::allow_left, ""));
    v.push_back(
        bindBool("path.allow_right", &LocalPlannerConfig::path, &fp::PathModeConfig::allow_right, ""));
    v.push_back(bindInt("route.max_adjacent_lanes", &LocalPlannerConfig::route,
                        &fp::RouteCorridorConfig::max_adjacent_lanes, "(map mode)"));
    v.push_back({"desired_speed",
                 [](const LocalPlannerConfig& c) { return rclcpp::ParameterValue(c.desired_speed); },
                 [](LocalPlannerConfig& c, const rclcpp::Parameter& p) { c.desired_speed = p.as_double(); },
                 "[m/s]"});
    v.push_back(
        {"degraded_horizon",
         [](const LocalPlannerConfig& c) { return rclcpp::ParameterValue(c.degraded_horizon); },
         [](LocalPlannerConfig& c, const rclcpp::Parameter& p) { c.degraded_horizon = p.as_double(); },
         "[s] collision horizon of the last retry before braking; 0 = off"});
    v.push_back({"recovery_decel",
                 [](const LocalPlannerConfig& c) { return rclcpp::ParameterValue(c.recovery_decel); },
                 [](LocalPlannerConfig& c, const rclcpp::Parameter& p) { c.recovery_decel = p.as_double(); },
                 "[m/s^2] initial deceleration of the first plan after an emergency stop"});
    return v;
  }();
  return b;
}

}  // namespace

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("frenet_planner", options) {
  declareParameters();
  planner_ = std::make_unique<fp::LocalPlanner>(configFromParameters(), vehicle_);

  const std::string l2_path = get_parameter("map.lanelet2_path").as_string();
  if (!l2_path.empty()) {
    fp::ros_common::Lanelet2Options lo;
    lo.path = l2_path;
    lo.projector = get_parameter("map.projector").as_string();
    lo.origin_lat = get_parameter("map.origin_lat").as_double();
    lo.origin_lon = get_parameter("map.origin_lon").as_double();
    lanelet_map_ = std::make_unique<fp::ros_common::Lanelet2Map>(lo);
    road_map_ = std::make_shared<fp::RoadMap>(lanelet_map_->roadMap());
    planner_->setMap(road_map_);
    has_reference_ = true;
    RCLCPP_INFO(get_logger(), "Loaded Lanelet2 map %s with %zu lanes", l2_path.c_str(),
                lanelet_map_->numLanes());
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/odom", sensor_qos, [this](const nav_msgs::msg::Odometry::ConstSharedPtr& m) { onOdom(m); });
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "~/reference_path", latched, [this](const nav_msgs::msg::Path::ConstSharedPtr& m) { onPath(m); });
  obstacles_sub_ = create_subscription<frenet_planner_msgs::msg::ObstacleArray>(
      "~/obstacles", rclcpp::QoS(5),
      [this](const frenet_planner_msgs::msg::ObstacleArray::ConstSharedPtr& m) { onObstacles(m); });
  stop_lines_sub_ = create_subscription<frenet_planner_msgs::msg::StopLineStateArray>(
      "~/stop_lines", rclcpp::QoS(5),
      [this](const frenet_planner_msgs::msg::StopLineStateArray::ConstSharedPtr& m) { onStopLines(m); });
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/goal", latched, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& m) { onGoal(m); });

  traj_pub_ = create_publisher<frenet_planner_msgs::msg::Trajectory>("~/trajectory", rclcpp::QoS(1));
  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1));
  ref_pub_ = create_publisher<nav_msgs::msg::Path>("~/reference", rclcpp::QoS(1));
  stats_pub_ = create_publisher<frenet_planner_msgs::msg::PlannerStats>("~/stats", rclcpp::QoS(10));
  candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/candidates", rclcpp::QoS(1));
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", rclcpp::QoS(1));

  param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& p) { return onParameters(p); });

  if (!plan_on_odom_) {
    const double rate = get_parameter("rate_hz").as_double();
    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / std::max(0.1, rate)), [this] { planCycle(); });
  }
  RCLCPP_INFO(get_logger(), "frenet_planner ready (%s, %s)", fp::toString(planner_->planner().type()).c_str(),
              lanelet_map_ ? "Lanelet2 map" : "reference-path mode");
}

PlannerNode::~PlannerNode() = default;

void PlannerNode::declareParameters() {
  // Base configuration: core defaults, optionally overridden by a YAML file in
  // the core format; ROS parameters then override individual fields.
  LocalPlannerConfig base;
  const std::string config_file = declare_parameter("config_file", std::string(""));
  if (!config_file.empty()) base.planner = fp::loadPlannerConfig(config_file, &vehicle_);

  map_frame_ = declare_parameter("map_frame", std::string("map"));
  odom_to_center_ = declare_parameter("odom_to_center_offset", 0.0);
  plan_on_odom_ = declare_parameter("plan_on_odom", false);
  publish_candidates_ = declare_parameter("publish_candidates", false);
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description =
        "predictions for obstacles sent with only their current state: as_given | constant_velocity | "
        "lane_following (constant velocity without a map)";
    prediction_mode_ =
        fp::predictionModeFromString(declare_parameter("prediction.mode", std::string("lane_following"), d));
  }
  declare_parameter("rate_hz", 10.0);
  declare_parameter("map.lanelet2_path", std::string(""));
  declare_parameter("map.projector", std::string("utm"));
  declare_parameter("map.origin_lat", 49.0);
  declare_parameter("map.origin_lon", 8.4);

  vehicle_.length = declare_parameter("vehicle.length", vehicle_.length);
  vehicle_.width = declare_parameter("vehicle.width", vehicle_.width);
  vehicle_.wheelbase = declare_parameter("vehicle.wheelbase", vehicle_.wheelbase);
  vehicle_.rear_axle_to_center =
      declare_parameter("vehicle.rear_axle_to_center", vehicle_.rear_axle_to_center);
  vehicle_.max_speed = declare_parameter("vehicle.max_speed", vehicle_.max_speed);
  vehicle_.max_accel = declare_parameter("vehicle.max_accel", vehicle_.max_accel);
  vehicle_.max_decel = declare_parameter("vehicle.max_decel", vehicle_.max_decel);
  vehicle_.max_steer = declare_parameter("vehicle.max_steer", vehicle_.max_steer);
  vehicle_.max_steer_rate = declare_parameter("vehicle.max_steer_rate", vehicle_.max_steer_rate);
  vehicle_.max_lat_accel = declare_parameter("vehicle.max_lat_accel", vehicle_.max_lat_accel);

  for (const auto& b : bindings()) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = b.description;
    declare_parameter(b.name, b.get(base), d);
  }
}

LocalPlannerConfig PlannerNode::configFromParameters() const {
  LocalPlannerConfig cfg;
  const std::string config_file = get_parameter("config_file").as_string();
  if (!config_file.empty()) cfg.planner = fp::loadPlannerConfig(config_file);
  for (const auto& b : bindings()) b.set(cfg, get_parameter(b.name));
  cfg.planner.keep_candidates = publish_candidates_;
  return cfg;
}

rcl_interfaces::msg::SetParametersResult PlannerNode::onParameters(
    const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  LocalPlannerConfig cfg = configFromParameters();
  for (const auto& p : params) {
    const auto it = std::find_if(bindings().begin(), bindings().end(),
                                 [&](const Binding& b) { return b.name == p.get_name(); });
    if (it == bindings().end()) {
      if (p.get_name() == "publish_candidates") continue;
      if (p.get_name() == "prediction.mode") {
        try {
          fp::predictionModeFromString(p.as_string());
        } catch (const std::exception& e) {
          result.successful = false;
          result.reason = e.what();
          return result;
        }
        continue;
      }
      result.successful = false;
      result.reason = "parameter '" + p.get_name() + "' cannot be changed at runtime";
      return result;
    }
    try {
      it->set(cfg, p);
    } catch (const std::exception& e) {
      result.successful = false;
      result.reason = p.get_name() + ": " + e.what();
      return result;
    }
  }
  for (const auto& p : params) {
    if (p.get_name() == "publish_candidates") publish_candidates_ = p.as_bool();
    if (p.get_name() == "prediction.mode") prediction_mode_ = fp::predictionModeFromString(p.as_string());
  }
  cfg.planner.keep_candidates = publish_candidates_;
  try {
    planner_->requestConfig(cfg);  // validated here, applied at the next cycle
  } catch (const std::exception& e) {
    result.successful = false;
    result.reason = e.what();
  }
  return result;
}

bool PlannerNode::toMapFrame(const std::string& frame, const rclcpp::Time& stamp, double& x, double& y,
                             double& yaw) const {
  if (frame.empty() || frame == map_frame_) return true;
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, frame, stamp, rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException& e) {
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, frame, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf %s -> %s: %s", frame.c_str(),
                           map_frame_.c_str(), e.what());
      return false;
    }
  }
  const double tyaw = tf2::getYaw(tf.transform.rotation);
  const double c = std::cos(tyaw), s = std::sin(tyaw);
  const double nx = tf.transform.translation.x + c * x - s * y;
  const double ny = tf.transform.translation.y + s * x + c * y;
  x = nx;
  y = ny;
  yaw = fp::normalizeAngle(yaw + tyaw);
  return true;
}

void PlannerNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  double x = msg->pose.pose.position.x, y = msg->pose.pose.position.y;
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  if (!toMapFrame(msg->header.frame_id, msg->header.stamp, x, y, yaw)) return;
  EgoState ego;
  ego.x = x + odom_to_center_ * std::cos(yaw);
  ego.y = y + odom_to_center_ * std::sin(yaw);
  ego.yaw = yaw;
  const auto& lin = msg->twist.twist.linear;
  ego.v = std::copysign(std::hypot(lin.x, lin.y), lin.x >= 0.0 ? 1.0 : -1.0);
  ego.kappa = std::abs(ego.v) > 0.5 ? msg->twist.twist.angular.z / ego.v : 0.0;
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (prev_v_stamp_) {
    const double dt = (stamp - *prev_v_stamp_).seconds();
    if (dt > 1e-3) ego.a = 0.7 * (ego_ ? ego_->a : 0.0) + 0.3 * (ego.v - prev_v_) / dt;  // low-pass
  }
  prev_v_ = ego.v;
  prev_v_stamp_ = stamp;
  ego_ = ego;
  ego_stamp_ = stamp;
  if (plan_on_odom_) planCycle();
}

void PlannerNode::onPath(const nav_msgs::msg::Path::ConstSharedPtr& msg) {
  if (lanelet_map_) return;  // the map provides the reference
  std::vector<double> xs, ys;
  for (const auto& p : msg->poses) {
    double x = p.pose.position.x, y = p.pose.position.y, yaw = 0.0;
    if (!toMapFrame(msg->header.frame_id, msg->header.stamp, x, y, yaw)) return;
    xs.push_back(x);
    ys.push_back(y);
  }
  try {
    planner_->setReferencePath(xs, ys);
    has_reference_ = true;
    RCLCPP_INFO(get_logger(), "Reference path with %zu points received", xs.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Invalid reference path: %s", e.what());
  }
}

void PlannerNode::onObstacles(const frenet_planner_msgs::msg::ObstacleArray::ConstSharedPtr& msg) {
  obstacles_ = msg;
}

void PlannerNode::onStopLines(const frenet_planner_msgs::msg::StopLineStateArray::ConstSharedPtr& msg) {
  for (const auto& s : msg->states) planner_->setStopLineActive(s.id, s.active);
}

void PlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  if (!lanelet_map_) {
    RCLCPP_WARN(get_logger(), "Goal ignored: routing needs a Lanelet2 map (map.lanelet2_path)");
    return;
  }
  if (!ego_) {
    RCLCPP_WARN(get_logger(), "Goal ignored: no odometry yet");
    return;
  }
  double gx = msg->pose.position.x, gy = msg->pose.position.y, gyaw = 0.0;
  if (!toMapFrame(msg->header.frame_id, msg->header.stamp, gx, gy, gyaw)) return;
  const auto route = lanelet_map_->route(ego_->x, ego_->y, gx, gy);
  if (route.empty()) {
    RCLCPP_WARN(get_logger(), "No route to the goal");
    return;
  }
  planner_->setMap(road_map_, route);
  RCLCPP_INFO(get_logger(), "Route with %zu lanelets", route.size());
}

std::vector<Obstacle> PlannerNode::obstaclesAt(const rclcpp::Time& now) const {
  std::vector<Obstacle> out;
  if (!obstacles_) return out;
  const rclcpp::Time stamp(obstacles_->header.stamp, get_clock()->get_clock_type());
  const double age = (now - stamp).seconds();
  for (const auto& o : obstacles_->obstacles) {
    Obstacle ob;
    ob.id = o.id;
    ob.length = o.length;
    ob.width = o.width;
    ob.is_static = o.is_static;
    for (const auto& p : o.prediction) {
      double x = p.x, y = p.y, yaw = p.yaw;
      if (!toMapFrame(obstacles_->header.frame_id, stamp, x, y, yaw)) return {};
      ob.prediction.push_back({p.t - age, x, y, yaw, p.v});
    }
    if (!ob.prediction.empty()) out.push_back(std::move(ob));
  }
  const double horizon = planner_->config().planner.sampling.max_t + 0.5;
  fp::completePredictions(out, prediction_mode_, road_map_.get(), horizon, 0.1);
  return out;
}

void PlannerNode::planCycle() {
  if (!ego_ || !has_reference_) return;
  const auto t0 = std::chrono::steady_clock::now();
  const rclcpp::Time now = plan_on_odom_ ? ego_stamp_ : this->now();
  fp::LocalPlanOutput out;
  const std::vector<Obstacle> obstacles = obstaclesAt(now);
  try {
    out = planner_->step(now.seconds(), *ego_, obstacles);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Planning failed: %s", e.what());
    return;
  }
  const double cycle_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  publish(out, now, cycle_ms);
  if (markers_pub_->get_subscription_count() > 0) publishScene(out, obstacles, now);
}

void PlannerNode::publish(const fp::LocalPlanOutput& out, const rclcpp::Time& stamp, double cycle_ms) {
  const std::string planner = fp::toString(planner_->planner().type());
  frenet_planner_msgs::msg::PlannerStats stats;
  stats.header.stamp = stamp;
  stats.header.frame_id = map_frame_;
  stats.planner = planner;
  stats.success = out.success;
  stats.fallback = out.fallback;
  stats.emergency = out.emergency;
  stats.stitched = out.stitched;
  stats.smoothed = out.smoothed;
  stats.time_ms = out.stats.time_ms;
  stats.cycle_ms = cycle_ms;
  stats.target_speed = out.target_speed;
  stats.num_generated = out.stats.num_generated;
  stats.num_validated = out.stats.num_validated;
  stats.num_collision_checks = out.stats.num_collision_checks;
  stats.ego_lane = out.ego_lane;
  stats.degraded = out.degraded;
  stats.blocking_obstacle_id = out.stats.blocking_id;
  stats.lead_obstacle_id = out.stats.lead.id;
  stats.lead_gap = out.stats.lead.gap;
  stats.time_headway = out.stats.lead.time_headway;
  stats.ttc = out.stats.lead.ttc;
  stats_pub_->publish(stats);

  const auto& t = out.trajectory;
  if (t.size() < 2) return;
  frenet_planner_msgs::msg::Trajectory traj;
  traj.header = stats.header;
  traj.planner = planner;
  traj.emergency = out.emergency;
  traj.cost = t.totalCost();
  nav_msgs::msg::Path path;
  path.header = stats.header;
  for (size_t i = 0; i < t.size(); ++i) {
    frenet_planner_msgs::msg::TrajectoryPoint p;
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
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = t.x[i];
    ps.pose.position.y = t.y[i];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, t.yaw[i]);
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }
  traj_pub_->publish(traj);
  path_pub_->publish(path);

  if (out.reference && ref_pub_->get_subscription_count() > 0) {
    nav_msgs::msg::Path ref;
    ref.header = stats.header;
    for (const auto& r : out.reference->sample(2.0)) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = ref.header;
      ps.pose.position.x = r.x;
      ps.pose.position.y = r.y;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, r.yaw);
      ps.pose.orientation = tf2::toMsg(q);
      ref.poses.push_back(ps);
    }
    ref_pub_->publish(ref);
  }

  if (publish_candidates_ && candidates_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    double lo = 1e9, hi = -1e9;
    for (const auto& c : out.candidates) {
      lo = std::min(lo, c.totalCost());
      hi = std::max(hi, c.totalCost());
    }
    int id = 0;
    for (const auto& c : out.candidates) {
      visualization_msgs::msg::Marker m;
      m.header = stats.header;
      m.ns = "candidates";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.scale.x = 0.05;
      const double w = hi > lo ? (c.totalCost() - lo) / (hi - lo) : 0.0;  // green (cheap) -> red
      m.color.r = static_cast<float>(w);
      m.color.g = static_cast<float>(1.0 - w);
      m.color.a = c.rejection == fp::Rejection::kOk ? 0.8f : 0.25f;
      for (size_t i = 0; i < c.size(); i += 2) {
        geometry_msgs::msg::Point p;
        p.x = c.x[i];
        p.y = c.y[i];
        m.points.push_back(p);
      }
      markers.markers.push_back(m);
    }
    candidates_pub_->publish(markers);
  }
}

void PlannerNode::publishScene(const fp::LocalPlanOutput& out, const std::vector<Obstacle>& obstacles,
                               const rclcpp::Time& stamp) {
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray markers;
  Marker clear;
  clear.action = Marker::DELETEALL;
  markers.markers.push_back(clear);
  int id = 0;
  const auto box_marker = [&](const fp::Box2d& b, const std::string& ns, float r, float g, float bl) {
    Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = map_frame_;
    m.ns = ns;
    m.id = id++;
    m.type = Marker::CUBE;
    m.pose.position.x = b.cx;
    m.pose.position.y = b.cy;
    m.pose.position.z = 0.75;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, b.yaw);
    m.pose.orientation = tf2::toMsg(q);
    m.scale.x = b.length;
    m.scale.y = b.width;
    m.scale.z = 1.5;
    m.color.r = r;
    m.color.g = g;
    m.color.b = bl;
    m.color.a = 0.85f;
    return m;
  };
  if (ego_) {
    const auto& v = vehicle_;
    markers.markers.push_back(
        box_marker({ego_->x, ego_->y, ego_->yaw, v.length, v.width}, "ego", 0.25f, 0.66f, 1.0f));
  }
  for (const auto& ob : obstacles) {
    const auto box = ob.boxAt(0.0);
    if (!box) continue;
    const bool blocking = ob.id == out.stats.blocking_id;
    const bool lead = ob.id == out.stats.lead.id;
    // Blocking obstacle orange, vehicle ahead yellow, others red.
    markers.markers.push_back(box_marker(*box,
                                         blocking ? "blocking"
                                         : lead   ? "lead"
                                                  : "obstacles",
                                         1.0f,
                                         blocking ? 0.55f
                                         : lead   ? 0.85f
                                                  : 0.36f,
                                         0.36f));
    if (ob.prediction.size() < 2) continue;
    Marker path;
    path.header = markers.markers.back().header;
    path.ns = "predictions";
    path.id = id++;
    path.type = Marker::LINE_STRIP;
    path.scale.x = 0.15;
    path.color.r = 1.0f;
    path.color.g = 0.36f;
    path.color.b = 0.36f;
    path.color.a = 0.6f;
    for (const auto& p : ob.prediction) {
      if (p.t < 0.0) continue;
      geometry_msgs::msg::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.1;
      path.points.push_back(pt);
    }
    markers.markers.push_back(path);
  }
  markers_pub_->publish(markers);
}

}  // namespace frenet_planner_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(frenet_planner_ros2::PlannerNode)
