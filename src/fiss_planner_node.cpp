/** fiss_planner_node.cpp
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Local Planner ROS Node
 */

#include "fiss_planner/fiss_planner_node.h"

namespace fiss
{

// FissPlanner settings
FissPlanner::Setting SETTINGS = FissPlanner::Setting();

// Constants values used as thresholds (Not for tuning)
const double WP_MAX_SEP = 3.0;                                    // Maximum allowable waypoint separation
const double WP_MIN_SEP = 0.1;                                    // Minimum allowable waypoint separation
const double HEADING_DIFF_THRESH = M_PI/2;                        // Maximum allowed heading diff between vehicle and path
const double MAX_DIST_FROM_PATH = 10.0;                           // Maximum allowed distance between vehicle and path

/* List of dynamic parameters */
// Hyperparameters for output path
double TRAJ_MAX_SIZE;  // Maximum size of the output path
double TRAJ_MIN_SIZE;  // Minimum size of the output path

// Control Parameters
double PID_Kp, PID_Ki, PID_Kd;
// Stanley gains
int NUM_WP_LOOK_AHEAD;        // Number of waypoints to look ahead for Stanley
double STANLEY_OVERALL_GAIN;  // Stanley overall gain
double TRACK_ERROR_GAIN;      // Cross track error gain

// Safety margins for collision check
double LANE_WIDTH;
double LEFT_LANE_WIDTH;       // Maximum left road width [m]
double RIGHT_LANE_WIDTH;      // Maximum right road width [m]
// double NARROW_PATH_OFFSET = -0.3;

bool CHECK_COLLISION;
bool USE_ASYNC;
bool USE_HEURISTIC;

bool SETTINGS_UPDATED = false;

// Dynamic parameter server callback function
void dynamicParamCallback(fiss_planner::fiss_planner_Config& config, uint32_t level)
{
  // General Settings
  CHECK_COLLISION = config.check_collision;
  USE_ASYNC = config.use_async;
  USE_HEURISTIC = config.use_heuristic;
  SETTINGS.tick_t = config.tick_t;

  // Sampling parameters (lateral)
  LANE_WIDTH = config.curr_lane_width;
  LEFT_LANE_WIDTH = config.left_lane_width;
  RIGHT_LANE_WIDTH = config.right_lane_width;
  SETTINGS.center_offset = config.center_offset;
  SETTINGS.num_width = config.num_width;
  // Sampling parameters (longitudinal)
  SETTINGS.max_t = config.max_t;
  SETTINGS.min_t = config.min_t;
  SETTINGS.num_t = config.num_t;
  
  SETTINGS.highest_speed = kph2mps(config.highest_speed);
  SETTINGS.lowest_speed = kph2mps(config.lowest_speed);
  SETTINGS.num_speed = config.num_speed;
  // Constraints
  // SETTINGS.max_speed = Vehicle::max_speed();
  // SETTINGS.max_accel = Vehicle::max_acceleration();
  // SETTINGS.max_decel = Vehicle::max_deceleration();
  // SETTINGS.max_curvature = Vehicle::max_curvature_front();
  // SETTINGS.steering_angle_rate = Vehicle::max_steering_rate();
  SETTINGS.max_speed = kph2mps(config.max_speed);
  SETTINGS.max_accel = config.max_acceleration;
  SETTINGS.max_decel = -config.max_deceleration;
  SETTINGS.max_curvature = config.max_curvature;
  SETTINGS.max_jerk_s = config.max_jerk_lon;
  SETTINGS.max_jerk_d = config.max_jerk_lat;
  // Cost Weights
   SETTINGS.k_heuristic = config.k_heuristic;
  SETTINGS.k_diff = config.k_diff;
  SETTINGS.k_time = config.k_time;
  SETTINGS.k_jerk = config.k_jerk;
  SETTINGS.k_lat = config.k_lat;
  SETTINGS.k_lon = config.k_lon;
  // SETTINGS.k_obstacle = config.k_obstacle;
  // Safety constraints
  SETTINGS.vehicle_length = config.vehicle_length;
  SETTINGS.vehicle_width = config.vehicle_width;
  SETTINGS.safety_margin_lon = config.safety_margin_lon;
  SETTINGS.safety_margin_lat = config.safety_margin_lat;
  SETTINGS.safety_margin_soft = config.safety_margin_soft;
  // PID and Stanley gains
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  NUM_WP_LOOK_AHEAD = config.num_wp_look_ahead;
  STANLEY_OVERALL_GAIN = config.stanley_overall_gain;
  TRACK_ERROR_GAIN = config.track_error_gain;
  // Hyperparameters for output path
  TRAJ_MAX_SIZE = config.traj_max_size;
  TRAJ_MIN_SIZE = config.traj_min_size;

  SETTINGS_UPDATED = true;
}

// Constructor
FissPlannerNode::FissPlannerNode() : tf_listener(tf_buffer)
{
  // topics
  std::string odom_topic;
  std::string lane_info_topic;
  std::string obstacles_topic;

  std::string ref_path_topic;
  std::string curr_traj_topic;
  std::string next_traj_topic;
  std::string sample_space_topic;
  std::string final_traj_topic;
  std::string candidate_trajs_topic;
  std::string vehicle_cmd_topic;

  ros::NodeHandle private_nh("~");
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  // Hyperparameters from launch file
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic));
  ROS_ASSERT(private_nh.getParam("lane_info_topic", lane_info_topic));
  ROS_ASSERT(private_nh.getParam("obstacles_topic", obstacles_topic));
  ROS_ASSERT(private_nh.getParam("ref_path_topic", ref_path_topic));
  ROS_ASSERT(private_nh.getParam("curr_traj_topic", curr_traj_topic));
  ROS_ASSERT(private_nh.getParam("next_traj_topic", next_traj_topic));
  ROS_ASSERT(private_nh.getParam("sample_space_topic", sample_space_topic));
  ROS_ASSERT(private_nh.getParam("final_traj_topic", final_traj_topic));
  ROS_ASSERT(private_nh.getParam("candidate_trajs_topic", candidate_trajs_topic));
  ROS_ASSERT(private_nh.getParam("vehicle_cmd_topic", vehicle_cmd_topic));

  // Instantiate FissPlanner
  frenet_planner_ = FissPlanner(SETTINGS);

  // Subscribe & Advertise
  odom_sub = nh.subscribe(odom_topic, 1, &FissPlannerNode::odomCallback, this);
  lane_info_sub = nh.subscribe(lane_info_topic, 1, &FissPlannerNode::laneInfoCallback, this);
  obstacles_sub = nh.subscribe(obstacles_topic, 1, &FissPlannerNode::obstaclesCallback, this);
  
  ref_path_pub = nh.advertise<nav_msgs::Path>(ref_path_topic, 1);
  curr_traj_pub = nh.advertise<nav_msgs::Path>(curr_traj_topic, 1);
  next_traj_pub = nh.advertise<nav_msgs::Path>(next_traj_topic, 1);

  sample_space_pub = nh.advertise<visualization_msgs::Marker>(sample_space_topic, 1);
  final_traj_pub = nh.advertise<visualization_msgs::Marker>(final_traj_topic, 1);
  candidate_paths_pub = nh.advertise<visualization_msgs::MarkerArray>(candidate_trajs_topic, 1);

  vehicle_cmd_pub = nh.advertise<autoware_msgs::VehicleCmd>(vehicle_cmd_topic, 1);
  obstacles_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("fiss_planner/objects", 1);

  run_time_pub = nh.advertise<std_msgs::Float32>("fiss_planner/run_time", 1);
  freqency_pub = nh.advertise<std_msgs::Float32>("fiss_planner/freqency", 1);
  iteration_pub = nh.advertise<std_msgs::Float32>("fiss_planner/iteration", 1);
  cost_pub = nh.advertise<std_msgs::Float32>("fiss_planner/cost", 1);

  // Initializing states
  regenerate_flag_ = false;
  target_lane_id_ = LaneID::CURR_LANE;
  pid_ = control::PID(0.1, Vehicle::max_acceleration(), Vehicle::max_deceleration(), PID_Kp, PID_Ki, PID_Kd);
};

void FissPlannerNode::laneInfoCallback(const nav_msgs::Path::ConstPtr& global_path)
{
  lane_ = Lane(global_path, LANE_WIDTH/2, LANE_WIDTH/2, LANE_WIDTH/2 + LEFT_LANE_WIDTH, LANE_WIDTH/2 + RIGHT_LANE_WIDTH);
  ROS_INFO("Local Planner: Lane Info Received, with %d points, filtered to %d points", int(lane_.points.size()), int(lane_.points.size()));
}

// Update vehicle current state from the tf transform
void FissPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  current_state_.v = magnitude(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", odom_msg->header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  geometry_msgs::Pose pose_in_map;
  tf2::doTransform(odom_msg->pose.pose, pose_in_map, transform_stamped);
  // Current XY of robot (map frame)
  current_state_.x = pose_in_map.position.x;
  current_state_.y = pose_in_map.position.y;
  map_height_ = pose_in_map.position.z - 0.3; // minus the tire radius

  tf2::Quaternion q_tf2(pose_in_map.orientation.x, pose_in_map.orientation.y,
                        pose_in_map.orientation.z, pose_in_map.orientation.w);
  tf2::Matrix3x3 m(q_tf2.normalize());
  double roll, pitch;
  m.getRPY(roll, pitch, current_state_.yaw);
}

void FissPlannerNode::obstaclesCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_obstacles)
{
  // Start a timing for the main algorithm
  const auto start_time = std::chrono::high_resolution_clock::now();
  
  auto obstacles = boost::make_shared<autoware_msgs::DetectedObjectArray>();
  transformObjects(*obstacles, *input_obstacles);
  obstacles_pub.publish(*obstacles);

  // Check if all required data are in position
  if (obstacles->objects.empty())
  {
    ROS_WARN("Local Planner: No obstacles received");
    // publishEmptyTrajsAndStop();
    // return;
  }
  if (SETTINGS_UPDATED)
  {
    frenet_planner_.updateSettings(SETTINGS);
    SETTINGS_UPDATED = false;
  }
    
  ROS_INFO("Local Planner: Planning Start");

  // Update Waypoints
  if (!feedWaypoints())
  {
    ROS_WARN("Local Planner: Waiting for Waypoints");
    publishEmptyTrajsAndStop();
    return;
  }

  // Update start state
  updateStartState();
  // Get the reference lane's centerline as a spline
  auto ref_path_and_curve = frenet_planner_.generateReferenceCurve(local_lane_);
  // Store the results into reference spline
  ref_spline_ = std::move(ref_path_and_curve.first);

  if (ref_spline_.x.empty())
  {
    ROS_ERROR("Local Planner: Reference Curve could not be be generated, No path generated");
    publishEmptyTrajsAndStop();
    return;
  }
  ROS_INFO("Local Planner: Reference Curve Generated");

  // Define ROI width for path sampling
  roi_boundaries_ = getSamplingWidthFromTargetLane(target_lane_id_, SETTINGS.vehicle_width, LANE_WIDTH, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH);

  // Get the planning result 
  const auto planning_results = frenet_planner_.frenetOptimalPlanning(ref_path_and_curve.second, start_state_, target_lane_id_, 
                                                                                      roi_boundaries_[0], roi_boundaries_[1], current_state_.v, 
                                                                                      *obstacles, CHECK_COLLISION, USE_ASYNC, USE_HEURISTIC);
  std::vector<FrenetPath> best_traj_list = planning_results.first;
  publishStats(planning_results.second);
  
  ROS_INFO("Local Planner: Frenet Optimal Planning Done");
  publishCandidateTrajs(frenet_planner_.all_trajs_);
  if (best_traj_list.empty())
  {
    ROS_ERROR("Local Planner: Frenet Optimal Planning Could Not Find a Safe Trajectory");
    publishEmptyTrajsAndStop();
    return;
  }

  // Find the best path from the all candidates 
  FrenetPath best_traj = selectLane(best_traj_list, current_lane_id_);
  ROS_INFO("Local Planner: Best trajs Selected");
  publishSampleSpace(ref_spline_);
  publishVisTraj(curr_trajectory_, best_traj);

  // Concatenate the best path into output_path
  concatPath(best_traj, TRAJ_MAX_SIZE, TRAJ_MIN_SIZE, WP_MAX_SEP, WP_MIN_SEP);

  // Publish the best trajs
  publishRefSpline(ref_spline_);
  publishCurrTraj(curr_trajectory_);
  publishNextTraj(best_traj);

  const auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
  ROS_INFO("Local Planner: Planning took %f ms, (or %f Hz)", elapsed_time.count(), 1000/elapsed_time.count());
}

void FissPlannerNode::transformObjects(autoware_msgs::DetectedObjectArray& output_objects, const autoware_msgs::DetectedObjectArray& input_objects)
{
  output_objects.header = input_objects.header;
  output_objects.header.stamp = ros::Time::now();
  output_objects.header.frame_id = "map";
  output_objects.objects.clear();

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", input_objects.header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  for (auto const& object : input_objects.objects)
  {
    geometry_msgs::Pose pose_transformed;
    tf2::doTransform(object.pose, pose_transformed, transform_stamped);
    
    geometry_msgs::Polygon polygon_transformed;
    for (auto const& point : object.convex_hull.polygon.points)
    {
      geometry_msgs::Point coord_before_transform, coord_after_transform;
      coord_before_transform.x = point.x;
      coord_before_transform.y = point.y;
      coord_before_transform.z = point.z;

      tf2::doTransform(coord_before_transform, coord_after_transform, transform_stamped);

      geometry_msgs::Point32 transformed_point;
      transformed_point.x = coord_after_transform.x;
      transformed_point.y = coord_after_transform.y;
      transformed_point.z = coord_after_transform.z;
      polygon_transformed.points.emplace_back(transformed_point);
    }

    autoware_msgs::DetectedObject transformed_object = object;
    transformed_object.header.frame_id = "map";
    transformed_object.pose = pose_transformed;
    transformed_object.convex_hull.polygon = polygon_transformed;
    transformed_object.convex_hull.header.frame_id = "map";
    
    output_objects.objects.emplace_back(std::move(transformed_object));
  }
}

// Publish the reference spline (for Rviz only)
void FissPlannerNode::publishRefSpline(const Path& path)
{
  nav_msgs::Path ref_path_msg;
  ref_path_msg.header.stamp = ros::Time::now();
  ref_path_msg.header.frame_id = "map";

  for (size_t i = 0; i < path.yaw.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = ref_path_msg.header;
    pose.pose.position.x = path.x[i];
    pose.pose.position.y = path.y[i];
    pose.pose.position.z = map_height_;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(path.yaw[i]);
    ref_path_msg.poses.emplace_back(pose);
  }

  ref_path_pub.publish(ref_path_msg);
}

// Publish the current path (for Rviz and MPC)
void FissPlannerNode::publishCurrTraj(const Path& path)
{
  nav_msgs::Path curr_trajectory_msg;
  curr_trajectory_msg.header.stamp = ros::Time::now();
  curr_trajectory_msg.header.frame_id = "map";

  for (size_t i = 0; i < path.yaw.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = curr_trajectory_msg.header;
    pose.pose.position.x = path.x[i];
    pose.pose.position.y = path.y[i];
    pose.pose.position.z = map_height_ + 2.0*path.v[i]/Vehicle::max_speed();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(path.yaw[i]);
    curr_trajectory_msg.poses.emplace_back(pose);
  }

  curr_traj_pub.publish(curr_trajectory_msg);
}

// Publish the best next path (for Rviz only)
void FissPlannerNode::publishNextTraj(const FrenetPath& next_traj)
{
  nav_msgs::Path curr_trajectory_msg;
  curr_trajectory_msg.header.stamp = ros::Time::now();
  curr_trajectory_msg.header.frame_id = "map";

  for (size_t i = 0; i < next_traj.c.size(); i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = curr_trajectory_msg.header;
    pose.pose.position.x = next_traj.x[i];
    pose.pose.position.y = next_traj.y[i];
    pose.pose.position.z = map_height_ + 2.0*std::hypot(next_traj.s_d[i], next_traj.d_d[i])/Vehicle::max_speed();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_traj.yaw[i]);
    curr_trajectory_msg.poses.emplace_back(pose);
  }

  next_traj_pub.publish(curr_trajectory_msg);
}

void FissPlannerNode::publishSampleSpace(const Path& ref_path)
{
  int marker_id = 0;
  const auto sample_space_marker = CollisionDetectorVisualization::visualizePredictedTrajectory(
    ref_path, SETTINGS.vehicle_width, 1.5, map_height_, current_state_, false, marker_id, "space", Visualization::COLOR::YELLOW, 0.1);
  sample_space_pub.publish(sample_space_marker);
}

void FissPlannerNode::publishVisTraj(const Path& current_traj, const FrenetPath& next_traj)
{
  int marker_id = 0;
  Path vis_traj = current_traj;
  // for (int i = 0; i < next_traj.x.size(); i++)
  // {
  //   if (std::isnormal(next_traj.x[i]) && std::isnormal(next_traj.y[i]) && std::isnormal(next_traj.yaw[i]))
  //   {
  //     vis_traj.x.push_back(next_traj.x[i]);
  //     vis_traj.y.push_back(next_traj.y[i]);
  //     vis_traj.yaw.push_back(next_traj.yaw[i]);
  //   }
  // }

  const auto output_traj_marker = CollisionDetectorVisualization::visualizePredictedTrajectory(
                                    vis_traj, SETTINGS.vehicle_width, 0.0, map_height_, current_state_, true, 
                                    marker_id, "final", Visualization::COLOR::GREEN, 0.15);
  final_traj_pub.publish(output_traj_marker);
}

/**
 * @brief publish candidate trajs for visualization in rviz
 * 
 */
void FissPlannerNode::publishCandidateTrajs(const std::vector<FrenetPath>& candidate_trajs)
{
  const auto candidate_paths_markers = LocalPlannerVisualization::visualizeCandidateTrajs(candidate_trajs, map_height_, Vehicle::max_speed());
  candidate_paths_pub.publish(std::move(candidate_paths_markers));
}

void FissPlannerNode::publishStats(std::vector<double> stats)
{
  std_msgs::Float32 run_time, frequency, iteration, cost;
  run_time.data = stats[0];
  frequency.data = stats[1];
  iteration.data = stats[2];
  cost.data = stats[3];
  run_time_pub.publish(run_time);
  freqency_pub.publish(frequency);
  iteration_pub.publish(iteration);
  cost_pub.publish(cost);
}

// Publish empty trajs (for Rviz only)
void FissPlannerNode::publishEmptyTrajsAndStop()
{
  // Publish empty trajs
  publishRefSpline(Path());
  publishCurrTraj(Path());
  publishNextTraj(FrenetPath());
  publishVehicleCmd(-1.0, 0.0);
}

// Update the vehicle front axle state (used in odomcallback)
void FissPlannerNode::updateVehicleFrontAxleState()
{
  // Current XY of robot (map frame)
  frontaxle_state_.x = current_state_.x + (Vehicle::L() * std::cos(current_state_.yaw));
  frontaxle_state_.y = current_state_.y + (Vehicle::L() * std::sin(current_state_.yaw));
  frontaxle_state_.yaw = current_state_.yaw;
  frontaxle_state_.v = current_state_.v;
}

// Feed map waypoints into local map
bool FissPlannerNode::feedWaypoints()
{
  if (lane_.points.empty())
  {
    ROS_WARN("Local Planner: Waiting for Lane Points");
    return false;
  }
  else if (lane_.points.size() < 5)
  {
    ROS_WARN("Local Planner: Global Path has less than 5 points, unable to plan");
    return false;
  }

  int start_id = lastWaypoint(current_state_, lane_);

  // if reached the end of the lane, stop
  if (start_id >= lane_.points.size() - 2)  // exclude last 2 waypoints for safety, and prevent code crashing
  {
    // ROS_WARN("Local Planner: Vehicle is at waypoint no.%d, with %d waypoints in total", start_id, int(lane_.points.size()));
    ROS_WARN("Local Planner: Vehicle has reached the destination");
    return false;
  }

  const double dist = distance(lane_.points[start_id].point.x, lane_.points[start_id].point.y, current_state_.x, current_state_.y);
  const double heading_diff = unifyAngleRange(current_state_.yaw - lane_.points[start_id].point.yaw);

  if (dist > MAX_DIST_FROM_PATH)
  {
    ROS_WARN("Local Planner: Vehicle's Location Is Too Far From The Target Lane");
    return false;
  }
  else if (std::abs(heading_diff) > HEADING_DIFF_THRESH)
  {
    ROS_WARN("Local Planner: Vehicle's Is Heading In A Different Direction");
    return false;
  }

  // clear the old waypoints
  local_lane_.clear(); 

  // Make sure there are at least 5 points in the remaining section of the global reference path
  if (start_id > lane_.points.size() - 5)
  {
    start_id = lane_.points.size() - 5;
  }

  // Check if the global waypoints need to be filtered
  const double ref_spline_length = SETTINGS.highest_speed*(SETTINGS.max_t);
  if ((lane_.points.back().point.s - lane_.points[start_id].point.s) >= ref_spline_length)
  {
    // Filter the waypoints to a uniform density
    double s_current = lane_.points[start_id].point.s;
    local_lane_.points.push_back(lane_.points[start_id]);
    for (size_t i = start_id + 1; i < lane_.points.size(); i++)
    {
      if (local_lane_.points.size() >= 5)
      {
        break;
      }
      else if ((lane_.points[i].point.s - s_current) >= ref_spline_length/5.0)
      {
        s_current = lane_.points[i].point.s;
        local_lane_.points.push_back(lane_.points[i]);
      }
    }
    ROS_INFO("Local Planner: Filtered the global path from %d to %d points", int(lane_.points.size()), int(local_lane_.points.size()));
  }
  else
  {
    // feed the new waypoints
    ROS_INFO("Local Planner: Global reference path only has %f meters left", lane_.points.back().point.s - lane_.points[start_id].point.s);
    if ((lane_.points.size() - start_id) >= 5)
    {
      const int first_id = start_id;                            // 0
      const int fifth_id = lane_.points.size() - 1;             // 4
      const int third_id = (first_id + fifth_id)/2;             // 2
      const int second_id = (first_id + third_id)/2;            // 1
      const int fourth_id = (third_id + fifth_id)/2;            // 3

      local_lane_.points.push_back(lane_.points[first_id]);
      local_lane_.points.push_back(lane_.points[second_id]);
      local_lane_.points.push_back(lane_.points[third_id]);
      local_lane_.points.push_back(lane_.points[fourth_id]);
      local_lane_.points.push_back(lane_.points[fifth_id]);
    }
    else
    {
      ROS_WARN("Local Planner: Global reference path only has %d points left, stopped planning!", int(lane_.points.size() - start_id));
      return false;
    }
  }

  return true;
}

// Update the vehicle start state in frenet
void FissPlannerNode::updateStartState()
{
  // Ensure the reference spline exists
  if (local_lane_.points.empty())
  {
    return;
  }

  // if the current path size is too small, regenerate
  // if (curr_trajectory_.x.size() < TRAJ_MIN_SIZE)
  // {
  //   regenerate_flag_ = true;
  // }

  // // if need to regenerate the entire path
  // if (regenerate_flag_)
  // {
  //   ROS_INFO("Local Planner: Regenerating The Entire Path...");
  //   // Update the starting state in frenet (using ref_spline_ can produce a finer result compared to local_lane_, but
  //   // at fringe cases, such as start of code, ref spline might not be available
  //   start_state_ = ref_spline_.yaw.empty() ? getFrenet(current_state_, local_lane_) : getFrenet(current_state_, ref_spline_);

  //   // Clear the last output path
  //   curr_trajectory_.clear();
  //   regenerate_flag_ = false;
  // }
  // if not regenerating
  // else
  // {
  //   ROS_INFO("Local Planner: Continuing From The Previous Path...");

  //   // End of the previous path speed
  //   const double curr_trajectory_last_speed = hypot(curr_trajectory_.x.back() - curr_trajectory_.x.end()[-2], curr_trajectory_.y.back() - curr_trajectory_.y.end()[-2]) / SETTINGS.tick_t;
  //   // End of the previous path state
  //   VehicleState last_state = VehicleState(curr_trajectory_.x.back(), curr_trajectory_.y.back(), curr_trajectory_.yaw.back(), curr_trajectory_.v.back());

  //   start_state_ = ref_spline_.yaw.empty() ? getFrenet(last_state, local_lane_) : getFrenet(last_state, ref_spline_);
  // }

  start_state_ = ref_spline_.yaw.empty() ? getFrenet(current_state_, local_lane_) : getFrenet(current_state_, ref_spline_);

  // Ensure the speed is above the minimum planning speed
  start_state_.s_d = std::max(start_state_.s_d, 1.0);

  // Update current lane
  if (std::abs(start_state_.d) <= LANE_WIDTH/2)
  {
    current_lane_id_ = LaneID::CURR_LANE;
  }
  else if (start_state_.d > LANE_WIDTH/2)
  {
    current_lane_id_ = LaneID::LEFT_LANE;
  }
  else if (start_state_.d < -LANE_WIDTH/2)
  {
    current_lane_id_ = LaneID::RIGHT_LANE;
  }
  else
  {
    current_lane_id_ = -1;
    ROS_WARN("Vehicle's lateral position is %f, too far off", start_state_.d);
  }
}

// Calculate the sampling width for the planner
std::vector<double> FissPlannerNode::getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width, const double current_lane_width,
                                                                    const double left_lane_width, const double right_lane_width)
{
  double left_bound, right_bound;

  switch (lane_id)
  {
    // all lanes
    case LaneID::ALL_LANES:
      left_bound = current_lane_width/2 + left_lane_width;
      right_bound = -current_lane_width/2 - right_lane_width;
      ROS_INFO("Local Planner: Sampling On ALL Lanes");
      break;

    // stay within the current lane
    case LaneID::CURR_LANE:
      left_bound = current_lane_width/2;
      right_bound = -current_lane_width/2;
      ROS_INFO("Local Planner: Sampling On The Current Lane");
      break;

    // change to left lane
    case LaneID::LEFT_LANE:
      left_bound = current_lane_width/2 + left_lane_width;
      right_bound = current_lane_width/2;
      ROS_INFO("Local Planner: Sampling On The Left Lane");
      break;
    // change to right lane
    case LaneID::RIGHT_LANE:
      left_bound = -current_lane_width/2;
      right_bound = -current_lane_width/2 - right_lane_width;
      ROS_INFO("Local Planner: Sampling On The Right Lane");
      break;
  }

  return {left_bound - vehicle_width/2, right_bound + vehicle_width/2};
}

// Select the ideal lane to proceed
FrenetPath FissPlannerNode::selectLane(const std::vector<FrenetPath>& best_traj_list, const int current_lane)
{
  FrenetPath best_traj;
  bool change_lane_flag;
  int keep_lane_id = -1;
  int change_lane_id = -1;
  double keep_lane_cost = std::numeric_limits<double>::max();
  double change_lane_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < best_traj_list.size(); i++)
  {
    if (!best_traj_list[i].x.empty())
    {
      // keep lane option
      if (best_traj_list[i].lane_id == current_lane || best_traj_list[i].lane_id == 0)
      {
        if (best_traj_list[i].final_cost < keep_lane_cost)
        {
          keep_lane_id = i;
          keep_lane_cost = best_traj_list[i].final_cost;
        }
      }
      // change lane option
      else
      {
        change_lane_id = i;
        change_lane_cost = best_traj_list[i].final_cost;
      }
    }
  }

  // if both lanes available
  if (keep_lane_id != -1 && change_lane_id != -1)
  {
    if (keep_lane_cost <= change_lane_cost)
    {
      ROS_INFO("Local Planner: Keeping Lane");
      change_lane_flag = false;
      best_traj = best_traj_list[keep_lane_id];
    }
    else
    {
      ROS_INFO("Local Planner: Changing Lane");
      change_lane_flag = true;
      best_traj = best_traj_list[change_lane_id];
    }
  }
  // if only keep lane available
  else if (keep_lane_id != -1 && change_lane_id == -1)
  {
    ROS_INFO("Local Planner: Keeping Lane");
    change_lane_flag = false;
    best_traj = best_traj_list[keep_lane_id];
  }
  // if only change lane available
  else if (keep_lane_id == -1 && change_lane_id != -1)
  {
    ROS_INFO("Local Planner: Changing Lane");
    change_lane_flag = true;
    best_traj = best_traj_list[change_lane_id];
  }
  // if none available
  else
  {
    ROS_INFO("Local Planner: No Path Available");
    change_lane_flag = false;
    // dummy path
    best_traj = FrenetPath();
  }

  return best_traj;
}

// Concatenate the best next path to the current path
void FissPlannerNode::concatPath(const FrenetPath& next_traj, const int traj_max_size, const int traj_min_size, const double wp_max_seperation, const double wp_min_seperation)
{
  // size_t diff = 0;
  // if (curr_trajectory_.x.size() <= traj_min_size)
  // {
  //   diff = std::min(traj_max_size - curr_trajectory_.x.size(), next_traj.x.size());
  // }

  // // Concatenate the best path to the output path
  curr_trajectory_.clear();
  const size_t N = std::min(next_traj.x.size(), next_traj.yaw.size());
  for (size_t i = 0; i < N; i++)
  {
    if (!std::isnormal(next_traj.x[i]) || !std::isnormal(next_traj.y[i]) || !std::isnormal(next_traj.yaw[i]) || 
        !std::isnormal(next_traj.s_d[i]) || !std::isnormal(next_traj.d_d[i]))
    {
      break;
    }
    else
    {
      curr_trajectory_.x.push_back(next_traj.x[i]);
      curr_trajectory_.y.push_back(next_traj.y[i]);
      curr_trajectory_.yaw.push_back(next_traj.yaw[i]);
      curr_trajectory_.v.push_back(std::hypot(next_traj.s_d[i], next_traj.d_d[i]));
    }
  }

  // Calculate control outputs and Erase the point that have been executed
  if (!curr_trajectory_.x.empty() && !curr_trajectory_.y.empty())
  {
    // Calculate steering angle
    updateVehicleFrontAxleState();
    const int next_frontlink_wp_id = nextWaypoint(frontaxle_state_, curr_trajectory_);
    // Calculate Control Outputs
    if (calculateControlOutput(next_frontlink_wp_id, frontaxle_state_))
    {
      // Publish steering angle
      publishVehicleCmd(acceleration_, steering_angle_);
    }
    else
    {
      ROS_ERROR("Local Planner: No output steering angle");
      publishVehicleCmd(-1.0, 0.0); // Publish empty control output
    }

    // const int next_wp_id = nextWaypoint(current_state_, curr_trajectory_);

    // for (size_t i = 0; i < next_wp_id; i++)
    // {
    //   curr_trajectory_.x.erase(curr_trajectory_.x.begin());
    //   curr_trajectory_.y.erase(curr_trajectory_.y.begin());
    //   curr_trajectory_.yaw.erase(curr_trajectory_.yaw.begin());
    //   curr_trajectory_.v.erase(curr_trajectory_.v.begin());
    // }
  }
  else
  {
    ROS_ERROR("Local Planner: Output Path is Empty, No Steering Angle");
    publishVehicleCmd(-1.0, 0.0); // Publish empty control output
  }
}

// Steering Help Function
bool FissPlannerNode::calculateControlOutput(const int next_wp_id, const VehicleState& frontaxle_state)
{
  const double wp_id = next_wp_id + NUM_WP_LOOK_AHEAD;

  // If the current path is too short, return error value
  if (curr_trajectory_.x.size() <= wp_id)
  {
    ROS_ERROR("Local Planner: Output Path Too Short! No output steering angle");
    // std::cout << "Output Path Size: " << curr_trajectory_.x.size() << " Required Size: " << wp_id + 2 << std::endl;
    regenerate_flag_ = true;
    return false;
  }
  else
  {
    // First Term
    const double delta_yaw = unifyAngleRange(curr_trajectory_.yaw[wp_id] - current_state_.yaw);

    // Second Term
    const double c = distance(curr_trajectory_.x[wp_id], curr_trajectory_.y[wp_id], curr_trajectory_.x[wp_id+1], curr_trajectory_.y[wp_id+1]);
    // if two waypoints overlapped, return error value
    if (c <= WP_MIN_SEP)
    {
      ROS_WARN("Local Planner: two points overlapped, Regenerate");
      regenerate_flag_ = true;
      return false;
    }
    const double a = distance(frontaxle_state.x, frontaxle_state.y, curr_trajectory_.x[wp_id], curr_trajectory_.y[wp_id]);
    const double b = distance(frontaxle_state.x, frontaxle_state.y, curr_trajectory_.x[wp_id+1], curr_trajectory_.y[wp_id+1]);
    // if the vehicle is too far from the waypoint, return error value
    if (a >= MAX_DIST_FROM_PATH || b >= MAX_DIST_FROM_PATH)
    {
      ROS_WARN("Local Planner: Vehicle is too far from the path, Regenerate");
      regenerate_flag_ = true;
      return false;
    }

    const double p = (a + b + c) / 2.0;
    const double triangle_area = sqrt(p * (p - a) * (p - b) * (p - c));
    const double x = triangle_area * 2.0 / c;
    const double u = std::max(1.0, current_state_.v);

    // Angle of vector vehicle -> waypoint
    const double vectors_angle_diff = atan2(frontaxle_state.y - curr_trajectory_.y[wp_id], frontaxle_state.x - curr_trajectory_.x[wp_id]) - curr_trajectory_.yaw[wp_id];
    const double vectors_angle_diff_unified = unifyAngleRange(vectors_angle_diff);
    const int direction = vectors_angle_diff_unified < 0 ? 1 : -1;

    // Final Angle
    steering_angle_ = STANLEY_OVERALL_GAIN * (delta_yaw + direction * atan(TRACK_ERROR_GAIN * x / u));
    // Check if exceeding max steering angle
    steering_angle_ = limitWithinRange(steering_angle_, -Vehicle::max_steering_angle(), Vehicle::max_steering_angle());

    // Calculate accelerator output
    acceleration_ = pid_.calculate(curr_trajectory_.v[wp_id], current_state_.v);

    ROS_INFO("Controller: Traget Speed: %2f, Current Speed: %2f, Acceleration: %.2f ", curr_trajectory_.v[wp_id], current_state_.v, acceleration_);
    ROS_INFO("Controller: Cross Track Error: %2f, Yaw Diff: %2f, SteeringAngle: %.2f ", direction*x, rad2deg(delta_yaw), rad2deg(steering_angle_));
    return true;
  }
}

// Publish the resulted steering angle (Stanley)
void FissPlannerNode::publishVehicleCmd(const double accel, const double angle)
{
  autoware_msgs::VehicleCmd vehicle_cmd;
  vehicle_cmd.twist_cmd.twist.linear.x = accel/Vehicle::max_acceleration();  // [pct]
  vehicle_cmd.twist_cmd.twist.angular.z = angle;                             // [rad]
  vehicle_cmd.gear_cmd.gear = autoware_msgs::Gear::DRIVE;
  vehicle_cmd_pub.publish(vehicle_cmd);
}

} // namespace fiss

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fiss_planner_node");
  fiss::FissPlannerNode fiss_planner_node;
  ros::spin();  // spin the ros node.
  return 0;
}