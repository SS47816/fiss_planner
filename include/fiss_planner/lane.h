/** frenet.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Defination of Waypoints and Lanes
 */

#ifndef LANE_H_
#define LANE_H_

#include <vector>

#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

#include "math_utils.h"
#include "vehicle_state.h"

namespace fiss
{

// enum class WaypointType
// {
//   NORMAL,
//   PATH_END,
//   TURN,
//   SLOPE,
//   OFFSET_START,
//   OFFSET_END
// };

// Class for lane id
enum LaneID
{
  RIGHT_LANE,
  CURR_LANE,
  LEFT_LANE,
  ALL_LANES,
};

struct Waypoint
{
 public:
  Waypoint();
  Waypoint(const double x, const double y, const double yaw);
  Waypoint(const double x, const double y, const double yaw, const double s);
  Waypoint(const tf::Pose& pose);
  Waypoint(const geometry_msgs::Pose& pose_msg);
  Waypoint(const geometry_msgs::Pose& pose_msg, const double s);

  double x, y, yaw, s;
};

struct LanePoint
{
 public:
  LanePoint();
  LanePoint(const Waypoint& point, const double left_width, const double right_width, const double far_left_width, const double far_right_width);
  LanePoint(const geometry_msgs::Pose& pose, const double left_width, const double right_width, const double far_left_width, const double far_right_width, const double s);
  
  double left_width, right_width;
  double far_left_width, far_right_width;
  Waypoint point;
};
  
class Lane
{
 public:
  // Constructors
  Lane(){};
  Lane(const nav_msgs::Path::ConstPtr& ref_path, const double left_width, const double right_width, const double far_left_width, const double far_right_width);
  Lane(const geometry_msgs::PoseArray::ConstPtr& ref_path, const double left_width, const double right_width, const double far_left_width, const double far_right_width);
  
  // Destructor
  virtual ~Lane() {};
  // Clear all contents
  void clear();

  std::vector<LanePoint> points;
};

class Path
{
 public:
  // Constructor
  Path(){};
  // Destructor
  virtual ~Path(){};
  // Clear all contents
  void clear();

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> v;
};

// Find the ID of the closest waypoint wrt current x, y position
int closestWaypoint(VehicleState current_state, const Path& path);
int closestWaypoint(VehicleState current_state, const Lane& map);

// Find the ID of the next waypoint of the closest waypoint wrt current x, y position
int nextWaypoint(VehicleState current_state, const Path& path);
int nextWaypoint(VehicleState current_state, const Lane& map);

// Find the ID of the previous waypoint
int lastWaypoint(VehicleState current_state, const Path& path);
int lastWaypoint(VehicleState current_state, const Lane& map);

}  // end of namespace fiss

#endif // LANE_H_