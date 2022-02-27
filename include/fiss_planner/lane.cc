/** lane.cc
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * Helper functions related to lanes
 */

/** frenet.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of Waypoints and Lanes
 */

#include "lane.h"

namespace fiss
{

Waypoint::Waypoint() {}
Waypoint::Waypoint(const double x, const double y, const double yaw)
  : x(x), y(y), yaw(yaw), s(0.0) {}
Waypoint::Waypoint(const double x, const double y, const double yaw, const double s)
  : x(x), y(y), yaw(yaw), s(s) {}
Waypoint::Waypoint(const tf::Pose& pose)
{
  this->x = pose.getOrigin().x();
  this->y = pose.getOrigin().y();
  this->yaw = tf::getYaw(pose.getRotation());
  this->s = 0.0;
}
Waypoint::Waypoint(const geometry_msgs::Pose& pose_msg)
{
  tf::Pose pose;
  tf::poseMsgToTF(pose_msg, pose);
  this->x = pose.getOrigin().x();
  this->y = pose.getOrigin().y();
  this->yaw = tf::getYaw(pose.getRotation());
  this->s = 0.0;
}
Waypoint::Waypoint(const geometry_msgs::Pose& pose_msg, const double s)
{
  tf::Pose pose;
  tf::poseMsgToTF(pose_msg, pose);
  this->x = pose.getOrigin().x();
  this->y = pose.getOrigin().y();
  this->yaw = tf::getYaw(pose.getRotation());
  this->s = s;
}

LanePoint::LanePoint() {};
LanePoint::LanePoint(const Waypoint& point, const double left_width, const double right_width, const double far_left_width, const double far_right_width)
  : point(point), left_width(left_width), right_width(right_width), far_left_width(far_left_width), far_right_width(far_right_width) {}
LanePoint::LanePoint(const geometry_msgs::Pose& pose, const double left_width, const double right_width, const double far_left_width, const double far_right_width, const double s)
  : left_width(left_width), right_width(right_width), far_left_width(far_left_width), far_right_width(far_right_width)
{
  this->point = Waypoint(pose, s);
}

Lane::Lane(const nav_msgs::Path::ConstPtr& ref_path, const double left_width, const double right_width, const double far_left_width, const double far_right_width)
{
  double s_total = 0.0;
  this->points.emplace_back(LanePoint(ref_path->poses[0].pose, left_width, right_width, far_left_width, far_right_width, s_total));
  for (size_t i = 1; i < ref_path->poses.size(); i++)
  {
    const double dist = distance(ref_path->poses[i-1].pose, ref_path->poses[i].pose);
    if (dist >= 0.01) // make sure there is no duplicated waypoints
    {
      s_total += dist;
      this->points.emplace_back(LanePoint(ref_path->poses[i].pose, left_width, right_width, far_left_width, far_right_width, s_total));
    }
  }
}

void Lane::clear()
{
  points.clear();
}

void Path::clear()
{
  x.clear();
  y.clear();
  yaw.clear();
  v.clear();
}

int closestWaypoint(VehicleState current_state, const Path& path)
{
  double closest_dist = 100000.0;  // start with a large number
  int closest_waypoint = 0;

  for (int i = 0; i < path.x.size(); i++)
  {
    double dist = distance(current_state.x, current_state.y, path.x[i], path.y[i]);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

int closestWaypoint(VehicleState current_state, const Lane& lane)
{
  double closest_dist = 100000.0;  // start with a large number
  int closest_waypoint = 0;

  for (int i = 0; i < lane.points.size(); i++)
  {
    double dist = distance(current_state.x, current_state.y, lane.points[i].point.x, lane.points[i].point.y);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

int nextWaypoint(VehicleState current_state, const Path& path)
{
  int closest_waypoint = closestWaypoint(current_state, path);
  double heading = atan2((path.y[closest_waypoint] - current_state.y), (path.x[closest_waypoint] - current_state.x));

  double angle = fabs(current_state.yaw - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    closest_waypoint++;
  }

  return closest_waypoint;
}

int nextWaypoint(VehicleState current_state, const Lane& lane)
{
  int closest_waypoint = closestWaypoint(current_state, lane);
  double heading = atan2((lane.points[closest_waypoint].point.y - current_state.y), (lane.points[closest_waypoint].point.x - current_state.x));

  double angle = fabs(current_state.yaw - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2)
  {
    closest_waypoint++;
  }

  return closest_waypoint;
}

int lastWaypoint(VehicleState current_state, const Path& path)
{
  return nextWaypoint(current_state, path) - 1;
}

int lastWaypoint(VehicleState current_state, const Lane& lane)
{
  return nextWaypoint(current_state, lane) - 1;
}

}  // end of namespace fiss