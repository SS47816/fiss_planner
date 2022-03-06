/** frenet.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Construction of frenet coordinates
 * Conversion between Frenet frame and Cartesian frame
 */

#include "frenet.h"

namespace fiss
{

FrenetPath::FrenetPath() {}
FrenetPath::FrenetPath(const int lane_id, FrenetState& end_state, const double fix_cost, const double heu_cost)
 : 
  lane_id(lane_id), 
  is_generated(false),
  is_searched(false),
  constraint_passed(false),
  collision_passed(false),
  fix_cost(fix_cost),
  dyn_cost(0.0),
  heu_cost(heu_cost),
  est_cost(fix_cost + heu_cost),
  final_cost(0.0),
  end_state(end_state)
{}

bool operator < (const FrenetPath& lhs, const FrenetPath& rhs)
{
  if (lhs.is_generated && rhs.is_generated)
  {
    return lhs.final_cost < rhs.final_cost;
  }
  else
  {
    return lhs.est_cost < rhs.est_cost;
  }
}

bool operator > (const FrenetPath& lhs, const FrenetPath& rhs)
{
  if (lhs.is_generated && rhs.is_generated)
  {
    return lhs.final_cost > rhs.final_cost;
  }
  else
  {
    return lhs.est_cost > rhs.est_cost;
  }
}

FrenetState getFrenet(const VehicleState& current_state, const Lane& lane)
{
  int next_wp_id = nextWaypoint(current_state, lane);
  // if it reaches the end of the waypoint list
  if (next_wp_id >= lane.points.size())
  {
    next_wp_id = lane.points.size() - 1;
  }
  const int prev_wp_id = std::max(next_wp_id - 1, 0);

  // vector n from previous waypoint to next waypoint
  const double n_x = lane.points[next_wp_id].point.x - lane.points[prev_wp_id].point.x;
  const double n_y = lane.points[next_wp_id].point.y - lane.points[prev_wp_id].point.y;
  // vector x from previous waypoint to current position
  const double x_x = current_state.x - lane.points[prev_wp_id].point.x;
  const double x_y = current_state.y - lane.points[prev_wp_id].point.y;
  const double x_yaw = atan2(x_y, x_x);
  // find the projection of x on n
  const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  const double proj_x = proj_norm * n_x;
  const double proj_y = proj_norm * n_y;

  FrenetState state;
  state.d = distance(x_x, x_y, proj_x, proj_y);

  const double wp_yaw = lane.points[prev_wp_id].point.yaw;
  const double delta_yaw = unifyAngleRange(current_state.yaw - wp_yaw);

  if (wp_yaw >= x_yaw)
  {
    state.d *= -1;
  }
  
  // calculate s value
  state.s = 0;
  for (int i = 0; i < prev_wp_id; i++)
  {
    state.s += distance(lane.points[i].point.x, lane.points[i].point.y, lane.points[i+1].point.x, lane.points[i+1].point.y);
  }

  state.s += distance(0.0, 0.0, proj_x, proj_y);

  // calculate s_d and d_d
  state.s_d = current_state.v * cos(delta_yaw);
  state.d_d = current_state.v * sin(delta_yaw);

  state.s_dd = 0.0;
  state.s_ddd = 0.0;
  state.d_dd = 0.0;
  state.d_ddd = 0.0;

  return state;
}

FrenetState getFrenet(const VehicleState& current_state, const Path& path)
{
  int next_wp_id = nextWaypoint(current_state, path);
  // if it reaches the end of the waypoint list
  if (next_wp_id >= path.x.size())
  {
    next_wp_id = path.x.size() - 1;
  }
  int prev_wp_id = std::max(next_wp_id - 1, 0);

  // std::vector n from previous waypoint to next waypoint
  const double n_x = path.x[next_wp_id] - path.x[prev_wp_id];
  const double n_y = path.y[next_wp_id] - path.y[prev_wp_id];
  // std::vector x from previous waypoint to current position
  const double x_x = current_state.x - path.x[prev_wp_id];
  const double x_y = current_state.y - path.y[prev_wp_id];
  // find the projection of x on n
  const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  const double proj_x = proj_norm * n_x;
  const double proj_y = proj_norm * n_y;

  FrenetState state;
  state.d = distance(x_x, x_y, proj_x, proj_y);

  // get the normal std::vector d
  const double wp_yaw = path.yaw[prev_wp_id];
  const double delta_yaw = fiss::unifyAngleRange(current_state.yaw - wp_yaw);
  // find the yaw of std::vector x
  const double x_yaw = atan2(x_y, x_x);
  const double yaw_x_n = fiss::unifyAngleRange(x_yaw - wp_yaw);

  if (yaw_x_n < 0.0)
  {
    state.d *= -1;
  }

  // calculate s value
  state.s = 0;
  for (int i = 0; i < prev_wp_id; i++)
  {
    state.s += distance(path.x[i], path.y[i], path.x[i + 1], path.y[i + 1]);
  }
  state.s += distance(0.0, 0.0, proj_x, proj_y);

  // calculate s_d and d_d
  state.s_d = current_state.v * cos(delta_yaw);
  state.d_d = current_state.v * sin(delta_yaw);
  // Give default values to the rest of the attributes
  state.s_dd = 0.0;
  state.d_dd = 0.0;
  state.s_ddd = 0.0;
  state.d_ddd = 0.0;

  return state;
}

}  // end of namespace fiss
