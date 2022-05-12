/** setting.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of the Planner Setting 
 */

#ifndef SETTING_H_
#define SETTING_H_

#include <iostream>

namespace fiss
{

struct Setting
{
 public:
  Setting() {};
  virtual ~Setting() {};

  // General Settings
  double tick_t;              // time tick [s]

  // Sampling Parameters
  double center_offset;       // offset from the center of the lane [m]
  int num_width;              // number of road width samples
  double max_t;               // max prediction time [s]
  double min_t;               // min prediction time [s]
  int num_t;                  // number of time samples
  double highest_speed;       // highest target speed [m/s]
  double lowest_speed;        // lowest target speed [m/s]
  int num_speed;              // number of speed samples

  // Hard Constraints
  double max_speed;           // maximum speed [m/s]
  double max_accel;           // maximum acceleration [m/ss]
  double max_decel;           // maximum deceleration [m/ss]
  double max_curvature;       // maximum curvature [rad/m]
  double max_jerk_s;          // maximum longitudinal jerk [m/s^3]
  double max_jerk_d;          // maximum lateral jerk [m/s^3]
  // double steering_angle_rate; // [rad/s]

  // Cost Weights
  double k_jerk;              // jerk cost weight
  double k_time;              // time cost weight
  double k_diff;              // speed and lateral offset cost weight
  double k_lat;               // lateral overall cost weight
  double k_lon;               // longitudinal overall cost weight
  double k_obstacle;          // obstacle cost weight
  double k_heuristic;         // heuristic cost weight

  // Collision Parameters
  double safety_margin_lon;   // lon safety margin [ratio]
  double safety_margin_lat;   // lat safety margin [ratio]
  double safety_margin_soft;  // soft safety margin [ratio]
  double vehicle_width;       // vehicle width [m]
  double vehicle_length;      // vehicle length [m]
};

}  // namespace fiss

#endif  // SETTING_H_