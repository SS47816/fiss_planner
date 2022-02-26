/** vehicle_state.h
 *
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 *
 * VehicleState class
 * ActuatorState class
 */

#ifndef VEHICLE_STATE_H_
#define VEHICLE_STATE_H_

#include <cmath>
#include <iostream>
#include "math_utils.h"

namespace fop
{

struct VehicleState
{
 public:
  VehicleState();
  VehicleState(const double x, const double y, const double yaw, const double speed);

  double x;
  double y;
  double yaw;  // yaw
  double v;    // velocity
};

class ActuatorState
{
 public:
  ActuatorState();
  ActuatorState(const double max_accel, const double max_decel, const double max_angle);
  void setAccel(const double a);
  void setAngle(const double delta);

  double accel;             // acceleration [m/s^2]
  double brake;             // deceleration [m/s^2]
  double angle;             // steering angle [rad]
  double max_accel;         // max acceleration [m/s^2]
  double max_decel;         // max deceleration [m/s^2]
  double max_angle;         // max steering angle [rad]
};

}  // namespace fop

#endif  // VEHICLE_STATE_H_