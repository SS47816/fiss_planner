/** vehicle_state.cc
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Date types and functions related to vehicle state
 */

#include "vehicle_state.h"

namespace fop
{

VehicleState::VehicleState() {};

VehicleState::VehicleState(const double x, const double y, const double yaw, const double speed)
  : x(x), y(y), yaw(yaw), v(speed) {};

ActuatorState::ActuatorState() {};

ActuatorState::ActuatorState(const double max_accel, const double max_decel, const double max_angle)
  : max_accel(max_accel), max_decel(max_decel), max_angle(max_angle) {};

void ActuatorState::setAccel(const double a)
{
  this->accel = std::min(std::max(0.0, a), max_accel);
  this->brake = std::min(std::abs(std::min(0.0, a)), max_decel);
}

void ActuatorState::setAngle(const double delta)
{
  this->angle = fop::limitWithinRange(delta, -max_angle, max_angle);
}

}  // namespace fop
