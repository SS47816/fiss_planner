/** vehicle_state.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of Vehicle State
 */

#include "vehicle_state.h"

namespace fiss
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
  this->angle = fiss::limitWithinRange(delta, -max_angle, max_angle);
}

}  // namespace fiss
