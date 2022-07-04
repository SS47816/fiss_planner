/** vehicle.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of Vehicle Parameters
 */

#include "vehicle.h"

namespace fiss
{
  Eigen::Vector3d Vehicle::bbox_size()
  {
    return Eigen::Vector3d{1.85, 1.4, 1.8};
  };
  Eigen::Vector3d Vehicle::bbox_center()
  {
    return Eigen::Vector3d{1.85, 1.4, 1.8};
  };
  std::vector<Eigen::Vector3d> Vehicle::footprint()
  {
    
    
    return std::vector<Eigen::Vector3d>{};
  }
  double Vehicle::L() { return 1.05; };
  double Vehicle::Lf() { return 0.65; };
  double Vehicle::Lr() { return 0.4; };

  double Vehicle::max_speed() { return kph2mps(100); };
  double Vehicle::max_acceleration() { return max_speed()/10.0; };
  double Vehicle::max_deceleration() { return -max_speed()/5.0; };
  double Vehicle::max_steering_angle() { return deg2rad(45); };
  double Vehicle::max_steering_rate() { return max_steering_angle()/3.0; }; 
  double Vehicle::max_curvature_front() { return std::sin(max_steering_angle())/L(); };
  double Vehicle::max_curvature_rear() { return std::tan(max_steering_angle())/L(); };
} // namespace fiss