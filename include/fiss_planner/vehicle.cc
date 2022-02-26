#include "vehicle.h"

namespace fiss
{
  Eigen::Vector3d Vehicle::bbox_size()
  {
    return Eigen::Vector3d{5.0, 2.0, 2.0};
  };
  Eigen::Vector3d Vehicle::bbox_center()
  {
    return Eigen::Vector3d{5.0, 2.0, 2.0};
  };
  std::vector<Eigen::Vector3d> Vehicle::footprint()
  {
    
    
    return std::vector<Eigen::Vector3d>{};
  }
  double Vehicle::L() { return 2.75; };
  double Vehicle::Lf() { return 1.25; };
  double Vehicle::Lr() { return 1.5; };

  double Vehicle::max_speed() { return kph2mps(100); };
  double Vehicle::max_acceleration() { return max_speed()/10.0; };
  double Vehicle::max_deceleration() { return -max_speed()/5.0; };
  double Vehicle::max_steering_angle() { return deg2rad(45); };
  double Vehicle::max_steering_rate() { return max_steering_angle()/3.0; }; 
  double Vehicle::max_curvature_front() { return std::sin(max_steering_angle())/L(); };
  double Vehicle::max_curvature_rear() { return std::tan(max_steering_angle())/L(); };
} // namespace fiss