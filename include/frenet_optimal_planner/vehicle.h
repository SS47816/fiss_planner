#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "math_utils.h"
#include "Eigen/Core"

namespace fop
{
enum LINK_TYPE
{
  BASE_LINK,
  FRONT_LINK
};

class Vehicle
{
 public:
  // Physical Bounding Box
  static Eigen::Vector3d bbox_size();
  static Eigen::Vector3d bbox_center();
  static std::vector<Eigen::Vector3d> footprint();

  // distance between front and rear axles, distance from CoG to front/rear axle
  static double L();
  static double Lf();
  static double Lr();

  // Constraints
  static double max_speed();
  static double max_acceleration();
  static double max_deceleration();
  static double max_steering_angle();
  static double max_steering_rate();
  static double max_curvature_front();
  static double max_curvature_rear();
};

}  // namespace fop

#endif  // VEHICLE_H_