/** math_utils.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Commonly used math functions
*/

#include "math_utils.h"

namespace fiss
{

// Return PI
double pi() { return M_PI; }

// Convert degrees to radians
double deg2rad(const double x) { return x * pi() / 180; }

// Convert radians to degrees
double rad2deg(const double x) { return x * 180 / pi(); }

// Convert metre per second to kilometers per hour
double mps2kph(const double x) { return x * 3.6; }

// Convert kilometers per hour to meter per second
double kph2mps(const double x) { return x / 3.6; }

// Convert angle into range [-pi, +pi]
double unifyAngleRange(const double angle)
{
  auto new_angle = angle;
  while (new_angle > pi())
  {
    new_angle -= 2 * pi();
  }
  while (new_angle < -pi())
  {
    new_angle += 2 * pi();
  }
  return new_angle;
}

// Limit the value within [lower_bound, upper_bound]
int limitWithinRange(const int value, const int lower_bound, const int upper_bound)
{
  auto new_value = std::max(value, lower_bound);
  new_value = std::min(new_value, upper_bound);
  return new_value;
}

// Limit the value within [lower_bound, upper_bound]
double limitWithinRange(const double value, const double lower_bound, const double upper_bound)
{
  auto new_value = std::max(value, lower_bound);
  new_value = std::min(new_value, upper_bound);
  return new_value;
}

// Calculate the Euclideam distance between two points
double distance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate the Euclideam distance between two poses
double distance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  return distance(a.position.x, a.position.y, b.position.x, b.position.y);
}

// Calculate the Euclideam distance between two points
double magnitude(const double x, const double y, const double z)
{
  return sqrt(x*x + y*y + z*z);
}

// Check if a value is legal (not nan or inf)
bool isLegal(const double x)
{
  return (std::isnan(x) || std::isinf(x))? false : true;
}

} // end of namespace fiss