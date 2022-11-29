/** math_utils.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Commonly used math functions
*/

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <vector>
#include <cmath>
#include <geometry_msgs/Pose.h>

namespace fiss
{

// Return PI
double pi();

// Convert degrees to radians
double deg2rad(const double x);

// Convert radians to degrees
double rad2deg(const double x);

// Convert metre per second to kilometers per hour
double mps2kph(const double x);

// Convert kilometers per hour to metre per second
double kph2mps(const double x);

// Convert angle into range [-pi, +pi]
double unifyAngleRange(const double angle);

// Limit the value within [lower_bound, upper_bound]
int limitWithinRange(int value, const int lower_bound, const int upper_bound);

// Limit the value within [lower_bound, upper_bound]
double limitWithinRange(double value, const double lower_bound, const double upper_bound);

// Calculate the Euclideam distance between two points
double distance(const double x1, const double y1, const double x2, const double y2);
// Calculate the Euclideam distance between two poses
double distance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b);

// Calculate the magnitude of a vector 
double magnitude(const double x, const double y, const double z);

// Check if a value is legal (not nan or inf)
bool isLegal(const double x);

} // end of namespace fiss

#endif // MATH_UTILS_H_