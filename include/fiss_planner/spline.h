/** spline.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Defination of 1D & 2D Splines
 */

#ifndef SPLINE_H_
#define SPLINE_H_

#include <vector>
#include "Eigen/Dense"

#include "frenet.h"
#include "math_utils.h"

namespace fiss
{
class Spline
{
public:
  // Constructors
  Spline(){};
  Spline(const std::vector<double>& x, const std::vector<double>& y);
  // Destructor
  virtual ~Spline(){};

  // Coefficients
  std::vector<double> a_;
  std::vector<long double> b_;
  std::vector<long double> c_;
  std::vector<long double> d_;
  std::vector<double> w_;

  std::vector<double> x_;
  std::vector<double> y_;

  int num_x_;

  // Calculate point y
  double calculatePoint(double t);
  // Calculate point dy
  double calculateFirstDerivative(double t);
  // Calculate point ddy
  double calculateSecondDerivative(double t);

private:
  // Construct matrix A
  Eigen::MatrixXd calculateMatrixA(std::vector<double> h);
  // Construct matrix B
  Eigen::MatrixXd calculateMatrixB(std::vector<double> h);

  // find the nearest last point
  int searchIndex(double x);
};

class Spline2D
{
public:
  // Constructors
  Spline2D(){};
  Spline2D(const Lane& ref_wps);
  // Destructor
  virtual ~Spline2D(){};

  // Data type for returned results
  struct ResultType
  {
    std::vector<double> rx;
    std::vector<double> ry;
    std::vector<double> ryaw;
    std::vector<double> rk;
    std::vector<double> s;
  };

  // Lists of point coordinates
  std::vector<double> s_;
  Spline sx_;
  Spline sy_;

  // Calculate point (x,y) coordinates
  VehicleState calculatePosition(double s);
  // Calculate curvature at the point
  double calculateCurvature(double s);
  // Calculate yaw at the point
  double calculateYaw(double s);
  // Construct the spline
  ResultType calculateSplineCourse(const Lane& ref_wps, double ds);

private:
  // Calculate a list of s
  std::vector<double> calculate_s(const Lane& ref_wps);
};

}  // namespace fiss

#endif  // SPLINE_H_