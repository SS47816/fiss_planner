/** quintic_polynomial.cc
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Implementation of Quintic Polynomial
 */

#include "quintic_polynomial.h"

namespace fiss
{

QuinticPolynomial::QuinticPolynomial(const std::vector<double> &start, const std::vector<double> &end, double T)
{
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  A << T * T * T, T * T * T * T, T * T * T * T * T,
       3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
       6 * T, 12 * T * T, 20 * T * T * T;

  Eigen::MatrixXd B = Eigen::MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
       end[1] - (start[1] + start[2] * T),
       end[2] - start[2];

  Eigen::MatrixXd C = A.inverse() * B;

  coefficients = {start[0], start[1], .5 * start[2]};

  for (int i = 0; i < C.size(); i++)
  {
    coefficients.push_back(C.data()[i]);
  }
}

QuinticPolynomial::QuinticPolynomial(const FrenetState& start, const FrenetState& end)
{
  const double T = end.T;
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  A << T * T * T, T * T * T * T, T * T * T * T * T,
       3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
       6 * T,     12 * T * T,    20 * T * T * T;

  Eigen::MatrixXd B = Eigen::MatrixXd(3, 1);
  B << end.d - (start.d + start.d_d * T + .5 * start.d_dd * T * T),
       end.d_d - (start.d_d + start.d_dd * T),
       end.d_dd - start.d_dd;

  Eigen::MatrixXd C = A.inverse() * B;

  coefficients = {start.d, start.d_d, .5 * start.d_dd};

  for (int i = 0; i < C.size(); i++)
  {
    coefficients.push_back(C.data()[i]);
  }
}

// calculate the s/d coordinate of a point
double QuinticPolynomial::calculatePoint(double t)
{
  return coefficients[0] + coefficients[1]*t + coefficients[2]*t*t + coefficients[3]*t*t*t + coefficients[4]*t*t*t*t + coefficients[5]*t*t*t*t*t;
}

double QuinticPolynomial::calculateFirstDerivative(double t)
{
  return coefficients[1] + 2*coefficients[2]*t + 3*coefficients[3]*t*t + 4*coefficients[4]*t*t*t + 5*coefficients[5]*t*t*t*t;
}

double QuinticPolynomial::calculateSecondDerivative(double t)
{
  return 2*coefficients[2]*t + 6*coefficients[3]*t + 12*coefficients[4]*t*t + 20*coefficients[5]*t*t*t;
}

double QuinticPolynomial::calculateThirdDerivative(double t)
{
  return 6*coefficients[3] + 24*coefficients[4]*t + 60*coefficients[5]*t*t;
}

} // namespace fiss
