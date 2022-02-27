/** polynomials.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Defination of generic polynomials
 */

#ifndef POLYNOMIALS_H_
#define POLYNOMIALS_H_

#include "Eigen/Dense"

using Eigen::VectorXd;

namespace fiss
{

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x);
// Fit a polynomial.
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);

} // end of namespace fiss

#endif // POLYNOMIALS_H_