/** quintic_polynomial.h
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Class for constructing and solving Quintic Polynomials
*/

#ifndef QUINTIC_POLYNOMIAL_H_
#define QUINTIC_POLYNOMIAL_H_

#include <vector>
#include "Eigen/Dense"
#include "frenet.h"

namespace fiss
{

class QuinticPolynomial
{
 public:
	// Constructor
	QuinticPolynomial(const std::vector<double> &start, const std::vector<double> &end, double T);
	QuinticPolynomial(const FrenetState& start, const FrenetState& end);
	
	// Destructor
	virtual ~QuinticPolynomial() {};
	
	// calculate the s/d coordinate of a point
	double calculatePoint(double t);

	double calculateFirstDerivative(double t);

	double calculateSecondDerivative(double t);

	double calculateThirdDerivative(double t);

 private:
	std::vector<double> coefficients;
};

} // namespace fiss

#endif //QUINTIC_POLYNOMIAL_H_