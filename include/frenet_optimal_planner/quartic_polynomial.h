/** quartic_polynomial.h
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Class for constructing and solving Quartic Polynomials
*/

#ifndef QUARTIC_POLYNOMIAL_H_
#define QUARTIC_POLYNOMIAL_H_

#include <vector>
#include "Eigen/Dense"
#include "frenet.h"

namespace fop
{

class QuarticPolynomial
{
 public:
	// Constructor
	QuarticPolynomial(const std::vector<double> &start, const std::vector<double> &end, double T);
	QuarticPolynomial(const FrenetState& start, const FrenetState& end);

	// Destructor
	virtual ~QuarticPolynomial() {};
	
	// calculate the s/d coordinate of a point
	double calculatePoint(double t);

	double calculateFirstDerivative(double t);

	double calculateSecondDerivative(double t);

	double calculateThirdDerivative(double t);
	
 private:
	std::vector<double> coefficients;
};

} // namespace fop

#endif //QUARTIC_POLYNOMIAL_H_