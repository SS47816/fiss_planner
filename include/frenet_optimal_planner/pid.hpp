#pragma once

#include <iostream>
#include <cmath>

namespace control
{
class PID
{
 public:
  PID() {};
  PID(double dt, double max, double min, double Kp, double Kd, double Ki);
  ~PID() {};

  // Returns the manipulated variable given a setpoint and current process value
  double calculate(double setpoint, double pv);

 private:
  double dt_;
  double max_;
  double min_;
  double Kp_;
  double Kd_;
  double Ki_;
  double pre_error_;
  double integral_;
};

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) :
  dt_(dt),
  max_(max),
  min_(min),
  Kp_(Kp),
  Kd_(Kd),
  Ki_(Ki),
  pre_error_(0),
  integral_(0) 
{}

double PID::calculate( double setpoint, double pv)
{
  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  const double P_term = Kp_ * error;

  // Integral term
  integral_ += error * dt_;
  const double I_term = Ki_ * integral_;

  // Derivative term
  const double derivative = (error - pre_error_) / dt_;
  const double D_term = Kd_ * derivative;

  // Calculate total output
  double output = P_term + I_term + D_term;

  // Restrict to max/min
  output = std::min(output, max_);
  output = std::max(output, min_);

  // Save error to previous error
  pre_error_ = error;

  return output;
}

} // namespace control