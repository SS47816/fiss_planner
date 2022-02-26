#include "behaviour.h"

namespace fop
{

/* --------------------------- Dynamic Parameters --------------------------- */

double NORMAL_CRUISE_SPEED = 3.0;
double OBSTACLE_SLOWDOWN_SPEED = 1.0;
double WAYPOINT_SLOWDOWN_SPEED = 2.7;
double MAP_OBSTACLE_SLOWDOWN_SPEED = 2.0;

double EMERGENCY_STOP_BRAKE_MIN = 0.8;
double EMERGENCY_STOP_BRAKE_MAX = 1.0;

//double HARD_STOP_BRAKE_MIN = 0.6;
double HARD_STOP_BRAKE_MIN = 0.4;
//double HARD_STOP_BRAKE_MAX = 0.8;
double HARD_STOP_BRAKE_MAX = 0.6;

double SOFT_STOP_BRAKE_MIN = 0.3;
double SOFT_STOP_BRAKE_MAX = 0.6;

double HARD_SLOW_BRAKE_MIN = 0.4;
double HARD_SLOW_BRAKE_MAX = 0.8;

double SOFT_SLOW_BRAKE_MIN = 0.2;
double SOFT_SLOW_BRAKE_MAX = 0.5;

void Behaviour::init(BehaviourType type, double target_speed, std::string assigner)
{
  this->target_speed = target_speed;
  this->behaviour_type = type;
  this->assigner = assigner;
  this->speed_diff_thresh = 0.02;
  setBrakeLimits();
}

/**
 * @brief get the most conservative bhaviour from a vector of Behaviours
 *
 * @param behaviours the vector of Behaviours to compare
 * @return Behaviour the most conservative Behaviour
 */
Behaviour Behaviour::getMostConservativeBehaviour(std::vector<Behaviour> behaviours)
{
  std::sort(behaviours.begin(), behaviours.end(), compareBehaviour);

  return behaviours.at(0);
}

/**
 * @brief set the brake limits of the Behaviour based on its type.
 *
 */
void Behaviour::setBrakeLimits()
{
  switch (this->behaviour_type)
  {
    case BehaviourType::END_POINT_STOP:
      min_brake = SOFT_STOP_BRAKE_MIN;
      max_brake = SOFT_STOP_BRAKE_MAX;
      break;

    case BehaviourType::EMERGENCY_STOP:
      min_brake = EMERGENCY_STOP_BRAKE_MIN;
      max_brake = EMERGENCY_STOP_BRAKE_MAX;
      break;

    case BehaviourType::HARD_STOP:
      min_brake = HARD_STOP_BRAKE_MIN;
      max_brake = HARD_STOP_BRAKE_MAX;
      break;

    case BehaviourType::SOFT_STOP:
      min_brake = SOFT_STOP_BRAKE_MIN;
      max_brake = SOFT_STOP_BRAKE_MAX;
      break;

    case BehaviourType::HARD_SLOW:
      min_brake = HARD_SLOW_BRAKE_MIN;
      max_brake = HARD_SLOW_BRAKE_MAX;
      break;

    case BehaviourType::SOFT_SLOW:
      min_brake = SOFT_SLOW_BRAKE_MIN;
      max_brake = SOFT_SLOW_BRAKE_MAX;
      break;

    case BehaviourType::NORMAL_SPEED:
      min_brake = 0.0;
      max_brake = 0.0;
      break;
  }
}

/**
 * @brief compare 2 behaviours
 *
 * @param behaviour_1 first behaviour to compare
 * @param behaviour_2 second behaviour to compare
 * @return true if behaviour_1 is more conservative
 */
bool Behaviour::compareBehaviour(Behaviour behaviour_1, Behaviour behaviour_2)
{
  if (behaviour_1.target_speed < behaviour_2.target_speed)
  {
    return true;
  }
  else if (behaviour_1.target_speed > behaviour_2.target_speed)
  {
    return false;
  }
  else  // same speed
  {
    if (behaviour_1.getAverageBrakeIntensity() > behaviour_2.getAverageBrakeIntensity())
    {
      return true;
    }
    else if (behaviour_1.getAverageBrakeIntensity() < behaviour_2.getAverageBrakeIntensity())
    {
      return false;
    }
    else  // same speed and average brake
    {
      if (behaviour_1.min_brake > behaviour_2.min_brake)
      {
        return true;
      }
      else if (behaviour_1.min_brake < behaviour_2.min_brake)
      {
        return false;
      }
      else  // same speed and average brake and min brake
      {
        return behaviour_1.speed_diff_thresh < behaviour_2.speed_diff_thresh;
      }
    }
  }
}

/**
 * @brief Get the brake intensity based on the current speed of the buggy and its Behaviour. Brake intensity is
 * calculated using a linear equation where brake intensity is the y axis and speed is the x axis.
 *
 * @param current_speed the current speed of the buggy
 * @return double the desired brake intensity. If the current speed is less than the target speed or within the
 * speed_diff_thresh, return 0.
 */
double Behaviour::getBrakeIntensity(double current_speed)
{
  if (current_speed < target_speed + speed_diff_thresh)
  {
    return 0;
  }
  else
  {
    if (min_brake > max_brake)
    {
      return min_brake;
    }
    else
    {
      return min_brake + ((max_brake - min_brake) / (NORMAL_CRUISE_SPEED - target_speed)) *
                             (current_speed - target_speed - speed_diff_thresh);
    }
  }
}

double Behaviour::getAverageBrakeIntensity()
{
  return (this->min_brake + this->max_brake) / 2;
}

double Behaviour::getTargetSpeed()
{
  return this->target_speed;
}

bool Behaviour::isStopping()
{
  return behaviour_type == BehaviourType::EMERGENCY_STOP || behaviour_type == BehaviourType::HARD_STOP ||
         behaviour_type == BehaviourType::SOFT_STOP;
}

bool Behaviour::isSlowing()
{
  return behaviour_type == BehaviourType::HARD_SLOW || behaviour_type == BehaviourType::SOFT_SLOW;
}

bool Behaviour::isNormal()
{
  return behaviour_type == BehaviourType::NORMAL_SPEED;
}

std::string Behaviour::toString()
{
  std::stringstream output;

  output << "[" + assigner + "] ";

  switch (behaviour_type)
  {
    case BehaviourType::EMERGENCY_STOP:
      output << "EMERGENCY STOP: ";
      break;

    case BehaviourType::HARD_STOP:
      output << "HARD STOP: ";
      break;

    case BehaviourType::SOFT_STOP:
      output << "SOFT STOP: ";
      break;

    case BehaviourType::HARD_SLOW:
      output << "HARD SLOW: ";
      break;

    case BehaviourType::SOFT_SLOW:
      output << "SOFT SLOW: ";
      break;

    case BehaviourType::NORMAL_SPEED:
      output << "NORMAL SPEED: ";
      break;
  }

  output << std::setprecision(2) << "Target Speed: " << target_speed << " m/s,"
         << " Brake Range: " << min_brake << "-" << max_brake;

  return output.str();
}

void Behaviour::toTerminal()
{
  if (isStopping())
  {
    ROS_ERROR("%s", toString().c_str());
  }
  else if (isSlowing())
  {
    ROS_WARN("%s", toString().c_str());
  }
  else
  {
    ROS_INFO("%s", toString().c_str());
  }
}

} // namespace fop