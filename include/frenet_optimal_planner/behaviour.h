#ifndef BEHAVIOUR_H_
#define BEHAVIOUR_H_

#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "ros/ros.h"

namespace fop
{

/**
 * @brief Possible behaviours of the buggy
 *
 */
enum class BehaviourType
{
  EMERGENCY_STOP,
  HARD_STOP,
  SOFT_STOP,
  HARD_SLOW,
  SOFT_SLOW,
  NORMAL_SPEED,
  END_POINT_STOP
};

class Behaviour
{
public:
  Behaviour() {};

  virtual ~Behaviour() {};

  void init(BehaviourType type, double target_speed, std::string assigner);

  static Behaviour getMostConservativeBehaviour(std::vector<Behaviour> behaviours);

  static bool compareBehaviour(Behaviour behaviour_1, Behaviour behaviour_2);

  void setBrakeLimits();

  double getBrakeIntensity(double current_velocity);

  double getAverageBrakeIntensity();

  double getTargetSpeed();

  bool isStopping();

  bool isSlowing();

  bool isNormal();

  std::string toString();

  void toTerminal();

private:
  BehaviourType behaviour_type;
  std::string assigner;

  double target_speed;
  double min_brake;
  double max_brake;
  double speed_diff_thresh;
};

} // namespace fop

#endif  // BEHAVIOUR_H_
