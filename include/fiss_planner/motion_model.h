/** motion_model.h
 * 
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Helper functions related to vehicle motion model
 */

#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_

#include <vector>

#include "vehicle_state.h"
#include "math_utils.h"

namespace fiss
{

// Get the next state std::vector of the bicycle vehicle kinematic model
VehicleState getNextKinematicModelState(const VehicleState &state, const ActuatorState &actuators, double Lf, double dt);

} // end of namespace fiss

#endif // MOTION_MODEL_H_