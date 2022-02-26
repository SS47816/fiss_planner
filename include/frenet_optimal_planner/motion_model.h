/** motion_model.h
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Helper functions related to vehicle motion model
 */

#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_

#include <vector>

#include "vehicle_state.h"
#include "math_utils.h"

namespace fop
{

// Get the next state std::vector of the bicycle vehicle kinematic model
VehicleState getNextKinematicModelState(const VehicleState &state, const ActuatorState &actuators, double Lf, double dt);

} // end of namespace fop

#endif // MOTION_MODEL_H_