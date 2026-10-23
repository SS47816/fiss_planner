// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2021-2026 Shuo Sun
#pragma once

#include <memory>
#include <string>

#include "frenet_planner/common/types.hpp"
#include "frenet_planner/planning/trajectory.hpp"

namespace frenet_planner {

struct ControlCommand {
  double steer = 0.0;  ///< front wheel angle [rad]
  double accel = 0.0;  ///< longitudinal acceleration [m/s^2]
};

/// Tracks a time-parameterized trajectory. `t` is the time since the
/// trajectory's t = 0; states refer to the box center (see EgoState).
class Controller {
 public:
  virtual ~Controller() = default;
  virtual ControlCommand compute(const EgoState& ego, const Trajectory& traj, double t) = 0;
  virtual void reset() {}
  virtual std::string name() const = 0;
};

/// Kinematic bicycle model (rear-axle reference) with steering angle and
/// steering-rate limits, used for closed-loop tests and as the MPC model.
class KinematicBicycle {
 public:
  KinematicBicycle(const VehicleParams& vehicle, const EgoState& initial, double initial_steer = 0.0);
  /// Apply a command for dt seconds (limits enforced, RK4 integration).
  void step(const ControlCommand& cmd, double dt);
  const EgoState& state() const { return state_; }
  double steer() const { return steer_; }

 private:
  VehicleParams vehicle_;
  EgoState state_;
  double steer_;
};

struct StanleyPidConfig {
  double k_heading = 1.0;  ///< heading error gain
  double k_cross = 1.5;    ///< cross-track gain
  double k_soft = 1.0;     ///< softening speed [m/s]
  double kp = 1.0, ki = 0.1, kd = 0.0;
  double dt = 0.05;  ///< control period (for the PID integrator) [s]
};

/// Stanley lateral control (front axle) with a PID + feedforward speed loop:
/// a simple, robust baseline.
class StanleyPidController : public Controller {
 public:
  StanleyPidController(const VehicleParams& vehicle, const StanleyPidConfig& config = {});
  ControlCommand compute(const EgoState& ego, const Trajectory& traj, double t) override;
  void reset() override;
  std::string name() const override { return "stanley_pid"; }

 private:
  VehicleParams vehicle_;
  StanleyPidConfig cfg_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  bool has_prev_ = false;
};

struct MpcConfig {
  int horizon = 25;         ///< steps
  double dt = 0.05;         ///< [s]
  double q_lat = 20.0;      ///< lateral error weight
  double q_heading = 10.0;  ///< heading error weight
  double q_lon = 2.0;       ///< longitudinal (along-track) error weight
  double q_speed = 4.0;     ///< speed error weight
  double r_steer = 1.0;
  double r_accel = 0.2;
  double rd_steer = 200.0;  ///< steering change weight (smoothness)
  double rd_accel = 2.0;
};

/// Linear time-varying MPC on the kinematic bicycle error dynamics around the
/// reference trajectory (lateral, heading, along-track and speed errors),
/// with steering angle, steering rate and acceleration limits, solved with
/// OSQP. Requires the library to be built with OSQP (see hasOptimizers()).
class MpcController : public Controller {
 public:
  MpcController(const VehicleParams& vehicle, const MpcConfig& config = {});
  ControlCommand compute(const EgoState& ego, const Trajectory& traj, double t) override;
  void reset() override { prev_steer_ = 0.0; }
  std::string name() const override { return "mpc"; }
  /// Whether the last compute() solved the QP (otherwise it fell back to
  /// feedforward plus Stanley).
  bool lastSolveOk() const { return last_ok_; }

 private:
  VehicleParams vehicle_;
  MpcConfig cfg_;
  StanleyPidController backup_;
  double prev_steer_ = 0.0;
  bool last_ok_ = false;
};

/// "mpc" or "stanley_pid".
std::unique_ptr<Controller> makeController(const std::string& name, const VehicleParams& vehicle);

}  // namespace frenet_planner
