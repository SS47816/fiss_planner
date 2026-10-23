# Changelog

## 1.0.0 — unreleased

The first release of the unified planner family: FOP, FOP+, FISS (RA-L 2022) and FISS+ (IROS 2023)
in one code base.

### Planning
- ROS-agnostic C++17 core with four interchangeable planners behind one interface (`makePlanner`)
  and a full pipeline (`LocalPlanner`):
  - route corridor;
  - stitching;
  - target-speed policy;
  - fail-safe fallbacks (previous plan, emergency braking).
- Exact time-domain Frenet ↔ Cartesian conversion; arc-length reference lines of any length.
- Werling-style cost with an admissible end-state estimate (used by FISS/FISS+).
- Kinematic, road-boundary and oriented-box collision validation against time-stamped predictions;
  parallel validation for FOP/FOP+.
- Keeping distance:
  - per-lane car following (the speed that closes the gap to the desired headway);
  - proximity cost: headway, time to collision and lateral clearance;
  - a hard safety envelope ("virtual bumper": minimum gap, headway and TTC) with a reaction-time
    grace for cut-ins.
- Map-aware lane-following prediction, and predictions for obstacles that arrive with only their
  current state.
- Fallback ladder: previous plan, a re-plan over a shorter collision horizon, then emergency braking.
- Explainability: per-reason rejection counts, the obstacle that blocked the cheapest candidate, and
  gap / headway / TTC to the vehicle ahead in every cycle's statistics.

### Maps
- Lane-level `RoadMap` with adapters for Lanelet2 (C++), CommonRoad and CARLA (Python), providing:
  - per-lane sampling on real lane geometry;
  - lane-change legality;
  - speed and curvature limits;
  - route urgency;
  - traffic-light stop lines.

### Optimization and control (optional, OSQP)
- Piecewise-jerk QP smoother inside a space–time driving corridor.
- LTV-MPC tracking controller with steering-rate limits; Stanley+PID baseline; kinematic bicycle
  simulator.

### Integrations
- Python package `frenet-planner` (nanobind) with a CommonRoad benchmark, ROS 2 bag tools and a CARLA
  closed-loop runner.
- ROS 2 (`frenet_planner_ros2`, component nodes) and ROS 1 (`frenet_planner_ros1`) wrappers;
  `frenet_planner_msgs` builds for both.
- Runtime planner switching through ROS 2 parameters, `dynamic_reconfigure` or Python.
- Scene markers and an RViz 2 configuration (ROS 2).
- Foxglove: MCAP recording (or live streaming) of CARLA and CommonRoad runs with a ready-made layout.

### Quality
- Test suites:
  - GoogleTest, including sanitizer builds;
  - pytest;
  - ROS 2 gtest;
  - ROS 1 rostest.
- CI for all of the above and for wheels.
- Reproducible benchmarks with pinned scenario sets.
