# Changelog

## Unreleased

Lane changes planned along the route, following and stopping behind the vehicle ahead, a safety
envelope that also guides the fallbacks, smooth reference lines on noisy maps, and a tuned planner
profile per benchmark.

### Planning
- Route urgency counts lane changes along every lane band. They fall due where a route lane leaves
  the route (the changes the route makes there), where a lane off the route ends or can no longer
  reach it, at a dead end, and at an optional goal point in the destination lane
  (`LocalPlanner::setMap(map, route, goal)`, `set_map(..., goal=(x, y))`), also beyond the
  reference line. The cost ramps over `max(urgency_distance, urgency_time · v)` per change (new
  `urgency_time`, 10 s) and grows steeply towards the deadline; a band takes its most urgent
  deadline, and lanes off the route cost `w_route_urgency` on top.
- The car-following speed deficit of a lane fades with the urgency of the ego lane
  (`LaneBand::urgency`), so a faster lane the route requires leaving does not hold back the change.
- At forks the lane chain follows the branch furthest along the route, so routes traced from
  waypoints that touch the straight-on lane before a turn are followed through the turn.
- Map reference lines are smoothed within 0.1 m of the lane centerlines (`smoothPolyline`,
  `route.smoothing_length` / `route.smoothing_tolerance`): lanes digitized with vertex noise give
  smooth curvature, curvature speed limits and accelerations.

- Following and stopping candidates (Werling et al., `sampling.follow_lead`): in the ego lane, end
  behind the vehicle ahead at its speed after 1 s, 2 s, ... at the desired gap or just outside the
  safety envelope; behind a stopped vehicle or stop line, stop at that gap (from rest: stay, or
  creep up to it). They compete with the grid in all four planners.
- The fallback retries keep the safety envelope as a cost (`envelope_retry_weight`,
  `GapEvaluator::breach`): when no candidate keeps it, the plan that breaches it least wins, so a
  close cut-in is met with firm braking.
- Low-speed lateral planning (`sampling.low_speed`, Werling et al.): below this start speed the
  lateral motion follows the distance driven, d(s), so the vehicle moves sideways only as it moves
  forward, and a plan that stands keeps its offset.
- Road-edge cost (`cost.w_road_edge`): the footprint's distance beyond the road edges along the
  trajectory, for prompt recovery after a wide turn. The road check holds the footprint's heading
  below 0.5 m/s, where the Frenet velocity no longer tells the vehicle's orientation.
- Standing still never breaches the envelope's gap floor. Below 1 m/s, plans start with at most the
  deceleration `3 v / max_t`, candidates may not crab (more than 45° off the lane), and samples
  slower than 0.05 m/s keep their heading.

### Configuration
- The whole `LocalPlannerConfig` loads from YAML (`loadLocalPlannerConfig` /
  `load_local_planner_config`, `parseLocalPlannerConfig`). Keys are named after the fields, as the
  ROS 2 parameters are, and settings missing from the file come from a base configuration.
  `toYaml` / `to_yaml` writes every setting so that it reads back exactly, and
  `LocalPlannerConfig::validate` checks the values.
- Planner profiles, one per benchmark (`fiss_planner.profiles`: `commonroad`, `carla_scenarios`),
  pin every setting. The CommonRoad benchmark and the CARLA runner take `--profile` and record the
  profile and a hash of the resolved settings with every result. The ROS 1/2 `config_file`
  accepts profiles.
- Benchmark splits are listing files (`benchmarks/commonroad/dev_set.txt`, `check_set.txt`,
  `bench_set.txt`), read by `--scenarios`. `benchmarks/tune.py` tunes a profile with Optuna on a
  development split, scores it on a check split, and refuses test splits.
- CommonRoad benchmark: `--samples`, `--adjacent-lanes` and `--smoother` / `--no-smoother` override
  the profile only when given. `--config` is an alias of `--profile`.

### Integrations
- nuPlan: `fiss_planner.nuplan.FissPlanner`, a nuPlan planner on the map around the ego along the
  mission's roadblocks (corrected to start at the ego and bridged where they are not connected),
  with red lights as stop lines and tracked objects as obstacles; profile `nuplan`, pixi environment
  `nuplan`, development split `mini_dev14` and scripts in `benchmarks/nuplan/`.
- The CARLA environment uses CARLA 0.9.15 (as Bench2Drive) with Python 3.10.
- Bench2Drive: `fiss_planner.carla.leaderboard_agent`, a privileged CARLA Leaderboard 2.0 agent
  (the map around the route, traffic lights as stop lines, actors and static props as obstacles,
  MPC tracking); profile `bench2drive`, scripts in `benchmarks/bench2drive/`. The CARLA adapter
  builds lane maps of a region around a route (large towns) and the traffic lights' stop lines.
- `benchmarks/tune.py` also tunes nuPlan profiles (search space `benchmarks/tuning/nuplan.yaml`).
- The ROS 1/2 nodes pass the goal pose to the planner; the CARLA runner passes the route's end.
- CommonRoad benchmark: the planner gets the goal region as its destination, and the target speed
  aims for the point of the region the ego reaches in the middle of the time window.

## 1.0.0 — 2026-10-25

The first release of FISS Planner: FISS (RA-L 2022), FISS+ (IROS 2023) and the FOP / FOP+ baselines
in one code base. The project continues `SS47816/frenet_planner` (formerly `frenet_optimal_planner`),
whose package was briefly published as `frenet-planner` 1.0.0, and `SS47816/fiss_plus_planner`.
`pip install frenet-planner` now installs `fiss-planner`; the paper code stays at the tags
`paper-ral2022` (this repository) and `paper-iros2023` (`fiss_plus_planner`).

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
    grace for cut-ins;
  - a post-encroachment time to crossing and oncoming vehicles, so turns across traffic keep clear
    of the vehicles they cross.
- Map-aware lane-following prediction, and predictions for obstacles that arrive with only their
  current state.
- Fallback ladder: previous plan, a re-plan without the safety envelope, a re-plan over a shorter
  collision horizon, then emergency braking; plans after an emergency stop resume from the measured
  state.
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
- Python package `fiss-planner` (nanobind) with a CommonRoad benchmark, ROS 2 bag tools and a CARLA
  closed-loop runner.
- ROS 2 (`fiss_planner_ros2`, component nodes) and ROS 1 (`fiss_planner_ros1`) wrappers;
  `fiss_planner_msgs` builds for both.
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
