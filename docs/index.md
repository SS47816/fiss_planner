# frenet_planner

Fast sampling-based trajectory planners for autonomous driving: **FOP**, **FOP+**, **FISS**
(RA-L 2022) and **FISS+** (IROS 2023) in one C++17 core, with Python bindings and ROS 1 / ROS 2
nodes, Lanelet2 / CommonRoad / CARLA support, an optional QP smoother and an MPC controller.

![FISS+ driving through dense traffic in CARLA](media/carla_traffic_hero.webp)

**[▶ Interactive replay](demo/index.html)**: step through CommonRoad scenarios with every planner,
including the candidate trajectories each one explored.

| | |
|---|---|
| ![FOP, FISS and FISS+ on the same merge](media/compare_fop_fiss_fissplus.gif) | ![Dense traffic on US-101](media/us101_fiss_plus.gif) |
| ![Left turn at a T-junction](media/tjunction_fiss_plus.gif) | ![Overtaking a parked car in CARLA](media/carla_overtake.webp) |

- [Concepts](concepts.md): reference line, corridor, sampling, the four search strategies, validation,
  smoothing, stitching and control.
- [Python](python.md): `pip install frenet-planner`, planners, `LocalPlanner`, maps, controllers.
- [ROS](ros.md): nodes, topics, parameters, Lanelet2, runtime switching.
- [Benchmarks](benchmarks.md): CommonRoad, ROS 2 bag replay and CARLA results, and how to reproduce them.
