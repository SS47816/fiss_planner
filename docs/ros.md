# ROS 1 / ROS 2

Both wrappers are thin: the same `LocalPlanner` runs inside, planning is timer driven (or on
every odometry message with `plan_on_odom`), and configuration changes are applied between
cycles.

| ROS 2 | ROS 1 |
|---|---|
| `ros2 launch frenet_planner_ros2 planner.launch.py` | `roslaunch frenet_planner_ros1 planner.launch` |
| `ros2 param set /frenet_planner planner.type fiss` | `rosrun dynamic_reconfigure dynparam set /frenet_planner planner_type fiss` |
| components `PlannerNode`, `ControllerNode` | nodes `planner_node`, `controller_node` |

## Inputs and outputs

See the topic table in the [README](https://github.com/SS47816/frenet_planner#quick-start).
Obstacles use `frenet_planner_msgs/ObstacleArray`: a rectangular footprint per object plus a
predicted trajectory (times relative to the header stamp). A single state with `is_static`
describes a parked object.

## Maps

Set `map.lanelet2_path` (plus `map.origin_lat/lon`) to load a Lanelet2 map. Lane-change
legality, speed limits and traffic-light stop lines come from the map and its routing graph. A
goal on `~/goal` computes the route. Without a map, publish `~/reference_path` and set
`path.lane_width` / `path.num_left_lanes` / `path.num_right_lanes`.

## Frames

Odometry and obstacles may come in any frame with a TF to `map_frame`. Positions inside the
planner refer to the vehicle box center; `odom_to_center_offset` is the distance from the
odometry frame (e.g. rear-axle `base_link`) to the center along the heading.
