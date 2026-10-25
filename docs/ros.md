# ROS 1 / ROS 2

Both wrappers are thin: the same `LocalPlanner` runs inside, planning is timer driven (or on
every odometry message with `plan_on_odom`), and configuration changes are applied between
cycles.

| ROS 2 | ROS 1 |
|---|---|
| `ros2 launch fiss_planner_ros2 planner.launch.py` | `roslaunch fiss_planner_ros1 planner.launch` |
| `ros2 param set /fiss_planner planner.type fiss` | `rosrun dynamic_reconfigure dynparam set /fiss_planner planner_type fiss` |
| components `PlannerNode`, `ControllerNode` | nodes `planner_node`, `controller_node` |

## Inputs and outputs

| Topic | Type | |
|---|---|---|
| `~/odom` | `nav_msgs/Odometry` | in |
| `~/obstacles` | `fiss_planner_msgs/ObstacleArray` | in |
| `~/reference_path` | `nav_msgs/Path` | in (path mode) |
| `~/goal` | `geometry_msgs/PoseStamped` | in (map mode) |
| `~/stop_lines` | `fiss_planner_msgs/StopLineStateArray` | in (traffic lights) |
| `~/trajectory` | `fiss_planner_msgs/Trajectory` | out: time-parameterized x, y, yaw, v, a, κ, s, d |
| `~/path`, `~/reference` | `nav_msgs/Path` | out |
| `~/stats` | `fiss_planner_msgs/PlannerStats` | out: mode, timings, counts, blocking obstacle, gap / headway / TTC to the vehicle ahead |
| `~/candidates` | `visualization_msgs/MarkerArray` | out (`publish_candidates`) |
| `~/markers` | `visualization_msgs/MarkerArray` | out (ROS 2): ego, obstacles, predicted paths; vehicle ahead and blocking obstacle highlighted |

Obstacles use `fiss_planner_msgs/ObstacleArray`: a rectangular footprint per object plus a
predicted trajectory (times relative to the header stamp). A single state with `is_static`
describes a parked object. A single state *without* `is_static` (a tracked object with its current
speed) gets a prediction according to `prediction.mode`: `lane_following` (default; along its lane
of the Lanelet2 map, constant velocity without a map), `constant_velocity`, or `as_given`.

## Keeping distance

The safety envelope and the proximity costs (see [Concepts](concepts.md#keeping-distance)) are
parameters like any other and can be tuned live:

```bash
ros2 param set /fiss_planner cost.desired_time_headway 2.0
ros2 param set /fiss_planner safety.min_ttc 2.0
ros2 param set /fiss_planner safety.crossing_time_gap 1.0   # crossing / oncoming traffic
```

`collision.ignore_rear_in_lane` is on in the shipped configuration: followers in the ego's lane
are expected to keep their distance. `degraded_horizon` sets the collision horizon of the retry
before an emergency stop.

## Visualization and recording

`ros2 launch fiss_planner_ros2 planner.launch.py rviz:=true` opens RViz 2 with the reference,
plan, candidates and scene markers. To record a run for [Foxglove](foxglove.md):

```bash
ros2 bag record -s mcap -o run /odom /perception/obstacles /tf \
    /fiss_planner/trajectory /fiss_planner/path /fiss_planner/stats \
    /fiss_planner/candidates /fiss_planner/markers
```

## Maps

Set `map.lanelet2_path` (plus `map.origin_lat/lon`) to load a Lanelet2 map. Lane-change
legality, speed limits and traffic-light stop lines come from the map and its routing graph. A
goal on `~/goal` computes the route. Without a map, publish `~/reference_path` and set
`path.lane_width` / `path.num_left_lanes` / `path.num_right_lanes`.

## Frames

Odometry and obstacles may come in any frame with a TF to `map_frame`. Positions inside the
planner refer to the vehicle box center; `odom_to_center_offset` is the distance from the
odometry frame (e.g. rear-axle `base_link`) to the center along the heading.
