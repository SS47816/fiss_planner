# Upgrading from the ROS 1 node (2021)

The first release of this project was a single ROS 1 node built for the LGSVL simulator. If your
stack still uses its interface, this table maps it to the current packages.

| 2021 ROS 1 node | Current |
|---|---|
| `frenet_optimal_planner_node` | `frenet_planner_ros1/planner_node` or `frenet_planner_ros2/planner_node` |
| `autoware_msgs/DetectedObjectArray` | `frenet_planner_msgs/ObstacleArray` (with predictions) |
| `nav_msgs/Path` lane info | `~/reference_path` (any length) or a Lanelet2 map |
| `auto_vehicle_cmd` (controller inside the planner) | separate controller node, `ackermann_msgs/AckermannDriveStamped` |
| `traj_max_size` / `traj_min_size` | stitching (`stitching.*` parameters) |
| `use_async` | `planner.num_threads` |
| `curr_lane_width`, `left_lane_width`, `right_lane_width` | `path.lane_width`, `path.num_left_lanes`, `path.num_right_lanes` |

The 2021 node remains available at the git tag `v0.1-legacy`.
