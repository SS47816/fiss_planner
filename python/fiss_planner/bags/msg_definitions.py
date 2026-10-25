# ruff: noqa: E501
"""Definitions of fiss_planner_msgs (kept identical to msgs/fiss_planner_msgs/msg by a test)."""

MSG_DEFINITIONS = {
    "Obstacle": "# Obstacle with a rectangular footprint and a predicted trajectory.\n# A single prediction state with is_static = true describes a parked object.\nint32 id\nfloat64 length\nfloat64 width\nbool is_static\nObstacleState[] prediction\n",
    "ObstacleArray": "std_msgs/Header header\nObstacle[] obstacles\n",
    "ObstacleState": "# Predicted pose of an obstacle. t is relative to the ObstacleArray header stamp.\nfloat64 t\nfloat64 x\nfloat64 y\nfloat64 yaw\nfloat64 v\n",
    "PlannerStats": "std_msgs/Header header\nstring planner\nbool success\nbool fallback\nbool emergency\nbool stitched\nbool smoothed\nfloat64 time_ms          # planner time\nfloat64 cycle_ms         # full cycle (corridor, planning, smoothing)\nfloat64 target_speed\nint32 num_generated\nint32 num_validated\nint32 num_collision_checks\nint32 ego_lane\nbool degraded            # planned in a fallback retry (no safety envelope, or a shorter collision horizon)\nint32 blocking_obstacle_id   # obstacle that rejected the cheapest candidate (-1: none)\nint32 lead_obstacle_id   # closest obstacle ahead in the ego's path (-1: none)\nfloat64 lead_gap         # bumper-to-bumper gap to it [m] (inf: none)\nfloat64 time_headway     # gap / ego speed [s]\nfloat64 ttc              # gap / closing speed [s] (inf: not closing)\n",
    "StopLineState": "# Activate (e.g. red light) or release a stop line of the loaded map.\nint32 id\nbool active\n",
    "StopLineStateArray": "std_msgs/Header header\nStopLineState[] states\n",
    "Trajectory": "std_msgs/Header header\nstring planner           # fop | fop_plus | fiss | fiss_plus\nbool emergency           # true: fail-safe braking trajectory\nfloat64 cost\nTrajectoryPoint[] points\n",
    "TrajectoryPoint": "# Point of a planned trajectory. Positions refer to the vehicle box center.\nfloat64 t      # time since the trajectory header stamp [s]\nfloat64 x\nfloat64 y\nfloat64 yaw\nfloat64 v\nfloat64 a\nfloat64 kappa\nfloat64 s      # Frenet coordinates in the planner's reference line\nfloat64 d\n",
}
