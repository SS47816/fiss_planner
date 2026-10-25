std_msgs/Header header
string planner
bool success
bool fallback
bool emergency
bool stitched
bool smoothed
float64 time_ms          # planner time
float64 cycle_ms         # full cycle (corridor, planning, smoothing)
float64 target_speed
int32 num_generated
int32 num_validated
int32 num_collision_checks
int32 ego_lane
bool degraded            # planned in a fallback retry (no safety envelope, or a shorter collision horizon)
int32 blocking_obstacle_id   # obstacle that rejected the cheapest candidate (-1: none)
int32 lead_obstacle_id   # closest obstacle ahead in the ego's path (-1: none)
float64 lead_gap         # bumper-to-bumper gap to it [m] (inf: none)
float64 time_headway     # gap / ego speed [s]
float64 ttc              # gap / closing speed [s] (inf: not closing)
