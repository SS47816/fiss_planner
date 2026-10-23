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
