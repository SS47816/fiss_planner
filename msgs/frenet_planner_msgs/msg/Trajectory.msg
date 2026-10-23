std_msgs/Header header
string planner           # fop | fop_plus | fiss | fiss_plus
bool emergency           # true: fail-safe braking trajectory
float64 cost
TrajectoryPoint[] points
