# Obstacle with a rectangular footprint and a predicted trajectory.
# A single prediction state with is_static = true describes a parked object.
int32 id
float64 length
float64 width
bool is_static
ObstacleState[] prediction
