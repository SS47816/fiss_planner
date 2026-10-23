# Concepts

## Frenet frame and reference line

`ReferenceLine` is an arc-length parameterized cubic spline through a polyline (any number of
points; it extends straight past both ends). It converts full kinematic states exactly between
Cartesian `(x, y, yaw, v, a, kappa)` and time-domain Frenet `(s, ṡ, s̈, d, ḋ, d̈)`; heading and
curvature of planned trajectories come from the analytic velocity and acceleration vectors.

## Corridor and lanes

A `Corridor` is a set of lane bands `[right(s), left(s)]` expressed in the reference frame. With a
map, `buildRouteCorridor` follows the ego's lane chain along the route and adds adjacent lanes
using their real boundaries, so curved and merging lanes keep their geometry. Each band carries:

- `change_allowed`: whether the ego may enter it (line markings or the routing graph; a route
  through the neighbour implies the change is legal);
- `extra_cost`: route urgency, which grows as a lane approaches the point where it leaves the
  route (or ends);
- `speed_limit`.

The union of allowed lanes is the road boundary used by the off-road check. Active stop lines
(red lights) become virtual static obstacles.

## Sampling and cost

End states form a grid over lateral offset (`num_width` samples per allowed lane) × target speed
× horizon. Lateral motion is a quintic and longitudinal motion a quartic polynomial. The cost
follows Werling et al.: horizon, terminal speed error, terminal offset from the lane center and
lane-change/urgency costs (all known from the end state), plus mean squared acceleration and
jerk. The end-state part is a lower bound of the full cost, which FISS uses as its heuristic.

## The four planners

| | Generates | Validates |
|---|---|---|
| FOP | all candidates | all candidates (optionally in parallel) |
| FOP+ | all candidates | in cost order until one passes |
| FISS | lazily, along a discrete gradient descent over the grid, guided by the previous cycle's solution | the cheapest generated candidate; repeat |
| FISS+ | lazily, by best-first neighbour exploration; then continuous refinement of the end state with a decaying step | the cheapest generated candidate; refined candidates if they beat it |

All four return the same kind of `Trajectory` and share every check.

## Validation

A candidate is accepted if it is finite and within speed, acceleration, curvature and
lateral-acceleration limits, stays inside the road bounds (heading-aware footprint), and its
inflated box does not overlap any obstacle's predicted box at the same time (separating-axis
test with a circle pre-check).

## Smoothing (optional)

The chosen trajectory defines a **space–time corridor**: at every time, the lateral interval
free of road edges and of obstacles on the side it passes them, and the longitudinal interval
free of obstacles sharing its lateral band. Two piecewise-jerk QPs (lateral, then longitudinal;
OSQP) smooth the trajectory inside that corridor while keeping the sampled speed profile's
timing. The result is used only if it passes the same validation.

## Stitching and fail-safes

`LocalPlanner` plans from the previous trajectory at `t + latency` while the vehicle tracks it
within tolerance (otherwise from the measured state). If no candidate is valid it keeps the rest
of the previous plan when that is still collision free, and otherwise brakes at the limit in its
lane (`emergency`).

## Control

`MpcController` is a linear time-varying MPC on kinematic-bicycle error dynamics (lateral,
heading, along-track and speed errors) with steering angle/rate and acceleration limits.
`StanleyPidController` is the simple baseline. Both track the time-parameterized trajectory.
