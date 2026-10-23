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

## Keeping distance

Obstacles are projected into the Frenet frame once per cycle (extent `[s_min, s_max] × [d_min,
d_max]` and along-track speed on a 0.5 s grid), so the terms below are cheap interval tests for
every candidate. Gap and time-to-collision rules apply to obstacles moving along the reference
line (within 45°) or standing still; crossing and oncoming traffic is left to the collision check.

**Car following.** Per lane, the vehicle ahead sets a following speed
`v_lead + (gap − (desired_gap + desired_time_headway · v_lead)) / follow_time`. Candidates ending
in that lane are scored against it instead of the desired speed, so the speed term no longer pulls
the ego into the vehicle ahead. The lane's deficit to the desired speed is charged as a lane cost,
so a free neighbouring lane stays attractive and the ego overtakes when that pays off.

**Proximity cost** (`CostTerms.proximity`, per-sample means like the comfort terms):

| Term | Penalizes | Default |
|---|---|---|
| `w_headway` | gap ahead below `desired_gap + desired_time_headway · v` | 10, 3 m + 1.5 s |
| `w_ttc` | time to collision with the vehicle ahead below `desired_ttc` | 5, 4 s |
| `w_clearance` | lateral gap to an obstacle alongside below `desired_clearance` | 5, 1 m |

Each term is `(1 − actual / desired)²` when positive and 0 otherwise; obstacles behind the ego
are ignored. Because it is computed when a candidate is generated and is never negative, the
lazy planners still validate in exact cost order.

**Safety envelope** (`SafetyConfig`, a hard "virtual bumper"): a candidate is rejected
(`safety_distance`) if its gap to an obstacle ahead in its path falls below
`min_gap + min_time_headway · v` (2 m + 0.3 s) or its time to collision below `min_ttc` (1.5 s).
An obstacle already in the path, or one that cuts into the ego's lane, is checked only from
`reaction_time` (0.5 s) after it appears and may not be approached closer than it appeared, so
after a close cut-in the braking candidates stay feasible. Lane changes towards an obstacle get no
such grace; samples that move laterally away from an obstacle (pulling out to overtake) are held
to its time to collision but not to the gap floor, so an overtake that cannot be completed still
leaves room to brake. The optional QP smoother respects the same envelope.

## Prediction

Obstacles carry their own predicted states, so any predictor can be plugged in.
`predictLaneFollowing` moves an obstacle along its lane of a `RoadMap` at constant speed
(straightest successor at forks), decaying its lateral offset to the centerline; it falls back to
constant velocity off the map, when the obstacle is not heading along its lane, or when it is
nearly stopped. `completePredictions` gives every moving obstacle that arrives with only its
current state (as tracked objects from perception usually do) such a prediction; the ROS nodes
apply it according to `prediction.mode`.

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
lateral-acceleration limits, stays inside the road bounds (heading-aware footprint), respects the
safety envelope, and its inflated box does not overlap any obstacle's predicted box at the same
time (separating-axis test with a circle pre-check). A start state already at or beyond a limit
(e.g. stitched onto a full stop) may not get worse rather than failing outright.

With `collision.ignore_rear_in_lane` (on in the CARLA runner and the ROS configs), lane-keeping
candidates ignore obstacles that start behind the ego in its own lane: with traffic that reacts,
the follower keeps its distance. Lane changes still check them.

Every rejection is counted per reason (`PlanStats.rejections`), and the obstacle that rejected the
cheapest candidate is reported as `PlanStats.blocking_id` (with `blocking_t`), together with the
gap, time headway and time to collision of the vehicle ahead (`PlanStats.lead`).

## Smoothing (optional)

The chosen trajectory defines a **space–time corridor**: at every time, the lateral interval
free of road edges and of obstacles on the side it passes them, and the longitudinal interval
free of obstacles sharing its lateral band. Two piecewise-jerk QPs (lateral, then longitudinal;
OSQP) smooth the trajectory inside that corridor while keeping the sampled speed profile's
timing. The result is used only if it passes the same validation.

## Stitching and fail-safes

`LocalPlanner` plans from the previous trajectory at `t + latency` while the vehicle tracks it
within tolerance (otherwise from the measured state). If no candidate is valid, it falls back in
three steps:

1. keep the rest of the previous plan while it is still collision free (`fallback`);
2. if the safety envelope rejected candidates, re-plan without it but with the full collision
   check: a close but collision-free plan is safer than hard braking in traffic (`degraded`);
3. re-plan with collisions checked only over the first `degraded_horizon` seconds (2.5 s), since
   long-horizon predictions are the least reliable (`degraded`);
4. brake at the limit in its lane (`emergency`).

After an emergency stop the next plan starts from the measured state, with its initial
deceleration capped at `recovery_decel` (2 m/s²): stitching onto the stop profile would carry full
braking into every new plan and bring the vehicle to a standstill even once the way is clear.

## Control

`MpcController` is a linear time-varying MPC on kinematic-bicycle error dynamics (lateral,
heading, along-track and speed errors) with steering angle/rate and acceleration limits.
`StanleyPidController` is the simple baseline. Both track the time-parameterized trajectory.
