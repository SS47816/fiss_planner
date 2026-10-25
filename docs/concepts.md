# Concepts

## Frenet frame and reference line

`ReferenceLine` is an arc-length parameterized cubic spline through a polyline (any number of
points; it extends straight past both ends). It converts full kinematic states exactly between
Cartesian `(x, y, yaw, v, a, kappa)` and time-domain Frenet `(s, ṡ, s̈, d, ḋ, d̈)`; heading and
curvature of planned trajectories come from the analytic velocity and acceleration vectors.

Map centerlines are often noisy: digitized or recorded lanes carry vertices a few centimetres off
the line, sometimes nearly on top of each other. An interpolating spline turns that into curvature
spikes, which cap the speed (through the curvature limit) and jolt the Cartesian acceleration of
planned trajectories. With a map, the lane chain is therefore first smoothed (`smoothPolyline`):
resampled every metre, fitted with a second-difference penalty that removes wiggles shorter than
`route.smoothing_length` (20 m), and reweighted until no point moves more than
`route.smoothing_tolerance` (0.1 m), so real curves keep their shape. On the NGSIM US-101 lanes this
lowers the 95th-percentile curvature from 0.017 to 0.002 1/m. Paths given with `setReferencePath` are
used as they are.

## Corridor and lanes

A `Corridor` is a set of lane bands `[right(s), left(s)]` expressed in the reference frame. With a
map, `buildRouteCorridor` follows the ego's lane chain along the route and adds adjacent lanes
using their real boundaries, so curved and merging lanes keep their geometry. Each band carries:

- `change_allowed`: whether the ego may enter it (line markings or the routing graph; a route
  through the neighbour implies the change is legal);
- `extra_cost` and `urgency`: the route's claim on the lane (below);
- `speed_limit`.

The union of allowed lanes is the road boundary used by the off-road check. Active stop lines
(red lights) become virtual static obstacles.

**Route urgency.** The route is a list of lane ids in driving order that ends in the destination
lane; `setMap` optionally takes a goal point in that lane. Along each band the planner looks for
the points where the band cannot go on as it is, also beyond the reference line:

- where a route lane leaves the route: the route's lane changes from there are due, until it
  continues lengthwise;
- where a band off the route ends or can no longer reach the route over permitted changes: the
  changes back to it are due;
- at the end of a dead-end road, and at the goal: the changes into the lane the route continues in,
  or into the destination lane.

For `n` changes due in a distance `D`, with `U = max(urgency_distance, urgency_time · v)` (80 m,
10 s), the urgency is `u = clamp(1 − D / (n·U), 0, 1)` and the cost `w_route_urgency · n · u / (1 − u)`
(w = 10, capped at 10 w n): nothing while the changes fit comfortably, `w · n` with half a ramp per
change left, then quickly more. The more changes are needed, the earlier they start, and changes
further along the route wait for their own deadlines. A band takes its most urgent deadline. Lanes
off the route cost another `w`, so leaving the route to pass slower traffic has to be worth it.

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
so a free neighbouring lane stays attractive and the ego overtakes when that pays off. This cost
fades with the urgency of the ego lane (`1 − u`): a faster lane that the route requires leaving
does not hold back the change into a slower one.

**Following and stopping** (Werling et al., `sampling.follow_lead`). Besides the grid, every cycle
tries candidates in the ego lane that end behind the vehicle ahead at its speed after
T = 1 s, 2 s, … `max_t`, at the desired gap (`desired_gap + desired_time_headway · v`) or just
outside the safety envelope (0.5 m beyond its gap), and continue at that speed. Behind a stopped
vehicle or an active stop line they stop at that gap instead, over the time a constant deceleration
would take (never shorter than the deceleration limit allows); from rest they stay or creep up to
the gap. Their position-controlled ends and short durations give firm braking and exact stops the
grid's speed-controlled samples (3–5 s) cannot. The cheapest valid candidate of grid and follower
wins, in all four planners alike.

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
leaves room to brake. Standing still never breaches the gap floor (it closes no gap), so a vehicle
that came to rest too close may stay. The optional QP smoother respects the same envelope.

**Turning across traffic.** The gap and TTC rules cover traffic heading within 45° of the ego.
Towards crossing and oncoming vehicles the envelope is a post-encroachment time instead: a
candidate may not occupy space such a vehicle occupies within `crossing_time_gap` (0.75 s) before
or after it, so an unprotected left turn either waits for a gap or goes clearly ahead, instead of
squeezing past within the collision margin. Like the other envelope rules it gives way when no
candidate satisfies it (see the fallbacks below), so the rule alone never forces a stop halfway
through a turn.

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
(e.g. stitched onto a full stop) may not get worse rather than failing outright. Below 1 m/s,
where curvature is not checked, a candidate may not move more sideways than along the lane (more
than 45° off it): a car cannot crab. Samples slower than 0.05 m/s keep the heading they came with,
since the direction of so small a velocity means nothing.

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
this order:

1. keep the rest of the previous plan while it is still collision free (`fallback`);
2. if the safety envelope rejected candidates, re-plan with the envelope as a cost instead of a
   check, with the full collision check: a close but collision-free plan is safer than hard
   braking in traffic, and the one that breaches the envelope least wins (`degraded`). The breach
   is the worst relative shortfall of the gap below its floor or of the time to collision below
   `min_ttc`, charged at `envelope_retry_weight` (1000) per unit, so after a close cut-in the
   retry brakes firmly instead of comfortably;
3. re-plan the same way with collisions checked only over the first `degraded_horizon` seconds
   (2.5 s), since long-horizon predictions are the least reliable (`degraded`);
4. brake at the limit in its lane (`emergency`).

After an emergency stop the next plan starts from the measured state, with its initial
deceleration capped at `recovery_decel` (2 m/s²): stitching onto the stop profile would carry full
braking into every new plan and bring the vehicle to a standstill even once the way is clear.
Likewise, below 1 m/s plans start with at most the deceleration `3 v / max_t`, with which a
polynomial stop within the horizon does not back up: a vehicle coming to rest does not keep
decelerating once it stands.

## Control

`MpcController` is a linear time-varying MPC on kinematic-bicycle error dynamics (lateral,
heading, along-track and speed errors) with steering angle/rate and acceleration limits.
`StanleyPidController` is the simple baseline. Both track the time-parameterized trajectory.
