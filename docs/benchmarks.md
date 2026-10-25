# Benchmarks

All numbers were measured on one machine: an Intel Core i9-12900K (24 threads) with an RTX 3090 Ti
for CARLA, running Ubuntu 24.04. Each planning cycle runs on a single core.
Raw per-run results are in [`benchmarks/results/`](https://github.com/SS47816/fiss_planner/tree/main/benchmarks/results).

## CommonRoad, closed loop

**Scenarios.** The set is 280 public scenarios from
[commonroad-scenarios](https://gitlab.lrz.de/tum-cps/commonroad-scenarios) (commit `2416dc2`): all 130 NGSIM
recordings (US-101, Lankershim, Peachtree) and 150 hand-crafted scenarios drawn with seed 0 (list in
`benchmarks/commonroad/bench_set.txt`).

**Protocol.**
- At every 0.1 s step the planner receives the ego state and all obstacles, then the ego moves to its planned state one step later (perfect tracking, as in the FISS / FISS+ papers).
- Traffic is the recorded, non-reactive log.
- Obstacle predictions are ground truth unless stated otherwise.
- The vehicle is CommonRoad's VW Vanagon, and the grid is 5×5×5 per lane.
- The route comes from CommonRoad's route planner. The planner's goal point, where the route's lane changes must be done, is the point of the goal region the ego would pass in the middle of the time window at its initial speed, kept one second of driving inside the region's ends. The target speed brings the ego there in time, given that the planner reaches a new speed within its mean horizon, and stays within the goal speed interval.

**Metrics.**
- *Success*: the official `GoalRegion.is_reached` check, covering position, time window, speed and heading, with no collision and no road-boundary violation (CommonRoad drivability checker).
- *Headway < 1 s*: time per scenario spent less than 1 s behind the vehicle ahead in the ego's path, while keeping the lane (overtaking pull-outs excluded).
- *Min TTC < 2 s*: share of scenarios whose minimum time to collision with that vehicle drops below 2 s.
- *Within 0.5 m*: share of scenarios in which the ego's footprint comes within 0.5 m of another road user's at the same instant.

Weights and thresholds of the distance keeping were fixed a priori (1.5 s desired headway, 4 s desired
TTC, 2 m / 1.5 s hard floor) and checked on a separate development set of 100 hand-crafted scenarios
that are not part of the benchmark; they were not tuned on these 280. The time gap to crossing
traffic (0.75 s) was chosen on the development set as well. The route urgency (a 10 s ramp per lane
change), the reference smoothing (20 m, 0.1 m) and the goal point were set a priori and checked on the
development set and on 103 further hand-crafted scenarios outside the benchmark (zipper merges, highway
and urban scenes, T-junctions).

### Main results

| Planner | Success | Collision | Timeout | Plan time mean / p95 | Generated / validated | Headway < 1 s | Min TTC < 2 s | Within 0.5 m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| fop | 66.8 % | 2.1 % | 26.4 % | 9.41 / 14.58 ms | 207 / 207 | 0.17 s | 15.4 % | 6.4 % |
| fop_plus | 66.8 % | 2.1 % | 26.4 % | 1.15 / 2.05 ms | 207 / 12 | 0.17 s | 15.4 % | 6.4 % |
| fiss | 66.1 % | 1.8 % | 27.5 % | 0.56 / 1.19 ms | 17 / 11 | 0.16 s | 15.0 % | 5.7 % |
| fiss_plus | 65.4 % | 2.5 % | 27.9 % | 0.82 / 1.79 ms | 42 / 15 | 0.18 s | 15.4 % | 6.4 % |
| fop + QP | 75.7 % | 2.1 % | 17.5 % | 8.55 / 12.64 ms | 207 / 207 | 0.13 s | 16.4 % | 6.8 % |
| fop_plus + QP | 75.7 % | 2.1 % | 17.5 % | 1.11 / 1.93 ms | 207 / 12 | 0.13 s | 16.4 % | 6.8 % |
| fiss + QP | 75.0 % | 1.8 % | 18.2 % | 0.51 / 1.06 ms | 17 / 11 | 0.13 s | 16.1 % | 9.3 % |
| fiss_plus + QP | 74.6 % | 2.5 % | 18.2 % | 0.75 / 1.60 ms | 42 / 15 | 0.13 s | 12.9 % | 9.3 % |

The four search strategies reach practically the same solutions. FISS generates about 12× fewer
trajectories than FOP and plans about 17× faster. The QP smoother adds about 9 points of success,
mostly by converting timeouts.

Most of the remaining failures are timeouts in the NGSIM recordings, whose goals combine a place, a
time window of 0.1–1 s and a speed interval in dense traffic. All collisions of FISS+ with the QP
smoother except the two short urban scenarios (`ZAM_Urban-4/5_1_S-1`, within 1.2 s of the start) are
recorded vehicles running into the ego from behind: three times while it waits at a red light, twice
after it brakes with a faster vehicle closing in behind (once just after a lane change). Replayed
traffic cannot react to the ego.

### Ablations (fiss_plus with the QP smoother)

| Variant | Success | Collision | Timeout | Headway < 1 s | Min TTC < 2 s | Within 0.5 m |
|---|---:|---:|---:|---:|---:|---:|
| default: ego and adjacent lanes, ground-truth prediction | 74.6 % | 2.5 % | 18.2 % | 0.13 s | 12.9 % | 9.3 % |
| without the safety envelope and proximity cost | 80.4 % | 1.4 % | 12.9 % | 0.48 s | 22.5 % | 20.4 % |
| ego lane only (as in the papers) | 66.1 % | 4.6 % | 24.6 % | 0.24 s | 18.6 % | 10.0 % |
| lane-following prediction | 69.3 % | 9.6 % | 13.9 % | 0.13 s | 2.1 % | 10.0 % |
| constant-velocity prediction | 67.5 % | 10.7 % | 15.0 % | 0.13 s | 2.5 % | 10.4 % |

- **Keeping distance** cuts the time spent below 1 s of headway to about a quarter, the share of runs
  that get within 2 s TTC of the vehicle ahead by 40 %, and the share that come within 0.5 m of another
  road user by half. In the replayed logs it costs 5.8 points of success: 18 runs in dense NGSIM traffic
  time out because the gaps their lane changes need before the goal window closes are refused, and
  recorded traffic does not open them. Its three extra collisions are recorded vehicles running into
  the ego from behind (two of these runs leave the road instead without distance keeping).
- **Route lane changes.** The planner counts the lane changes the route still needs and makes them in
  time for the goal: several changes start earlier than one, and a slower target lane does not hold
  them back once they are due (see [route urgency](concepts.md#corridor-and-lanes)).
- **Turning across traffic.** The time gap to crossing and oncoming vehicles is part of the safety
  envelope. On the development set it brings the runs within 0.5 m of another vehicle from 31 to 1
  (of 100) at the same success rate. In the T-junction of the gallery (`ZAM_Tjunction-1_101`) the
  closest approach during the left turn grows from 0.3 m to 1.0 m, and the time gap from 0.2 s to
  1.0 s. Larger gaps backfire in replayed logs, where traffic does not yield: the ego starts to give
  way and then stops in the vehicle's path. At 1 s one development scenario ends in a collision, at
  1.5 s four do.
- **Map-aware lane sampling** (adjacent lanes) avoids conflicts the ego lane alone cannot.
- **Prediction.** With predictions from the current state only, as a deployed system has them,
  lane-following prediction gains 1.8 points of success over constant-velocity prediction, with
  9.6 % instead of 10.7 % collisions. Recorded traffic still does not react to the ego, so the remaining
  gap to ground-truth predictions is largely the replay itself. TTC values are measured from each
  cycle's predictions, so they are comparable within a prediction mode only.

## ROS 2 bag replay (open loop)

`fiss_planner.bags.from_commonroad` turns NGSIM recordings into MCAP bags. A logged vehicle is the
ego (`/odom`), its lane chain is the reference (`/planning/reference_path`), and all other traffic is
published with constant-velocity predictions (`/perception/obstacles`). `bag_replay_eval.py` plays
each bag through the real ROS 2 node (`plan_on_odom`, sim time) and compares every plan with where
the logged driver went.

| Planner | Cycles with a plan | Emergency stops | ADE 3 s | FDE 3 s | Node cycle mean / p95 |
|---|---:|---:|---:|---:|---:|
| fop | 84.0 % | 3.8 % | 1.45 m | 4.02 m | 10.9 / 17.1 ms |
| fop_plus | 83.6 % | 3.7 % | 1.37 m | 3.96 m | 3.2 / 6.7 ms |
| fiss | 83.3 % | 3.6 % | 1.33 m | 4.33 m | 2.6 / 7.3 ms |
| fiss_plus | 83.2 % | 3.2 % | 1.29 m | 4.21 m | 2.4 / 5.7 ms |

The replay is open loop: the ego follows the log, not the plan. A missing plan therefore mostly
means the logged driver's situation, for example a gap closing under constant-velocity prediction,
violates the planner's margins.

## CARLA

CARLA 0.9.16 runs headless and synchronously (20 Hz physics) on Town04. Every 0.1 s the planner gets
the ego state, the CARLA map as a `RoadMap`, with lane-change rules from the road markings and the route
from CARLA's `GlobalRoutePlanner`, and all actors with lane-following predictions on that map. CARLA
traffic reacts to the ego, so followers in its own lane are left to keep their distance
(`ignore_rear_in_lane`). The controller tracks the plan at 20 Hz. Collisions and lane invasions come
from CARLA sensors; time headway and time to collision are measured to the vehicle ahead in the ego's
path every planning cycle.

Scenarios:
- *lead vehicle*: a slower car ahead.
- *parked car*: a stopped car in the ego lane.
- *cut-in*: a scripted car drives at 9 m/s in the adjacent lane, then merges into the ego lane 8–20 m ahead within 2.5 s.
- *traffic*: 49 Traffic Manager vehicles, 60 s along a cross-map route.

| Chase camera | Bird's-eye (planner view) |
|---|---|
| ![FISS+ reacting to a cut-in in CARLA](media/carla_cut_in.webp) | ![Bird's-eye view of the same cut-in](media/carla_birdseye_cut_in.gif) |

Full-length videos (cut-in, parked car, traffic) are attached to the GitHub release; regenerate them with
`python -m fiss_planner.carla.record --scenario <name> --mp4 <file>`.

### Planners (MPC controller, Town04)

| Scenario | Planner | Collisions | Plan success | Emergency | Min TTC | Min headway | Min clearance | Mean speed | Plan time |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| lead vehicle | fop | 0 | 100.0 % | 0.0 % | 3.7 s | 1.8 s | 1.9 m | 12.0 m/s | 5.20 ms |
| lead vehicle | fop_plus | 0 | 100.0 % | 0.0 % | 3.6 s | 1.8 s | 1.9 m | 12.1 m/s | 0.60 ms |
| lead vehicle | fiss | 0 | 100.0 % | 0.0 % | 4.4 s | 1.9 s | 1.3 m | 11.3 m/s | 0.13 ms |
| lead vehicle | fiss_plus | 0 | 100.0 % | 0.0 % | 3.7 s | 1.8 s | 1.9 m | 12.2 m/s | 0.24 ms |
| parked car | fop | 0 | 100.0 % | 0.0 % | 7.6 s | 7.6 s | 1.2 m | 12.2 m/s | 4.50 ms |
| parked car | fop_plus | 0 | 100.0 % | 0.0 % | 7.6 s | 7.6 s | 1.2 m | 12.2 m/s | 0.58 ms |
| parked car | fiss | 0 | 100.0 % | 0.0 % | 7.6 s | 7.6 s | 1.2 m | 12.2 m/s | 0.15 ms |
| parked car | fiss_plus | 0 | 100.0 % | 0.0 % | 4.9 s | 4.9 s | 1.8 m | 12.4 m/s | 0.29 ms |
| cut-in (scripted) | fop | 0 | 96.0 % | 2.7 % | 0.5 s | 0.1 s | 0.0 m | 10.4 m/s | 6.01 ms |
| cut-in (scripted) | fop_plus | 0 | 95.7 % | 3.0 % | 0.6 s | 0.2 s | 0.0 m | 10.2 m/s | 1.17 ms |
| cut-in (scripted) | fiss | 0 | 96.0 % | 2.7 % | 0.4 s | 0.1 s | 0.1 m | 10.2 m/s | 0.65 ms |
| cut-in (scripted) | fiss_plus | 0 | 96.3 % | 1.7 % | 1.2 s | 0.4 s | 0.3 m | 10.9 m/s | 0.60 ms |
| traffic (49 TM vehicles, 60 s) | fop | 0 | 100.0 % | 0.0 % | 6.8 s | 3.7 s | 2.0 m | 13.3 m/s | 4.72 ms |
| traffic (49 TM vehicles, 60 s) | fop_plus | 0 | 100.0 % | 0.0 % | 4.4 s | 2.1 s | 1.5 m | 13.4 m/s | 0.52 ms |
| traffic (49 TM vehicles, 60 s) | fiss | 0 | 100.0 % | 0.0 % | 4.5 s | 2.1 s | 1.6 m | 13.4 m/s | 0.16 ms |
| traffic (49 TM vehicles, 60 s) | fiss_plus | 0 | 100.0 % | 0.0 % | 3.9 s | 1.9 s | 1.5 m | 13.4 m/s | 0.27 ms |

The Stanley+PID runs are in the JSON file; they give the same picture. Minimum TTC and headway are
measured to the vehicle ahead while keeping the lane. The scripted cut-in merges 8–20 m ahead within
2.5 s, closing at several metres per second: every planner brakes (the emergency cycles), then follows
or moves over a lane, without a collision registered by CARLA's collision sensor. It is the tightest
scenario, and the closest approach hinges on the approach speed: the ego passes the slower car at its
desired speed, and when the car merges, swerving within the lateral-acceleration limit is possible at
12.4 m/s but not at 13.4 m/s. With the MPC controller the boxes stay 0.3 m apart with FISS+ and touch
with FOP and FOP+; over both controllers they touch in four of the eight runs and come within 0.1 m in
two more, while the car merges alongside. Reference changes of a few centimetres move
the FISS+ result between 0.0 and 0.9 m, so single runs of this scenario are not a ranking.

### Dense traffic, ten seeds (FISS+, MPC)

Traffic Manager behaviour differs from run to run, so configurations are compared over ten seeds of the
60 s traffic scenario rather than single runs. The second row switches off map-aware prediction,
distance keeping, the rear-vehicle rule and the fallback retries.

| Configuration | Runs | Collisions | Plan success | Emergency | Fallback | Headway < 1 s | Median min TTC | Distance | Plan time |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| default | 10 | 0 | 97.2 % | 0.22 % | 2.58 % | 0.24 s | 9.79 s | 669 m | 0.37 ms |
| constant-velocity prediction, no distance keeping, no rear rule, no retries | 10 | 0 | 93.8 % | 0.97 % | 5.25 % | 6.70 s | 1.11 s | 631 m | 0.58 ms |

Unlike the second row, the defaults do not brake for vehicles passing in the next lane (their
predictions follow their lanes instead of sweeping across the ego's), keep their distance, and resume
right after an emergency stop instead of braking on to a standstill. The routes cross the town, and
the ego takes their turns at junctions.

Two default runs end in a jam. In seed 7 the ego covers 285 m, then queues behind a stopped car.
While it stands, no sampled candidate keeps it at rest within the limits, so the planner alternates
between the rest of its previous plan and the standstill braking profile (72 % of all fallback
cycles). In seed 9 it stops at a junction for a vehicle crossing its path (all emergency cycles). The
other eight runs plan 99.8 % of their cycles, with 0.19 % fallback and no emergency braking. A
dedicated stopping mode for queues is on the roadmap.

### Controllers (FISS+ planner)

| Scenario | Controller | Collisions | Lateral error RMS | Along-track lag RMS |
|---|---|---:|---:|---:|
| cruise | mpc | 0 | 1.4 cm | 1.05 m |
| cruise | stanley_pid | 0 | 12.4 cm | 0.86 m |
| parked car | mpc | 0 | 1.5 cm | 1.05 m |
| parked car | stanley_pid | 0 | 12.6 cm | 0.85 m |
| cut-in (scripted) | mpc | 0 | 0.7 cm | 0.88 m |
| cut-in (scripted) | stanley_pid | 0 | 7.6 cm | 0.67 m |
| traffic (30 s) | mpc | 0 | 1.1 cm | 1.01 m |
| traffic (30 s) | stanley_pid | 0 | 8.5 cm | 0.97 m |

Lane invasions are not shown in the tables because evasive lane changes and overtakes trigger them by design. They are
included in the JSON files. The along-track lag is mostly the 0.05 s latency compensation in the
stitching plus the simple throttle/brake mapping of the runner (`Actuation`); lateral tracking is the
controller's job, and the MPC holds it to 1–2 cm.

