# Benchmarks

All numbers were measured on one machine: an Intel Core i9-12900K (24 threads) with an RTX 3090 Ti
for CARLA, running Ubuntu 24.04. Each planning cycle runs on a single core.
Raw per-run results are in [`benchmarks/results/`](https://github.com/SS47816/frenet_planner/tree/main/benchmarks/results).

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

**Metrics.**
- *Success*: the official `GoalRegion.is_reached` check, covering position, time window, speed and heading, with no collision and no road-boundary violation (CommonRoad drivability checker).
- *Headway < 1 s*: time per scenario spent less than 1 s behind the vehicle ahead in the ego's path, while keeping the lane (overtaking pull-outs excluded).
- *Min TTC < 2 s*: share of scenarios whose minimum time to collision with that vehicle drops below 2 s.

Weights and thresholds of the distance keeping were fixed a priori (1.5 s desired headway, 4 s desired
TTC, 2 m / 1.5 s hard floor) and checked on a separate development set of 100 hand-crafted scenarios
that are not part of the benchmark; they were not tuned on these 280.

### Main results

| Planner | Success | Collision | Timeout | Plan time mean / p95 | Generated / validated | Headway < 1 s | Min TTC < 2 s |
|---|---:|---:|---:|---:|---:|---:|---:|
| fop | 60.7 % | 2.1 % | 31.8 % | 6.65 / 9.21 ms | 205 / 205 | 0.19 s | 13.2 % |
| fop_plus | 60.7 % | 2.1 % | 31.8 % | 1.12 / 1.79 ms | 205 / 11 | 0.19 s | 13.2 % |
| fiss | 59.6 % | 2.1 % | 32.1 % | 0.56 / 1.09 ms | 18 / 12 | 0.19 s | 15.0 % |
| fiss_plus | 60.4 % | 1.8 % | 31.8 % | 0.74 / 1.51 ms | 41 / 14 | 0.19 s | 11.8 % |
| fop + QP | 71.1 % | 2.1 % | 21.4 % | 6.20 / 8.58 ms | 205 / 205 | 0.15 s | 14.3 % |
| fop_plus + QP | 71.1 % | 2.1 % | 21.4 % | 1.02 / 1.65 ms | 205 / 10 | 0.15 s | 14.3 % |
| fiss + QP | 69.3 % | 1.4 % | 23.2 % | 0.48 / 0.94 ms | 16 / 10 | 0.14 s | 13.2 % |
| fiss_plus + QP | 70.0 % | 1.4 % | 22.5 % | 0.71 / 1.40 ms | 41 / 14 | 0.16 s | 14.6 % |

The four search strategies reach practically the same solutions. FISS generates about 11× fewer
trajectories than FOP and plans about 12× faster. The QP smoother adds about 10 points of success,
mostly by converting timeouts.

### Ablations (fiss_plus with the QP smoother)

| Variant | Success | Collision | Timeout | Headway < 1 s | Min TTC < 2 s |
|---|---:|---:|---:|---:|---:|
| default: ego and adjacent lanes, ground-truth prediction | 70.0 % | 1.4 % | 22.5 % | 0.16 s | 14.6 % |
| without the safety envelope and proximity cost | 70.7 % | 1.4 % | 21.8 % | 0.55 s | 21.8 % |
| ego lane only (as in the papers) | 65.4 % | 5.0 % | 23.6 % | 0.23 s | 14.6 % |
| lane-following prediction | 65.4 % | 9.6 % | 17.5 % | 0.17 s | 2.1 % |
| constant-velocity prediction | 62.5 % | 12.5 % | 17.9 % | 0.13 s | 2.1 % |

- **Keeping distance** cuts the time spent below 1 s of headway to under a third and the share of
  runs that get within 2 s TTC of the vehicle ahead by a third, with the same collision rate. It costs
  0.7 points of success: in a replayed log a larger gap can mean reaching the goal one time step after
  its window closes.
- **Map-aware lane sampling** (adjacent lanes) avoids conflicts the ego lane alone cannot.
- **Prediction.** With predictions from the current state only, as a deployed system has them,
  lane-following prediction gains 2.9 points of success and avoids almost a quarter of the collisions
  of constant-velocity prediction. Recorded traffic still does not react to the ego, so the remaining
  gap to ground-truth predictions is largely the replay itself. TTC values are measured from each
  cycle's predictions, so they are comparable within a prediction mode only.

## ROS 2 bag replay (open loop)

`frenet_planner.bags.from_commonroad` turns NGSIM recordings into MCAP bags. A logged vehicle is the
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
`python -m frenet_planner.carla.record --scenario <name> --mp4 <file>`.

### Planners (MPC controller, Town04)

| Scenario | Planner | Collisions | Plan success | Emergency | Min TTC | Min headway | Mean speed | Plan time |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| lead vehicle | fop | 0 | 100.0 % | 0.0 % | 3.7 s | 1.8 s | 12.0 m/s | 4.63 ms |
| lead vehicle | fop_plus | 0 | 100.0 % | 0.0 % | 3.7 s | 1.8 s | 12.1 m/s | 0.58 ms |
| lead vehicle | fiss | 0 | 100.0 % | 0.0 % | 4.5 s | 1.9 s | 11.2 m/s | 0.13 ms |
| lead vehicle | fiss_plus | 0 | 100.0 % | 0.0 % | 11.3 s | 11.3 s | 12.0 m/s | 0.23 ms |
| parked car | fop | 0 | 100.0 % | 0.0 % | 7.6 s | 7.6 s | 12.1 m/s | 4.68 ms |
| parked car | fop_plus | 0 | 100.0 % | 0.0 % | 7.6 s | 7.6 s | 12.1 m/s | 0.61 ms |
| parked car | fiss | 0 | 100.0 % | 0.0 % | 7.6 s | 7.6 s | 12.0 m/s | 0.14 ms |
| parked car | fiss_plus | 0 | 100.0 % | 0.0 % | 4.9 s | 4.9 s | 12.3 m/s | 0.29 ms |
| cut-in (scripted) | fop | 0 | 95.7 % | 2.7 % | 0.6 s | 0.6 s | 9.4 m/s | 4.55 ms |
| cut-in (scripted) | fop_plus | 0 | 95.7 % | 2.7 % | 0.5 s | 0.5 s | 9.3 m/s | 1.12 ms |
| cut-in (scripted) | fiss | 0 | 95.7 % | 2.7 % | 0.6 s | 0.6 s | 9.0 m/s | 0.64 ms |
| cut-in (scripted) | fiss_plus | 0 | 96.0 % | 3.0 % | 0.5 s | 0.5 s | 9.9 m/s | 0.76 ms |
| traffic (49 TM vehicles, 60 s) | fop | 0 | 100.0 % | 0.0 % | 3.4 s | 1.7 s | 13.7 m/s | 4.81 ms |
| traffic (49 TM vehicles, 60 s) | fop_plus | 0 | 100.0 % | 0.0 % | 3.8 s | 1.8 s | 13.9 m/s | 0.66 ms |
| traffic (49 TM vehicles, 60 s) | fiss | 0 | 100.0 % | 0.0 % | 4.1 s | 1.8 s | 13.8 m/s | 0.15 ms |
| traffic (49 TM vehicles, 60 s) | fiss_plus | 0 | 100.0 % | 0.0 % | 3.8 s | 1.8 s | 14.0 m/s | 0.22 ms |

The Stanley+PID runs are in the JSON file; they give the same picture. Minimum TTC and headway are
measured to the vehicle ahead while keeping the lane. The scripted cut-in merges 8–20 m ahead within
2.5 s, closing at several metres per second: every planner brakes hard for about a second (the
emergency cycles), then follows or passes, without a collision.

### Dense traffic, ten seeds (FISS+, MPC)

Traffic Manager behaviour differs from run to run, so configurations are compared over ten seeds of the
60 s traffic scenario rather than single runs. The second row switches off map-aware prediction,
distance keeping, the rear-vehicle rule and the fallback retries.

| Configuration | Runs | Collisions | Plan success | Emergency | Fallback | Headway < 1 s | Median min TTC | Distance | Plan time |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| default | 10 | 0 | 99.3 % | 0.02 % | 0.65 % | 0.01 s | 3.7 s | 713 m | 0.31 ms |
| constant-velocity prediction, none of the above | 10 | 0 | 96.5 % | 0.40 % | 3.13 % | 0.48 s | 2.9 s | 712 m | 0.56 ms |

Unlike the second row, the defaults do not brake for vehicles passing in the next lane (their
predictions follow their lanes instead of sweeping across the ego's), keep their distance, and resume
right after an emergency stop instead of braking on to a standstill.

### Controllers (FISS+ planner)

| Scenario | Controller | Collisions | Lateral error RMS | Along-track lag RMS |
|---|---|---:|---:|---:|
| cruise | mpc | 0 | 1.4 cm | 1.03 m |
| cruise | stanley_pid | 0 | 12.2 cm | 0.84 m |
| parked car | mpc | 0 | 1.6 cm | 1.04 m |
| parked car | stanley_pid | 0 | 12.4 cm | 0.84 m |
| cut-in (scripted) | mpc | 0 | 1.7 cm | 0.74 m |
| cut-in (scripted) | stanley_pid | 0 | 8.3 cm | 0.68 m |
| traffic (30 s) | mpc | 0 | 1.1 cm | 1.01 m |
| traffic (30 s) | stanley_pid | 0 | 9.3 cm | 0.97 m |

Lane invasions are not shown in the tables because evasive lane changes and overtakes trigger them by design. They are
included in the JSON files. The along-track lag is mostly the 0.05 s latency compensation in the
stitching plus the simple throttle/brake mapping of the runner (`Actuation`); lateral tracking is the
controller's job, and the MPC holds it to 1–2 cm.

