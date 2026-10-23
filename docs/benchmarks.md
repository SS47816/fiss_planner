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
- *Goal area*: lenient; the ego center enters the goal region at any time, ignoring time, speed and heading.

### Main results

| Planner | Success | Collision | Off-road | Timeout | Goal area | Plan time (mean / p95) | Generated / validated |
|---|---:|---:|---:|---:|---:|---:|---:|
| fop | 62.9 % | 2.1 % | 5.0 % | 30.0 % | 74.3 % | 7.1 / 9.8 ms | 205 / 205 |
| fop_plus | 62.9 % | 2.1 % | 5.0 % | 30.0 % | 74.3 % | 0.7 / 1.2 ms | 205 / 11 |
| fiss | 61.1 % | 2.1 % | 5.7 % | 30.7 % | 72.9 % | 0.4 / 0.8 ms | 17 / 11 |
| fiss_plus | 62.9 % | 2.5 % | 5.0 % | 29.3 % | 73.9 % | 0.6 / 1.2 ms | 41 / 15 |
| fop + QP | 70.4 % | 2.1 % | 5.7 % | 21.4 % | 73.9 % | 10.5 / 18.0 ms | 205 / 205 |
| fop_plus + QP | 70.4 % | 2.1 % | 5.7 % | 21.4 % | 73.9 % | 1.1 / 3.0 ms | 205 / 11 |
| fiss + QP | 70.4 % | 1.4 % | 5.7 % | 22.1 % | 74.6 % | 0.5 / 1.3 ms | 16 / 10 |
| fiss_plus + QP | 70.4 % | 2.1 % | 5.7 % | 21.4 % | 73.9 % | 0.8 / 2.2 ms | 41 / 15 |

The four search strategies reach practically the same solutions. FISS generates about 12× fewer
trajectories than FOP and is about 17× faster. The QP smoother adds 7.5 points of success, mostly by
converting timeouts.

### Ablations (all with the QP smoother)

| Variant | Success (fiss_plus) | Collision | Note |
|---|---:|---:|---|
| default: ego lane + adjacent lanes, ground-truth prediction | 70.4 % | 2.1 % | |
| ego lane only (as in the papers) | 68.2 % | 3.6 % | map-aware lane sampling avoids more conflicts |
| constant-velocity prediction | 60.0 % | 13.9 % | recorded traffic does not react to the ego |

The constant-velocity row is the realistic deployment setting, and prediction quality dominates
there. Because predictions are an input (`Obstacle.prediction`), a learned predictor can be plugged
in without touching the planner.

## ROS 2 bag replay (open loop)

`frenet_planner.bags.from_commonroad` turns NGSIM recordings into MCAP bags. A logged vehicle is the
ego (`/odom`), its lane chain is the reference (`/planning/reference_path`), and all other traffic is
published with constant-velocity predictions (`/perception/obstacles`). `bag_replay_eval.py` plays
each bag through the real ROS 2 node (`plan_on_odom`, sim time) and compares every plan with where
the logged driver went.

| Planner | Cycles with a plan | Emergency stops | ADE 3 s | FDE 3 s | Node cycle mean / p95 |
|---|---:|---:|---:|---:|---:|
| fop | 75.9 % | 8.6 % | 1.37 m | 4.30 m | 9.5 / 15.3 ms |
| fop_plus | 76.8 % | 8.0 % | 1.33 m | 4.07 m | 2.3 / 4.9 ms |
| fiss | 80.1 % | 6.1 % | 1.30 m | 3.32 m | 2.1 / 5.3 ms |
| fiss_plus | 78.5 % | 6.9 % | 1.28 m | 3.98 m | 1.7 / 4.4 ms |

The replay is open loop: the ego follows the log, not the plan. A missing plan therefore mostly
means the logged driver's situation, for example a gap closing under constant-velocity prediction,
violates the planner's margins.

## CARLA

CARLA 0.9.16 runs headless and synchronously (20 Hz physics) on Town04. Every 0.1 s the planner gets
the ego state, the CARLA map as a `RoadMap`, with lane-change rules from the road markings and the route
from CARLA's `GlobalRoutePlanner`, and all actors with constant-velocity predictions. The controller
tracks the plan at 20 Hz. Collisions and lane invasions come from CARLA sensors.

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

| Scenario | Planner | Collisions | Plan success | Emergency | Mean speed | Plan time |
|---|---|---:|---:|---:|---:|---:|
| lead vehicle | fop | 0 | 99.7 % | 0.3 % | 11.9 m/s | 4.60 ms |
| lead vehicle | fop_plus | 0 | 100.0 % | 0.0 % | 12.1 m/s | 3.18 ms |
| lead vehicle | fiss | 0 | 100.0 % | 0.0 % | 12.1 m/s | 0.26 ms |
| lead vehicle | fiss_plus | 0 | 97.8 % | 0.3 % | 11.9 m/s | 0.78 ms |
| parked car | fop | 0 | 100.0 % | 0.0 % | 12.1 m/s | 4.57 ms |
| parked car | fop_plus | 0 | 100.0 % | 0.0 % | 12.2 m/s | 3.17 ms |
| parked car | fiss | 0 | 100.0 % | 0.0 % | 12.0 m/s | 0.27 ms |
| parked car | fiss_plus | 0 | 98.1 % | 0.0 % | 12.3 m/s | 0.78 ms |
| cut-in (scripted) | fop | 0 | 99.3 % | 0.7 % | 11.2 m/s | 4.66 ms |
| cut-in (scripted) | fop_plus | 0 | 97.7 % | 2.3 % | 9.7 m/s | 3.12 ms |
| cut-in (scripted) | fiss | 0 | 93.7 % | 5.0 % | 8.7 m/s | 0.81 ms |
| cut-in (scripted) | fiss_plus | 0 | 97.0 % | 3.0 % | 9.4 m/s | 1.04 ms |
| traffic (49 TM vehicles, 60 s) | fop | 0 | 98.7 % | 1.2 % | 13.4 m/s | 7.36 ms |
| traffic (49 TM vehicles, 60 s) | fop_plus | 0 | 95.7 % | 2.7 % | 12.2 m/s | 3.22 ms |
| traffic (49 TM vehicles, 60 s) | fiss | 0 | 99.3 % | 0.7 % | 13.0 m/s | 0.37 ms |
| traffic (49 TM vehicles, 60 s) | fiss_plus | 0 | 98.0 % | 1.3 % | 13.0 m/s | 0.65 ms |

### Controllers (FISS+ planner)

| Scenario | Controller | Collisions | Lateral error RMS | Along-track lag RMS |
|---|---|---:|---:|---:|
| cruise | mpc | 0 | 1.5 cm | 1.02 m |
| cruise | stanley_pid | 0 | 11.8 cm | 0.85 m |
| static obstacle | mpc | 0 | 1.4 cm | 1.02 m |
| static obstacle | stanley_pid | 0 | 12.0 cm | 0.85 m |
| traffic | mpc | 0 | 1.3 cm | 0.91 m |
| traffic | stanley_pid | 0 | 8.6 cm | 0.88 m |

Lane invasions are not shown in the tables because evasive lane changes and overtakes trigger them by design. They are
included in the JSON files. The along-track lag is mostly the 0.05 s latency compensation in the
stitching plus the simple throttle/brake mapping of the runner (`Actuation`); lateral tracking is the
controller's job, and the MPC holds it to about 1.5 cm.

