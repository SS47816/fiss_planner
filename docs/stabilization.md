# Stabilization handoff — 3 November 2026

The starting revision is `466b1bcc4d4d99209dea8522dd6a14aa43bf4168`.
The stabilization changes are submitted for review; release and package publication
remain separate work.

## Changes

- Pin Pixi 0.82.1 in the manifest and CI. Map conda `ray-core` only to PyPI `ray`:
  the previous hash mapping also claimed bundled packages such as `aiohttp` at
  Ray's version, invalidating the lockfile whenever it was checked.
- Run ROS 2 CI in the official Humble/Jammy and Jazzy/Noble ROS base images,
  retaining both build/test jobs. This removes the setup action's unauthenticated
  latest-release lookup and ROS repository-package download, which failed with
  HTTP 404 before the Humble build. The images already provide ROS, colcon and
  rosdep; dependency installation still runs for the repository's package manifests.
- Keep base profile tests independent of CommonRoad and run the optional integration
  test explicitly in CommonRoad CI. All four profiles are checked in installed wheels.
- Recheck oncoming clearance until the measured footprint leaves the lane, including
  when stitching projects a future departure. Remember the blockers of a committed
  pass, discard that state on return/reset/map replacement, and require fresh
  clearance for a different blockage. After passing the blockers, retain oncoming
  access until the measured footprint has merged back, while reopening the home
  lane for that return. Collision checking remains active.
- Exercise parking recovery through LocalPlanner: entry, blocked exit, yielding to
  moving traffic, resumption and handover. These tests use deterministic synthetic maps.
- Run Bench2Drive routes under a Linux child subreaper, including detached CARLA
  sessions. Finished driving failures remain valid results; incomplete checkpoints
  retry once. Attempt logs and incomplete directories are retained, with archives
  outside the directory scanned by the result summarizer.

## Validation

Local results:

| Check | Result |
|---|---|
| C++ core | 164 passed |
| ASan / UBSan | 164 passed |
| Python with CommonRoad | 38 passed, including 15 runner tests |
| Installed Python 3.13 wheel without CommonRoad | 37 passed, 1 optional integration test skipped |
| ROS 1 / ROS 2 | 3 / 4 tests, zero failures |
| Locked installs | All six environments passed |
| Documentation | Strict MkDocs build passed |
| CommonRoad development smoke | 3 / 3 goals, no collisions or road departures |
| Bench2Drive validation smoke | Parking exit completed; construction route timed out at 31.64%, no collisions |

Reproduce the checks from the repository root with Pixi 0.82.1:

```bash
pixi lock --check
pixi install --locked -e default -e bench -e carla -e nuplan -e jazzy -e noetic
pixi run test
pixi run test-asan
pixi run -e bench py-test
pixi run -e noetic ros1-test
pixi run -e jazzy ros2-test
pixi run -e bench python -m fiss_planner.commonroad.benchmark \
  --scenarios benchmarks/commonroad/dev_set.txt --limit 3 --seed 0 \
  --planners fiss_plus --jobs 2 --out results/stabilization/commonroad-verified.csv
```

The wheel check uses a separate Python 3.13 virtual environment, builds with
`python -m build --wheel`, installs the wheel and pytest, then runs `python -m pytest`
against the repository's `python/tests` from `/tmp`. CommonRoad must be absent:
37 tests pass and its single integration test skips. Optimizers and all four
profile loads are checked explicitly from the installed package.

The local CommonRoad smoke run completes all three development scenarios with
zero collisions or road departures. Resolved settings and source fingerprints
are recorded locally in `results/stabilization/validation-manifest.json` and
`*-resolved.yaml`; this small sample is a
regression check, not a new performance claim.

The local validation above precedes PR submission. Remote Actions results are
tracked on the stabilization PR. The previous compiler/sanitizer and ROS 2 jobs
passed, while Pixi setup and Python collection were the confirmed failures
addressed here. Local results and the remote matrix are separate evidence.

## Remaining benchmark work

The completed 30-trial Bench2Drive study used 42 validation routes in a separate
`fiss_tune` checkout. Its best tuning Driving Score was 75.41 (base 69.28); those
are tuning scores. The [candidate record](https://github.com/SS47816/fiss_planner/blob/main/benchmarks/results/bench2drive_tuning_candidate.json)
retains parameters and provenance. No code revision or independent check split
was recorded, so it has not replaced the shipped profile.

Passing-state continuity assumes stable obstacle IDs. Permission is not a
collision-free guarantee: trajectory validation may still hold or brake for
traffic. The synthetic recovery tests and two focused CARLA routes do not
establish general Bench2Drive success. Full tuning, held-out Bench2Drive/nuPlan
evaluation and release preparation remain separate work.

The focused `ConstructionObstacle` validation route `2520` still stops progressing:
it reaches 31.64% before `Failed - TickRuntime` at about 200 simulator seconds,
with no collision or road-departure infraction. Its trace shows repeated
near-zero-speed plans while `pass_blocked` is active. This remains a
sampling/cost/profile investigation; the passing-state fixes do not establish
that this scenario is solved. Local trace: `results/stabilization/bench2drive-traces/`.
Parking-exit route `26537` completes with Driving Score 100; minimum-speed
infractions are still recorded. The two-route smoke mean is Driving Score 65.82
and 50% success under the benchmark's definition that excludes minimum-speed infractions.
These are development validation routes. `FISS_B2D_TRACE` enables candidate
recording in the verified run; the configuration hash excludes this diagnostic setting.

Historical gallery and CARLA performance results use 0.9.16. Current development
and the focused Bench2Drive runs use 0.9.15; do not compare them as one protocol.
