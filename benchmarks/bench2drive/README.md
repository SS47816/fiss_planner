# Bench2Drive

Closed-loop CARLA Leaderboard 2.0 routes of [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive)
driven by the planner as a privileged agent (`python/fiss_planner/carla/leaderboard_agent.py`,
profile `bench2drive`): the map around the route, the other actors and the traffic lights come from
the simulator, as for the rule-based expert PDM-Lite.

## Setup

1. CARLA 0.9.15 with the additional maps (Town12/13/15):
   `CARLA_0.9.15.tar.gz` and `AdditionalMaps_0.9.15.tar.gz` (put the latter in `Import/` and run
   `ImportAssets.sh`) from https://github.com/carla-simulator/carla/releases/tag/0.9.15.
2. `pixi run -e carla bash benchmarks/bench2drive/fetch_code.sh` clones Bench2Drive at the pinned
   commit into `data/cache/bench2drive/Bench2Drive` (or `$BENCH2DRIVE_ROOT`) and adapts it to
   Python 3.10. `pixi run -e carla py-build` builds the planner for the environment.

## Runs

```bash
CARLA_ROOT=/path/to/CARLA_0.9.15 pixi run -e carla bash benchmarks/bench2drive/run.sh \
    bench2drive_0.0.4_val results/b2d/val [profile] [route ids, e.g. 26537,11658]
pixi run -e bench python benchmarks/bench2drive/summarize.py results/b2d/val --routes
```

The evaluator starts CARLA itself. `run_routes.sh` evaluates routes one by one, each in its own
directory with a time limit and a retry, on the CARLA port `$PORT` (default 20000; run several on
ports 4000 apart). **Splits:** `bench2drive_0.0.4_val` (220 routes over 42 scenario types) for
tuning, `bench2drive220` (the 220 benchmark routes) as the test split.
**Metrics:** Driving Score (route completion x infraction penalty) and Success Rate (route
completed without infractions other than minimum speed), as in Bench2Drive's `tools/`.

## Tuning

```bash
CARLA_ROOT=/path/to/CARLA_0.9.15 pixi run -e bench python benchmarks/tune.py \
    benchmarks/tuning/bench2drive.yaml --jobs 4 --trials 30 --out results/tune/bench2drive
```

Each trial drives `tune_routes.txt` (the first validation route of every scenario type) on `--jobs`
CARLA servers (ports `$B2D_PORT`, default 30000, 2000 apart); a route without a result scores 0.
The planner code is loaded from the working tree, so leave it unchanged while a study runs.

## Resume and process cleanup

`run_routes.sh` delegates to a Linux Python supervisor. It skips a checkpoint
only when `entry_status` is `Finished`, progress is `[1, 1]`, and the matching
route record is terminal. Completed driving failures (including infractions or
blocked vehicles) remain results. Malformed, partial, crashed and rejected
checkpoints are retried once; incomplete evaluations make the runner exit nonzero.

Each attempt has its own log. Incomplete route directories move to a sibling
`<out_dir>.incomplete/` directory so they remain inspectable and are excluded
from result aggregation. The supervisor reaps only its descendants, including
CARLA servers that start new sessions, after completion, timeout or interruption.
Existing servers on other invocations are left alone.

Test this without CARLA: `pixi run -e bench pytest -q python/tests/test_bench2drive_runner.py`.
