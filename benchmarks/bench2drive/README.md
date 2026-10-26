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

The evaluator starts CARLA itself. **Splits:** `bench2drive_0.0.4_val` (220 routes, 44 scenario
types x 5) for tuning, `bench2drive220` (the 220 benchmark routes) as the test split.
**Metrics:** Driving Score (route completion x infraction penalty) and Success Rate (route
completed without infractions other than minimum speed), as in Bench2Drive's `tools/`.
