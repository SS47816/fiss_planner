# nuPlan

Closed-loop nuPlan simulations of the planner (`fiss_planner.nuplan.FissPlanner`, profile
`nuplan`) and the baselines PDM-Closed (tuplan_garage) and IDM (nuplan-devkit).

## Setup

1. Download from https://www.nuscenes.org/nuplan#download (account and licence required):
   `nuplan-maps-v1.0.zip`, `nuplan-v1.1_mini.zip` and, for Val14, `nuplan-v1.1_val.zip`.
2. Unpack them under `$NUPLAN_ROOT/dataset` (default `NUPLAN_ROOT`: `data/cache/nuplan`):

   ```
   $NUPLAN_ROOT/dataset/maps/nuplan-maps-v1.0.json, us-ma-boston/, ...
   $NUPLAN_ROOT/dataset/nuplan-v1.1/splits/mini/*.db
   ```
3. `pixi run -e nuplan nuplan-fetch` clones nuplan-devkit and tuplan_garage at pinned commits
   into `$NUPLAN_ROOT/src`, and `pixi run -e nuplan py-build` builds the planner for Python 3.9.

## Runs

```bash
pixi run -e nuplan bash benchmarks/nuplan/simulate.sh fiss_planner closed_loop_nonreactive_agents mini_dev14 \
    worker=ray_distributed worker.threads_per_node=16
pixi run -e nuplan python benchmarks/nuplan/summarize.py $NUPLAN_ROOT/exp/exp/<experiment>/<challenge>/<time>
```

Planners: `fiss_planner`, `pdm_closed_planner`, `idm_planner`. Challenges:
`closed_loop_nonreactive_agents` (CLS-NR) and `closed_loop_reactive_agents` (CLS-R).

**Splits.** `mini_dev14` (tuning) applies the Val14 scenario types and filter to the 44 nuPlan-mini
logs from the train split; the mini logs from the val and test splits are left out, so it shares no
log with Val14 or Test14. Val14 and Test14-hard are test splits.

Set `FISS_NUPLAN_TRACE=<dir>` to write a JSON trace per scenario (route, lanes, mode, rejections and
plans per cycle) for debugging.
