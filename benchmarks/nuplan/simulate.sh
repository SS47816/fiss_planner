#!/usr/bin/env bash
# Run a nuPlan closed-loop simulation (inside `pixi run -e nuplan`):
#   benchmarks/nuplan/simulate.sh <planner> <challenge> <scenario_filter> [hydra overrides ...]
# planner: fiss_planner, pdm_closed_planner, idm_planner, ...
# challenge: closed_loop_nonreactive_agents (CLS-NR) or closed_loop_reactive_agents (CLS-R)
# scenario_filter: mini_dev14 (tuning), val14_split, ... (test splits need scenario_builder=nuplan)
# e.g. benchmarks/nuplan/simulate.sh fiss_planner closed_loop_nonreactive_agents mini_dev14 worker.threads_per_node=16
set -euo pipefail
planner=$1 challenge=$2 filter=$3
shift 3
here="$(cd "$(dirname "$0")" && pwd)"
python "$NUPLAN_DEVKIT_ROOT/nuplan/planning/script/run_simulation.py" \
  +simulation="$challenge" planner="$planner" scenario_filter="$filter" scenario_builder=nuplan_mini \
  experiment_name="${planner}_${filter}" \
  "hydra.searchpath=[pkg://nuplan.planning.script.config.common,pkg://nuplan.planning.script.experiments,file://$here/config/common,file://$here/config/simulation,pkg://tuplan_garage.planning.script.config.common,pkg://tuplan_garage.planning.script.config.simulation]" \
  "$@"
