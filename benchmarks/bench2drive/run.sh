#!/usr/bin/env bash
# Bench2Drive closed-loop evaluation of the planner (inside `pixi run -e carla`, CARLA_ROOT set):
#   benchmarks/bench2drive/run.sh <routes> <out_dir> [profile] [routes_subset] [port]
# routes: bench2drive_0.0.4_val (tuning) or bench2drive220 (test), from leaderboard/data/;
# routes_subset: route ids, e.g. 26537 or 26537,26601; the evaluator starts CARLA itself.
set -euo pipefail
routes=$1 out=$(realpath -m "$2") profile=${3:-bench2drive} subset=${4:-} port=${5:-20000}
b2d="${BENCH2DRIVE_ROOT:-$PIXI_PROJECT_ROOT/data/cache/bench2drive/Bench2Drive}"
: "${CARLA_ROOT:?set CARLA_ROOT to the CARLA 0.9.15 directory}"
mkdir -p "$out"
export PYTHONPATH="$PIXI_PROJECT_ROOT/python:$b2d/leaderboard:$b2d/scenario_runner:$CARLA_ROOT/PythonAPI/carla"
export SCENARIO_RUNNER_ROOT="$b2d/scenario_runner" LEADERBOARD_ROOT="$b2d/leaderboard"
export IS_BENCH2DRIVE=True SAVE_PATH="$out" CHALLENGE_TRACK_CODENAME=SENSORS PYTHONUNBUFFERED=1
cd "$b2d"
python leaderboard/leaderboard/leaderboard_evaluator.py \
  --routes="leaderboard/data/$routes.xml" ${subset:+--routes-subset=$subset} --repetitions=1 --track=SENSORS \
  --checkpoint="$out/results.json" --debug-checkpoint="$out/live.txt" \
  --agent="$PIXI_PROJECT_ROOT/python/fiss_planner/carla/leaderboard_agent.py" --agent-config="$profile" \
  --debug=0 --resume=True --port="$port" --traffic-manager-port=$((port + 1000)) --gpu-rank=0
