#!/usr/bin/env bash
# Evaluate routes one by one, each in its own directory with a time limit and one retry
# (CARLA occasionally hangs); finished routes are skipped (inside `pixi run -e carla`):
#   benchmarks/bench2drive/run_routes.sh <routes> <out_dir> [profile] [route ids ...]
set -uo pipefail
routes=$1 out=$2 profile=${3:-bench2drive}
shift 3 2>/dev/null || shift $#
here="$(cd "$(dirname "$0")" && pwd)"
b2d="${BENCH2DRIVE_ROOT:-$PIXI_PROJECT_ROOT/data/cache/bench2drive/Bench2Drive}"
ids=("$@")
mkdir -p "$out"
[ ${#ids[@]} -gt 0 ] || mapfile -t ids < <(grep -o '<route id="[0-9]*"' "$b2d/leaderboard/data/$routes.xml" | grep -o '[0-9]*')
for id in "${ids[@]}"; do
  dir="$out/$id"
  for attempt in 1 2; do
    if [ -f "$dir/results.json" ] && grep -q '"route_id"' "$dir/results.json"; then break; fi
    rm -rf "$dir"
    timeout --kill-after=30 "${ROUTE_TIMEOUT:-900}" bash "$here/run.sh" "$routes" "$dir" "$profile" "$id" > "$dir.log" 2>&1
    # a timed-out evaluator leaves its CARLA server behind
    for p in $(ps -eo pid,args | awk -v port="carla-rpc-port=${PORT:-20000}" '$0 ~ port && /CarlaUE4-Linux/ {print $1}'); do kill -9 "$p"; done
  done
  echo "$id $(grep -c '"route_id"' "$dir/results.json" 2>/dev/null || echo 0)"
done
