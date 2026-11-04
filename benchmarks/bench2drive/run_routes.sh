#!/usr/bin/env bash
# Inside pixi run -e carla: <routes> <out_dir> [profile] [route ids ...]
set -euo pipefail
exec python "$(dirname "$0")/run_routes.py" "$@"
