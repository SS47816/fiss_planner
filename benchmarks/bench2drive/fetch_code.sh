#!/usr/bin/env bash
# Clone Bench2Drive (leaderboard, scenario runner, routes) at the pinned commit into $BENCH2DRIVE_ROOT.
set -euo pipefail
root="${BENCH2DRIVE_ROOT:-${PIXI_PROJECT_ROOT:-.}/data/cache/bench2drive/Bench2Drive}"
[ -d "$root/.git" ] || git clone -q https://github.com/Thinklab-SJTU/Bench2Drive.git "$root"
git -C "$root" fetch -q --depth 1 origin 7ec25d1c9f7522d923ce5f3420986cef1cb2d956 2>/dev/null || true
git -C "$root" checkout -q 7ec25d1c9f7522d923ce5f3420986cef1cb2d956
echo "Bench2Drive in $root"
# Python >= 3.9 (the CARLA environment uses 3.10): ElementTree.getchildren() was removed.
grep -rl "getchildren()" "$root/leaderboard" "$root/scenario_runner" --include=*.py | xargs -r sed -i 's/\([A-Za-z_]*\)\.getchildren()/list(\1)/g'
