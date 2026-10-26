#!/usr/bin/env bash
# Clone nuplan-devkit and tuplan_garage (PDM-Closed) at the pinned commits into $NUPLAN_ROOT/src.
set -euo pipefail
src="${NUPLAN_ROOT:?run inside the pixi nuplan environment}/src"
mkdir -p "$src"
fetch() {  # url dir commit
  [ -d "$src/$2/.git" ] || git clone -q "$1" "$src/$2"
  git -C "$src/$2" fetch -q --depth 1 origin "$3" 2>/dev/null || true
  git -C "$src/$2" checkout -q "$3"
}
fetch https://github.com/motional/nuplan-devkit.git nuplan-devkit e9241677997dd86bfc0bcd44817ab04fe631405b
fetch https://github.com/autonomousvision/tuplan_garage.git tuplan_garage b51d5d04fac1bd4389653b9ab2ff73ea88f435a3
echo "nuplan-devkit and tuplan_garage in $src"
