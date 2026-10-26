# Activation for the pixi `nuplan` environment. NUPLAN_ROOT holds dataset/ (maps and
# nuplan-v1.1/splits), src/ (nuplan-devkit and tuplan_garage, see fetch_code.sh) and exp/.
export NUPLAN_ROOT="${NUPLAN_ROOT:-$PIXI_PROJECT_ROOT/data/cache/nuplan}"
export NUPLAN_DATA_ROOT="${NUPLAN_DATA_ROOT:-$NUPLAN_ROOT/dataset}"
export NUPLAN_MAPS_ROOT="${NUPLAN_MAPS_ROOT:-$NUPLAN_ROOT/dataset/maps}"
export NUPLAN_EXP_ROOT="${NUPLAN_EXP_ROOT:-$NUPLAN_ROOT/exp}"
export NUPLAN_MAP_VERSION="nuplan-maps-v1.0"
export NUPLAN_DEVKIT_ROOT="$NUPLAN_ROOT/src/nuplan-devkit"
# PYTHONPATH (repo python/, nuplan-devkit, tuplan_garage) is set in pixi.toml.
