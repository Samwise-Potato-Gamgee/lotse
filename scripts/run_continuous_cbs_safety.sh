#!/usr/bin/env bash
# Reruns the continuous safety benchmark with CBS as the planner (prioritized
# planning is the fallback when CBS hits its time limit). The results are written
# under a "_cbs" tag so the stage 3 prioritized results are not overwritten.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAPS="${LOTSE_CONT_MAPS:-maps/warehouse_small.map,maps/warehouse_rooms.map,maps/warehouse_aisles.map,maps/warehouse_open.map,maps/warehouse_large.map}"
FLEET="${LOTSE_CONT_FLEET:-2,5,10,15,20}"
DELAYS="${LOTSE_CONT_CBS_DELAYS:-0,0.1}"
SEEDS="${LOTSE_CONT_CBS_SEEDS:-3}"
STEPS="${LOTSE_CONT_CBS_STEPS:-400}"
ARRIVAL="${LOTSE_CONT_CBS_ARRIVAL:-2}"
JOBS="${LOTSE_CONT_JOBS:-0}"

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target continuous_benchmark
    mkdir -p results
    .build/bench/apps/continuous_benchmark --mode safety --planner cbs --tag _cbs \
        --maps '${MAPS}' --fleet '${FLEET}' --delays '${DELAYS}' \
        --seeds '${SEEDS}' --steps '${STEPS}' --arrival '${ARRIVAL}' \
        --jobs '${JOBS}' --out-dir results
"
