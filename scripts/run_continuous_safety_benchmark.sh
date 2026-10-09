#!/usr/bin/env bash
# Runs only the continuous-mode safety benchmark (with the pass-order control)
# and writes the raw results and the summary into results/ in the repository.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAPS="${LOTSE_CONT_MAPS:-maps/warehouse_small.map,maps/warehouse_rooms.map,maps/warehouse_aisles.map,maps/warehouse_open.map,maps/warehouse_large.map}"
FLEET="${LOTSE_CONT_FLEET:-2,5,10,15,20}"
DELAYS="${LOTSE_CONT_SAFETY_DELAYS:-0,0.1,0.3}"
SEEDS="${LOTSE_CONT_SEEDS:-4}"
STEPS="${LOTSE_CONT_SAFETY_STEPS:-600}"
ARRIVAL="${LOTSE_CONT_SAFETY_ARRIVAL:-2}"
JOBS="${LOTSE_CONT_JOBS:-0}"

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target continuous_benchmark
    mkdir -p results
    .build/bench/apps/continuous_benchmark --mode safety --planner prioritized \
        --maps '${MAPS}' --fleet '${FLEET}' --delays '${DELAYS}' \
        --seeds '${SEEDS}' --steps '${STEPS}' --arrival '${ARRIVAL}' \
        --jobs '${JOBS}' --out-dir results
"
