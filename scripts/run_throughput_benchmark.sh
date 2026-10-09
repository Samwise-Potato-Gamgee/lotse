#!/usr/bin/env bash
# Runs the continuous-mode throughput benchmark at two loads, and the
# CBS-against-prioritized throughput comparison, writing the raw results and
# summaries into results/ in the repository.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAPS="${LOTSE_CONT_MAPS:-maps/warehouse_small.map,maps/warehouse_rooms.map,maps/warehouse_aisles.map,maps/warehouse_open.map,maps/warehouse_large.map}"
FLEET="${LOTSE_CONT_FLEET:-2,5,10,15,20}"
DELAYS="${LOTSE_CONT_THROUGHPUT_DELAYS:-0,0.1}"
SEEDS="${LOTSE_CONT_SEEDS:-4}"
SATURATED_STEPS="${LOTSE_CONT_THROUGHPUT_STEPS:-600}"
SATURATED_ARRIVAL="${LOTSE_CONT_THROUGHPUT_ARRIVAL:-1}"
BELOW_STEPS="${LOTSE_CONT_BELOW_STEPS:-1500}"
BELOW_ARRIVAL="${LOTSE_CONT_BELOW_ARRIVAL:-40}"
COMPARE_STEPS="${LOTSE_CONT_COMPARE_STEPS:-400}"
JOBS="${LOTSE_CONT_JOBS:-0}"

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target continuous_benchmark
    mkdir -p results
    .build/bench/apps/continuous_benchmark --mode throughput --planner prioritized \
        --maps '${MAPS}' --fleet '${FLEET}' --delays '${DELAYS}' --seeds '${SEEDS}' \
        --steps '${SATURATED_STEPS}' --arrival '${SATURATED_ARRIVAL}' \
        --jobs '${JOBS}' --out-dir results
    .build/bench/apps/continuous_benchmark --mode throughput --planner prioritized \
        --tag _below --maps '${MAPS}' --fleet '${FLEET}' --delays '${DELAYS}' \
        --seeds '${SEEDS}' --steps '${BELOW_STEPS}' --arrival '${BELOW_ARRIVAL}' \
        --jobs '${JOBS}' --out-dir results
    .build/bench/apps/continuous_benchmark --mode throughput-compare \
        --maps '${MAPS}' --fleet '${FLEET}' --delays 0 --seeds '${SEEDS}' \
        --steps '${COMPARE_STEPS}' --arrival '${SATURATED_ARRIVAL}' \
        --jobs '${JOBS}' --out-dir results
"
