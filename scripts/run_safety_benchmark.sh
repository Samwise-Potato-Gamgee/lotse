#!/usr/bin/env bash
# Builds and runs the safety benchmark inside the development image, both with
# the pass-order rule and without it, and writes the raw results and the
# combined summary into results/ in the repository.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAPS="${LOTSE_BENCH_MAPS:-maps/warehouse_small.map,maps/warehouse_aisles.map,maps/warehouse_rooms.map,maps/warehouse_open.map}"
FLEET="${LOTSE_BENCH_FLEET:-2,5,10,15,20}"
DELAYS="${LOTSE_BENCH_DELAYS:-0,0.05,0.1,0.2,0.3}"
SEEDS="${LOTSE_BENCH_SEEDS:-10}"
JOBS="${LOTSE_BENCH_JOBS:-0}"

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target safety_benchmark
    mkdir -p results
    rm -f results/safety_control_summary.txt
    .build/bench/apps/safety_benchmark --maps '${MAPS}' --fleet '${FLEET}' \
        --delays '${DELAYS}' --seeds '${SEEDS}' --jobs '${JOBS}' --out-dir results
"
