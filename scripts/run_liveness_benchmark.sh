#!/usr/bin/env bash
# Runs the core liveness benchmark inside the development image and writes the
# raw runs and summaries into results/. Every run is judged against a discrete
# baseline: the same order list in the stage 3 continuous runner, with one cell
# crossing per step. A run is slow when it does not finish within three times
# that baseline. The script compares planning from a stale snapshot (background
# on, the old real-time mode) with planning from the current positions
# (background off, the fixed mode).

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAPS="${LOTSE_LIVE_MAPS:-maps/warehouse_large.map,maps/warehouse_aisles.map,maps/warehouse_rooms.map,maps/warehouse_open.map}"
FLEET="${LOTSE_LIVE_FLEET:-4,10,20}"
CELLS="${LOTSE_LIVE_CELLS:-0.6,1.0,1.6}"
SEEDS="${LOTSE_LIVE_SEEDS:-30}"
ORDERS="${LOTSE_LIVE_ORDERS:-30}"
SLOWDOWN="${LOTSE_LIVE_SLOWDOWN:-0.3}"
STOP="${LOTSE_LIVE_STOP:-0.05}"
HORIZON="${LOTSE_LIVE_HORIZON:-1500}"
DEADLOCK="${LOTSE_LIVE_DEADLOCK:-400}"
STALL="${LOTSE_LIVE_STALL:-300}"
TICK="${LOTSE_LIVE_TICK:-0.2}"
JOBS="${LOTSE_LIVE_JOBS:-0}"

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target liveness_benchmark
    mkdir -p results
    .build/bench/apps/liveness_benchmark --maps '${MAPS}' --fleet '${FLEET}' \
        --cell-seconds '${CELLS}' --seeds '${SEEDS}' --orders '${ORDERS}' \
        --slowdown '${SLOWDOWN}' --stop '${STOP}' --horizon '${HORIZON}' \
        --deadlock '${DEADLOCK}' --stall '${STALL}' --tick '${TICK}' \
        --background on --gap on --jobs '${JOBS}' \
        --out-dir results --tag '_stale_plans'
    .build/bench/apps/liveness_benchmark --maps '${MAPS}' --fleet '${FLEET}' \
        --cell-seconds '${CELLS}' --seeds '${SEEDS}' --orders '${ORDERS}' \
        --slowdown '${SLOWDOWN}' --stop '${STOP}' --horizon '${HORIZON}' \
        --deadlock '${DEADLOCK}' --stall '${STALL}' --tick '${TICK}' \
        --background off --gap on --jobs '${JOBS}' \
        --out-dir results --tag '_current_plans'
"
