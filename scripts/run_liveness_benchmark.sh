#!/usr/bin/env bash
# Runs the liveness benchmark inside the development image, once with the
# one-step gap planning option off and once on, and writes the raw runs and the
# stall summaries into results/ in the repository.

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
BACKGROUND="${LOTSE_LIVE_BACKGROUND:-off}"
JOBS="${LOTSE_LIVE_JOBS:-0}"

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target liveness_benchmark
    mkdir -p results
    for gap in off on; do
        .build/bench/apps/liveness_benchmark --maps '${MAPS}' --fleet '${FLEET}' \
            --cell-seconds '${CELLS}' --seeds '${SEEDS}' --orders '${ORDERS}' \
            --slowdown '${SLOWDOWN}' --stop '${STOP}' --horizon '${HORIZON}' \
            --deadlock '${DEADLOCK}' --stall '${STALL}' --tick '${TICK}' \
            --background '${BACKGROUND}' --gap \"\${gap}\" --jobs '${JOBS}' \
            --out-dir results --tag \"_\${gap}\"
    done
"
