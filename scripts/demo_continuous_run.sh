#!/usr/bin/env bash
# Replays a short continuous run in the terminal so orders, pickups, drop-offs
# and idle robots parking can be watched. Configure with LOTSE_CONT_* variables.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAP="${LOTSE_CONT_MAP:-maps/warehouse_small.map}"
FLEET="${LOTSE_CONT_FLEET:-4}"
ORDERS="${LOTSE_CONT_ORDERS:-8}"
DELAY="${LOTSE_CONT_DELAY:-0.2}"
SEED="${LOTSE_CONT_SEED:-7}"
STEPS="${LOTSE_CONT_STEPS:-200}"
ANIMATE="${LOTSE_CONT_ANIMATE:-1}"

ANIMATE_FLAG=""
if [[ "${ANIMATE}" == "1" ]]; then
    ANIMATE_FLAG="--animate"
fi

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target continuous_demo
    .build/bench/apps/continuous_demo --map '${MAP}' --fleet '${FLEET}' \
        --orders '${ORDERS}' --delay '${DELAY}' --seed '${SEED}' --steps '${STEPS}' \
        ${ANIMATE_FLAG}
"
