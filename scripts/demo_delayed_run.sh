#!/usr/bin/env bash
# Replays one delayed run in the terminal so the robots can be watched waiting
# for each other. Configure with the LOTSE_DEMO_* environment variables.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

MAP="${LOTSE_DEMO_MAP:-maps/warehouse_rooms.map}"
FLEET="${LOTSE_DEMO_FLEET:-6}"
DELAY="${LOTSE_DEMO_DELAY:-0.25}"
SEED="${LOTSE_DEMO_SEED:-7}"
ANIMATE="${LOTSE_DEMO_ANIMATE:-1}"

ANIMATE_FLAG=""
if [[ "${ANIMATE}" == "1" ]]; then
    ANIMATE_FLAG="--animate"
fi

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target replay_demo
    .build/bench/apps/replay_demo --map '${MAP}' --fleet '${FLEET}' \
        --delay '${DELAY}' --seed '${SEED}' ${ANIMATE_FLAG}
"
