#!/usr/bin/env bash
# Downloads the public MAPF benchmark data if needed, then runs the planner
# comparison (prioritized planning against CBS) inside the development image and
# writes the raw results and summary into results/ in the repository.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

"${REPO_ROOT}/scripts/download_mapf_benchmark.sh"

MAPS="${LOTSE_MAPF_MAPS:-random-32-32-20,room-32-32-4,maze-32-32-2,empty-32-32}"
ROBOTS="${LOTSE_MAPF_ROBOTS:-5,10,15,20,25}"
SCENARIOS="${LOTSE_MAPF_SCENARIOS:-10}"
TIME_LIMIT_MS="${LOTSE_MAPF_TIME_LIMIT_MS:-5000}"
JOBS="${LOTSE_MAPF_JOBS:-0}"

lotse::build_image
lotse::run bash -lc "
    set -euo pipefail
    cmake -S . -B .build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release
    cmake --build .build/bench --target planner_comparison
    mkdir -p results
    .build/bench/apps/planner_comparison --data-dir .bench-data/mapf \
        --maps '${MAPS}' --robots '${ROBOTS}' --scenarios '${SCENARIOS}' \
        --time-limit-ms '${TIME_LIMIT_MS}' --jobs '${JOBS}' \
        --out-dir results --lotse-hard
"
