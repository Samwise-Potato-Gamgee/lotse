#!/usr/bin/env bash
# One command for the recorded Isaac Sim demo with the labelled overlay. It runs
# the seed-1 order list, writes the results, encodes the full video, the sped-up
# clip, the waiting GIF and the stills, and checks that no lotse- container is
# left behind. The recording goes to .build/isaac/run-1-video so the four
# unrecorded result runs stay untouched.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SEED="${1:-1}"

LOTSE_ISAAC_RUN_NAME="run-${SEED}-video" ./scripts/run_isaac_demo.sh "${SEED}" normal 1

LEFTOVER="$(docker ps -a --format '{{.Names}}' | grep '^lotse-' || true)"
if [[ -n "${LEFTOVER}" ]]; then
    echo "run_isaac_video.sh FAILED: lotse- containers left behind:" >&2
    echo "${LEFTOVER}" >&2
    exit 1
fi

echo "run_isaac_video.sh PASSED: no lotse- container left behind."
