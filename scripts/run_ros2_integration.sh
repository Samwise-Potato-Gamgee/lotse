#!/usr/bin/env bash
# Runs the stage 6 integration matrix inside the development image: fleets of 4
# and 10 robots on the warehouse maps with injected slowdowns over three seeds,
# plus one obstacle scenario. Writes results/ros2_integration.txt.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

lotse::build_image
mapfile -d '' -t args < <(lotse::_docker_args)
docker run --rm --env "LOTSE_ROS_SKIP_LARGE=${LOTSE_ROS_SKIP_LARGE:-0}" \
    "${args[@]}" "${LOTSE_DEV_IMAGE}" bash -lc './scripts/ros2_integration_inner.sh'
