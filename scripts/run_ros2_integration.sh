#!/usr/bin/env bash
# Runs the stage 6 integration matrix inside the development image: fleets of 4
# and 10 robots on the warehouse maps with injected slowdowns over three seeds,
# plus one obstacle scenario. Writes results/ros2_integration.txt.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

lotse::build_image
lotse::run bash -lc './scripts/ros2_integration_inner.sh'
