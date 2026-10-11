#!/usr/bin/env bash
# Runs the A1 clock test inside the development image: the same seeded stand-in
# scenario at a slow and a fast time scale must give the same outcome measured
# in simulated time. Builds the packages and writes results/clock_test.txt.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

lotse::build_image
lotse::run bash -lc './scripts/clock_test_inner.sh'
