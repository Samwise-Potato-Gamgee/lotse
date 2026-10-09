#!/usr/bin/env bash
# Runs the whole continuous-mode benchmark suite: the safety benchmark with the
# pass-order control, the throughput benchmark at two loads, and the
# CBS-against-prioritized throughput comparison. Both parts write into results/.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

"${REPO_ROOT}/scripts/run_continuous_safety_benchmark.sh"
"${REPO_ROOT}/scripts/run_throughput_benchmark.sh"
