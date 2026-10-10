#!/usr/bin/env bash
# Stops the Lotse service started by run_service.sh. The order database lives in
# the lotse-data volume, so it survives a stop and start.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

docker compose -f "${REPO_ROOT}/docker-compose.yml" down "$@"
