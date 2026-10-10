#!/usr/bin/env bash
# Starts the Lotse service with the grid simulator through Docker Compose and
# waits until the health check answers. This is the one command for stage 5.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

lotse::build_image

echo "building and starting lotse-service ..."
docker compose -f "${REPO_ROOT}/docker-compose.yml" up --build -d

for attempt in $(seq 1 60); do
    if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then
        echo "lotse service is up"
        curl -fsS http://127.0.0.1:8080/health
        echo
        echo "dashboard: http://127.0.0.1:8080/"
        exit 0
    fi
    sleep 1
done

echo "lotse service did not become healthy in time" >&2
docker compose -f "${REPO_ROOT}/docker-compose.yml" logs --tail 30 >&2
exit 1
