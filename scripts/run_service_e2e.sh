#!/usr/bin/env bash
# End-to-end check for stage 5: build the service image, start it with the grid
# simulator, submit orders through the HTTP API, wait until they complete and
# verify the fleet stayed collision-free and deadlock-free. The container is
# removed on exit, whatever the outcome.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

IMAGE="lotse-service:latest"
NAME="lotse-e2e"
BASE="http://127.0.0.1:18080"
ORDER_COUNT=12
DATA_DIR="${REPO_ROOT}/.build/e2e-data"

echo "== build the development image and the service image =="
lotse::build_image
docker build -f "${REPO_ROOT}/docker/lotse-service.Dockerfile" \
    -t "${IMAGE}" --build-arg "LOTSE_DEV_IMAGE=${LOTSE_DEV_IMAGE}" "${REPO_ROOT}"

# Removes the end-to-end container, whatever happened.
cleanup() {
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

rm -rf "${DATA_DIR}"
mkdir -p "${DATA_DIR}"

echo "== start the service with the grid simulator =="
docker run -d --name "${NAME}" --entrypoint /lotse/lotse_service \
    -p 127.0.0.1:18080:8080 \
    -v "${DATA_DIR}:/data" \
    "${IMAGE}" --config /lotse/config/lotse-e2e.conf --bind 0.0.0.0 >/dev/null

for attempt in $(seq 1 60); do
    if curl -fsS "${BASE}/health" >/dev/null 2>&1; then
        break
    fi
    if [[ "${attempt}" -eq 60 ]]; then
        echo "service did not become healthy" >&2
        docker logs "${NAME}" | tail -20 >&2
        exit 1
    fi
    sleep 1
done

echo "== invalid input is rejected =="
CODE=$(curl -s -o /dev/null -w '%{http_code}' -X POST "${BASE}/orders" \
    -H 'Content-Type: application/json' \
    -d '{"pickup":"does_not_exist","dropoff":"dropoff0"}')
if [[ "${CODE}" != "400" ]]; then
    echo "unknown station was not rejected (HTTP ${CODE})" >&2
    exit 1
fi

echo "== submit ${ORDER_COUNT} orders =="
PICKUPS=(pickup0 pickup1 pickup2 pickup5 pickup7 pickup9 pickup11 pickup13 pickup15 pickup17
    pickup18 pickup19)
DROPOFFS=(dropoff0 dropoff1 dropoff2 dropoff3 dropoff4 dropoff5)
for index in $(seq 0 $((ORDER_COUNT - 1))); do
    pickup="${PICKUPS[$((index % ${#PICKUPS[@]}))]}"
    dropoff="${DROPOFFS[$((index % ${#DROPOFFS[@]}))]}"
    curl -fsS -X POST "${BASE}/orders" -H 'Content-Type: application/json' \
        -d "{\"pickup\":\"${pickup}\",\"dropoff\":\"${dropoff}\",\"priority\":0}" >/dev/null
done

echo "== wait for completion =="
COMPLETED=0
for attempt in $(seq 1 600); do
    BODY=$(curl -fsS "${BASE}/throughput")
    COMPLETED=$(echo "${BODY}" | grep -o '"orders_completed":[0-9]*' | cut -d: -f2)
    OPEN=$(echo "${BODY}" | grep -o '"orders_open":[0-9]*' | cut -d: -f2)
    if [[ "${COMPLETED}" -ge "${ORDER_COUNT}" && "${OPEN}" -eq 0 ]]; then
        break
    fi
    sleep 0.5
done

FLEET=$(curl -fsS "${BASE}/fleet")
COLLISIONS=$(echo "${FLEET}" | grep -o '"collisions":[0-9]*' | head -1 | cut -d: -f2)
SWAPS=$(echo "${FLEET}" | grep -o '"swaps":[0-9]*' | head -1 | cut -d: -f2)
DEADLOCKED=$(echo "${FLEET}" | grep -o '"deadlocked":\(true\|false\)' | head -1 | cut -d: -f2)

echo "orders submitted ${ORDER_COUNT}, completed ${COMPLETED}"
echo "collisions ${COLLISIONS}, swaps ${SWAPS}, deadlocked ${DEADLOCKED}"

if [[ "${COMPLETED}" -lt "${ORDER_COUNT}" || "${COLLISIONS}" -ne 0 || "${SWAPS}" -ne 0 ||
    "${DEADLOCKED}" != "false" ]]; then
    echo "E2E FAILED" >&2
    docker logs "${NAME}" | tail -20 >&2
    exit 1
fi

echo "E2E PASSED: ${COMPLETED} orders completed with zero collisions and no deadlock"
