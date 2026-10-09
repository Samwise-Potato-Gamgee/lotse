#!/usr/bin/env bash
# Downloads the public MAPF benchmark maps and random scenario files into a
# gitignored directory. The files are not committed to the repository.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${LOTSE_MAPF_DIR:-${REPO_ROOT}/.bench-data/mapf}"
MAP_URL="https://movingai.com/benchmarks/mapf/mapf-map.zip"
SCEN_URL="https://movingai.com/benchmarks/mapf/mapf-scen-random.zip"

mkdir -p "${DEST}"

if [[ ! -d "${DEST}/maps" ]]; then
    echo "downloading ${MAP_URL}"
    curl -sS -L -o "${DEST}/mapf-map.zip" "${MAP_URL}"
    unzip -q -o "${DEST}/mapf-map.zip" -d "${DEST}/maps"
fi

if [[ ! -d "${DEST}/scen-random" ]]; then
    echo "downloading ${SCEN_URL}"
    curl -sS -L -o "${DEST}/mapf-scen-random.zip" "${SCEN_URL}"
    unzip -q -o "${DEST}/mapf-scen-random.zip" -d "${DEST}"
fi

echo "MAPF benchmark data ready in ${DEST}"
