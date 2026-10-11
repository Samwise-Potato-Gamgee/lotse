#!/usr/bin/env bash
# Runs the stage 7 Isaac Sim matrix: three seeded runs of 4 robots with a fixed
# 12-order list, plus one obstacle run (also recorded), then aggregates the
# results into results/isaac_runs.csv and results/isaac_summary.txt. Exits
# non-zero when any run misses the required result.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SEEDS="${LOTSE_ISAAC_SEEDS:-3}"
DIST="${REPO_ROOT}/.build/isaac"
OUT="${REPO_ROOT}/results/isaac_summary.txt"
RAW="${REPO_ROOT}/results/isaac_runs.csv"

if [[ "${LOTSE_ISAAC_SKIP_RUNS:-0}" != "1" ]]; then
    for seed in $(seq 1 "${SEEDS}"); do
        ./scripts/run_isaac_demo.sh "${seed}" normal 0 || true
    done
    ./scripts/run_isaac_demo.sh 1 obstacle 1 || true
fi

# Reads a numeric field from a small JSON result file, ignoring spaces.
# file: the JSON file to read
# field: the field name to extract
readValue() {
    grep -oE "\"$2\"[[:space:]]*:[[:space:]]*[0-9.]+" "$1" | head -1 \
        | sed -E 's/.*:[[:space:]]*//'
}

# Reads a true/false field from a small JSON result file.
# file: the JSON file to read
# field: the field name to extract
readBool() {
    grep -o "\"$2\":\(true\|false\)" "$1" | head -1 | cut -d: -f2
}

mkdir -p "${REPO_ROOT}/results"
echo "run,seed,obstacle,orders,completed,judge_contacts,sim_contacts,deadlocked,grid,min_distance,sim_seconds,wall_seconds,stopped_frames,status" > "${RAW}"

FAILURES=0
for spec in "seed1:1:no:run-1-normal" "seed2:2:no:run-2-normal" \
    "seed3:3:no:run-3-normal" "obstacle:1:yes:run-1-obstacle"; do
    name="${spec%%:*}"
    rest="${spec#*:}"
    seed="${rest%%:*}"
    rest="${rest#*:}"
    obstacle="${rest%%:*}"
    dir="${DIST}/${rest#*:}"
    fleet="${dir}/fleet.json"
    judge="${dir}/judge.json"
    scene="${dir}/scene.json"
    if [[ ! -f "${fleet}" || ! -f "${judge}" || ! -f "${scene}" ]]; then
        printf '%s,%s,%s,0,0,-1,-1,true,-1,0,0,0,0,MISSING\n' \
            "${name}" "${seed}" "${obstacle}" >> "${RAW}"
        FAILURES=$((FAILURES + 1))
        continue
    fi
    orders=$(readValue "${fleet}" orders_released)
    completed=$(readValue "${fleet}" orders_completed)
    judge_contacts=$(readValue "${judge}" contacts)
    sim_contacts=$(readValue "${scene}" contacts)
    deadlocked=$(readBool "${fleet}" deadlocked)
    grid=$(readValue "${fleet}" collisions)
    min_distance=$(readValue "${judge}" min_distance)
    sim_seconds=$(readValue "${fleet}" sim_seconds)
    wall_seconds=$(readValue "${fleet}" wall_seconds)
    stopped=$(readValue "${fleet}" stopped_frames)
    status="ok"
    if [[ "${completed}" -lt "${orders}" || "${judge_contacts}" -ne 0 ||
        "${sim_contacts}" -ne 0 || "${deadlocked}" != "false" || "${grid}" -ne 0 ]]; then
        status="FAIL"
    fi
    if [[ "${obstacle}" == "yes" && "${stopped}" -eq 0 ]]; then
        status="FAIL"
    fi
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "${name}" "${seed}" "${obstacle}" "${orders}" "${completed}" \
        "${judge_contacts}" "${sim_contacts}" "${deadlocked}" "${grid}" \
        "${min_distance}" "${sim_seconds}" "${wall_seconds}" "${stopped}" "${status}" >> "${RAW}"
    if [[ "${status}" != "ok" ]]; then
        FAILURES=$((FAILURES + 1))
    fi
done

{
    echo "Stage 7 Isaac Sim runs"
    echo "Command: ./scripts/run_isaac_runs.sh"
    echo "4 Carter robots, warehouse_small, 12 orders, forward raycast scan, simulator clock."
    echo "Contacts are the independent judge on true poses and the simulator's own reports."
    echo
    printf '%-10s %5s %6s %5s %8s %6s %8s %7s %9s %10s %8s %6s\n' \
        "run" "seed" "orders" "done" "judge" "sim" "deadlock" "grid" "closest" "sim_sec" "wall_sec" "status"
    awk -F, 'NR > 1 {
        printf "%-10s %5s %6s %5s %8s %6s %8s %7s %9.3f %10.1f %8.1f %6s\n",
            $1, $2, $4, $5, $6, $7, $8, $9, $10, $11, $12, $14
    }' "${RAW}"
    echo
    echo "failures: ${FAILURES}"
} > "${OUT}"

cat "${OUT}"
if [[ "${FAILURES}" -ne 0 ]]; then
    exit 1
fi
