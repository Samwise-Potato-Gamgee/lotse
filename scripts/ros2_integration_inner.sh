#!/usr/bin/env bash
# Runs inside the development container. It builds the ROS 2 packages, then runs
# the stage 6 integration matrix: ten seeds for 4 robots on warehouse_small and
# ten seeds for 10 robots on warehouse_large, each with injected slowdowns, plus
# one obstacle scenario. Runs are spread over separate ROS domains and a few run
# at once, so the whole set fits in the time budget. The safety number is the
# independent judge's contact count from true positions; the manager's grid
# counter and its plan adoptions and rejections are reported too. Exits non-zero
# when any required result is unmet.

set -eo pipefail

source /opt/ros/jazzy/setup.bash
colcon --log-base .build/ros2/log build \
    --base-paths ros2 \
    --build-base .build/ros2/build \
    --install-base .build/ros2/install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+
source .build/ros2/install/setup.bash
set -u

OUT="results/ros2_integration.txt"
RAW="results/ros2_integration_runs.csv"
mkdir -p results
: > "${OUT}"
: > "${RAW}"
echo "setup,map,fleet,seed,obstacle,order_count,completed,contacts,min_distance,deadlocked,grid_collisions,plan_adoptions,plan_rejections,robots_used,stopped_frames,status,planning_calls,planning_p95_ms,planning_max_ms" >> "${RAW}"

MAX_PARALLEL="${LOTSE_ROS_PARALLEL:-1}"
DOMAIN_BASE="${LOTSE_ROS_DOMAIN_BASE:-40}"
SEEDS="${LOTSE_ROS_SEEDS:-10}"
SKIP_LARGE="${LOTSE_ROS_SKIP_LARGE:-0}"
AGGREGATE_ONLY="${LOTSE_ROS_AGGREGATE_ONLY:-0}"
JOB_INDEX=0
RUNNING=0
FAILURES=0

# Reads a numeric field from a small JSON result file.
readValue() {
    grep -o "\"$2\":[0-9.]*" "$1" | head -1 | cut -d: -f2
}

# Reads a true/false field from a small JSON result file.
readBool() {
    grep -o "\"$2\":\(true\|false\)" "$1" | head -1 | cut -d: -f2
}

# Runs one launch on its own ROS domain and writes the raw fields and a status.
# spec: a signal file
# setup: a label for the aggregate table
# map: the map file path
# fleet: number of robots
# seed: random seed
# obstacle: "yes" to inject an obstacle in front of robot 0
# order_count: how many orders must complete
# run_timeout: seconds before the fleet node stops itself
# domain: the ROS domain id
# dir: the directory for the result files
runCase() {
    local setup="$1" map="$2" fleet="$3" seed="$4" obstacle="$5" order_count="$6"
    local run_timeout="$7" domain="$8" dir="$9"
    mkdir -p "${dir}"
    rm -f "${dir}/fleet.json" "${dir}/judge.json"
    local extra=""
    if [[ "${obstacle}" == "yes" ]]; then
        extra="obstacle_robot:=0 obstacle_start:=4.0 obstacle_end:=9.0"
    fi
    # The stand-in runs at time_scale 3.0, so a wall budget is three times as
    # many simulated seconds; the manager's timeout now counts simulated time.
    local sim_timeout=$((run_timeout * 3))
    ROS_DOMAIN_ID="${domain}" timeout "$((run_timeout + 60))" \
        ros2 launch lotse_ros warehouse.launch.py \
        map_path:="${map}" fleet_size:="${fleet}" seed:="${seed}" \
        slowdown_probability:=0.15 stop_probability:=0.01 time_scale:=3.0 \
        order_count:="${order_count}" timeout_seconds:="${sim_timeout}" \
        result_dir:="${dir}" ${extra} > "${dir}/launch.log" 2>&1 || true

    if [[ ! -f "${dir}/fleet.json" || ! -f "${dir}/judge.json" ]]; then
        printf '%s,%s,%s,%s,%s,%s,0,-1,0,true,-1,0,0,0,0,MISSING,0,0,0\n' \
            "${setup}" "${map}" "${fleet}" "${seed}" "${obstacle}" "${order_count}" > "${dir}/fields"
        echo "FAIL" > "${dir}/status"
        return
    fi

    local completed contacts minimum stopped deadlocked grid adoptions rejections robots
    local planning_calls planning_p95 planning_max
    completed=$(readValue "${dir}/fleet.json" orders_completed)
    contacts=$(readValue "${dir}/judge.json" contacts)
    minimum=$(readValue "${dir}/judge.json" min_distance)
    stopped=$(readValue "${dir}/fleet.json" stopped_frames)
    deadlocked=$(readBool "${dir}/fleet.json" deadlocked)
    grid=$(readValue "${dir}/fleet.json" collisions)
    adoptions=$(readValue "${dir}/fleet.json" plan_adoptions)
    rejections=$(readValue "${dir}/fleet.json" plan_rejections)
    robots=$(readValue "${dir}/fleet.json" robots_used)
    planning_calls=$(readValue "${dir}/fleet.json" planning_calls)
    planning_p95=$(readValue "${dir}/fleet.json" planning_p95_ms)
    planning_max=$(readValue "${dir}/fleet.json" planning_max_ms)

    local status="ok"
    if [[ "${completed}" -lt "${order_count}" || "${deadlocked}" != "false" ||
        "${contacts}" -ne 0 || "${grid}" -ne 0 ]]; then
        status="FAIL"
    fi
    if [[ "${obstacle}" == "yes" && "${stopped}" -eq 0 ]]; then
        status="FAIL"
    fi
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "${setup}" "${map}" "${fleet}" "${seed}" "${obstacle}" "${order_count}" \
        "${completed}" "${contacts}" "${minimum}" "${deadlocked}" "${grid}" \
        "${adoptions}" "${rejections}" "${robots}" "${stopped}" "${status}" \
        "${planning_calls}" "${planning_p95}" "${planning_max}" > "${dir}/fields"
    echo "${status}" > "${dir}/status"
}

# Starts one run in the background, keeping at most MAX_PARALLEL running.
startJob() {
    while [[ "${RUNNING}" -ge "${MAX_PARALLEL}" ]]; do
        wait -n || true
        RUNNING=$((RUNNING - 1))
    done
    startJobInner "$@"
    RUNNING=$((RUNNING + 1))
}

# Runs one launch in the background and records its exit.
startJobInner() {
    local setup="$1" map="$2" fleet="$3" seed="$4" obstacle="$5" order_count="$6"
    local run_timeout="$7"
    local domain=$((DOMAIN_BASE + JOB_INDEX))
    local dir=".build/ros2/case-${setup}-${seed}-${obstacle}"
    JOB_INDEX=$((JOB_INDEX + 1))
    runCase "${setup}" "${map}" "${fleet}" "${seed}" "${obstacle}" "${order_count}" \
        "${run_timeout}" "${domain}" "${dir}" &
}

# Waits for all background runs.
waitJobs() {
    wait || true
}

if [[ "${AGGREGATE_ONLY}" != "1" ]]; then
    for seed in $(seq 1 "${SEEDS}"); do
        startJob small-4 maps/warehouse_small.map 4 "${seed}" no 12 420
    done
    if [[ "${SKIP_LARGE}" == "0" ]]; then
        for seed in $(seq 1 "${SEEDS}"); do
            startJob large-10 maps/warehouse_large.map 10 "${seed}" no 30 1200
        done
        startJob obstacle maps/warehouse_large.map 4 1 yes 12 500
    fi
    waitJobs
fi

for dir in .build/ros2/case-*/; do
    if [[ -f "${dir}fields" ]]; then
        IFS=',' read -r run_setup run_map run_fleet run_seed run_obstacle run_orders \
            run_completed run_contacts run_min run_deadlocked run_grid run_adoptions \
            run_rejections run_robots run_stopped run_status run_planning_calls \
            run_planning_p95 run_planning_max < "${dir}fields"
        run_status="ok"
        if [[ "${run_completed}" -lt "${run_orders}" || "${run_deadlocked}" != "false" ||
            "${run_contacts}" -ne 0 || "${run_grid}" -ne 0 ]]; then
            run_status="FAIL"
        fi
        if [[ "${run_obstacle}" == "yes" && "${run_stopped}" -eq 0 ]]; then
            run_status="FAIL"
        fi
        printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
            "${run_setup}" "${run_map}" "${run_fleet}" "${run_seed}" "${run_obstacle}" \
            "${run_orders}" "${run_completed}" "${run_contacts}" "${run_min}" \
            "${run_deadlocked}" "${run_grid}" "${run_adoptions}" "${run_rejections}" \
            "${run_robots}" "${run_stopped}" "${run_status}" "${run_planning_calls}" \
            "${run_planning_p95}" "${run_planning_max}" >> "${RAW}"
        if [[ "${run_status}" != "ok" ]]; then
            FAILURES=$((FAILURES + 1))
        fi
    fi
done

{
    echo "ROS 2 integration matrix"
    echo "Command: ./scripts/run_ros2_integration.sh"
    echo "Slowdown probability 0.15, stop probability 0.01, time_scale 3.0."
    echo "Safety is the judge's contact count from true positions."
    echo "Run budgets are simulated seconds; the manager follows the stand-in /clock."
    echo
    printf '%-12s %5s %6s %7s %9s %10s %8s %9s %9s %11s %9s %8s\n' \
        "setup" "runs" "orders" "ok" "contacts" "deadlocks" "closest" "grid" "adopted" \
        "rejected" "min_robots" "all_worked"
    awk -F, 'NR > 1 {
        key = $1
        runs[key] += 1
        orders[key] += $7
        contacts[key] += $8
        if ($10 == "true") deadlocks[key] += 1
        if ($16 == "ok") ok[key] += 1
        if ($9 + 0 < closest[key] || runs[key] == 1) closest[key] = $9 + 0
        grid[key] += $11
        adopted[key] += $12
        rejected[key] += $13
        if (minrobots[key] == "" || $14 + 0 < minrobots[key]) minrobots[key] = $14 + 0
        if ($14 + 0 == $3 + 0) allworked[key] += 1
    }
    END {
        for (key in runs) {
            printf "%-12s %5d %6d %7d %9d %10d %8.3f %9d %9d %11d %9d %8d\n",
                key, runs[key], orders[key], ok[key], contacts[key], deadlocks[key],
                closest[key], grid[key], adopted[key], rejected[key], minrobots[key],
                allworked[key]
        }
    }' "${RAW}" | sort
    echo
    echo "planning cost per setup (tick period is 50 ms at 20 Hz):"
    awk -F, 'NR > 1 {
        key = $1
        calls[key] += $17
        runs[key] += 1
        if ($18 + 0 > p95[key]) p95[key] = $18 + 0
        if ($19 + 0 > maxms[key]) maxms[key] = $19 + 0
    }
    END {
        for (key in runs) {
            printf "%-12s total_calls %6d worst_p95_ms %9.2f worst_max_ms %9.2f\n",
                key, calls[key], p95[key], maxms[key]
        }
    }' "${RAW}" | sort
    echo
    echo "runs that did not finish every order:"
    awk -F, 'NR > 1 && $16 != "ok" {
        print "  " $1 " seed " $4 ": " $7 "/" $6 " orders, contacts " $8 ", deadlocked " $10 ", grid " $11
    }' "${RAW}"
    echo
    echo "per-run detail:"
    cat "${RAW}"
} > "${OUT}"

RUN_COUNT=$(($(wc -l < "${RAW}") - 1))
echo "runs ${RUN_COUNT}, failures ${FAILURES}" | tee -a "${OUT}"
if [[ "${FAILURES}" -ne 0 ]]; then
    exit 1
fi
