#!/usr/bin/env bash
# Runs the A1 clock test inside the development container. It builds the ROS 2
# packages, runs one seeded 12-order scenario at time_scale 1.0 and 4.0, and
# checks the outcomes agree in simulated time. Exits non-zero when they do not.

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

MAP="maps/warehouse_small.map"
FLEET=4
SEED=7
ORDERS=12
OUT="results/clock_test.txt"
mkdir -p results

# Reads a numeric field from a small JSON result file.
# file: the JSON file to read
# field: the field name to extract
readValue() {
    grep -o "\"$2\":[0-9.]*" "$1" | head -1 | cut -d: -f2
}

# Reads a true/false field from a small JSON result file.
# file: the JSON file to read
# field: the field name to extract
readBool() {
    grep -o "\"$2\":\(true\|false\)" "$1" | head -1 | cut -d: -f2
}

# Runs one launch at a time scale and writes its result files.
# time_scale: simulated seconds produced per wall second
# domain: the ROS domain id, to keep the two runs apart
# dir: directory for the launch log and result files
runScale() {
    local time_scale="$1" domain="$2" dir="$3"
    mkdir -p "$dir"
    rm -f "$dir/fleet.json" "$dir/judge.json"
    ROS_DOMAIN_ID="$domain" timeout 1800 \
        ros2 launch lotse_ros warehouse.launch.py \
        map_path:="${MAP}" fleet_size:="${FLEET}" seed:="${SEED}" \
        slowdown_probability:=0.15 stop_probability:=0.01 time_scale:="${time_scale}" \
        order_count:="${ORDERS}" timeout_seconds:=1800 result_dir:="${dir}" \
        > "${dir}/launch.log" 2>&1 || true
}

AGGREGATE_ONLY="${CLOCK_TEST_AGGREGATE_ONLY:-0}"
if [[ "${AGGREGATE_ONLY}" != "1" ]]; then
    runScale 1.0 63 .build/clock-test-slow
    runScale 4.0 64 .build/clock-test-fast
fi

SLOW_FLEET=".build/clock-test-slow/fleet.json"
FAST_FLEET=".build/clock-test-fast/fleet.json"
SLOW_JUDGE=".build/clock-test-slow/judge.json"
FAST_JUDGE=".build/clock-test-fast/judge.json"

for file in "${SLOW_FLEET}" "${FAST_FLEET}" "${SLOW_JUDGE}" "${FAST_JUDGE}"; do
    if [[ ! -f "${file}" ]]; then
        echo "clock test: missing ${file}" >&2
        exit 1
    fi
done

{
    echo "A1 clock test: one seeded scenario at two time scales"
    echo "Command: ./scripts/clock_test.sh"
    echo "Scenario: warehouse_small, 4 robots, seed ${SEED}, ${ORDERS} orders,"
    echo "slowdown 0.15, stop 0.01. Simulated time comes from the stand-in /clock."
    echo
    printf '%-11s %9s %8s %9s %5s %6s %10s %12s\n' \
        "time_scale" "completed" "contacts" "deadlocks" "grid" "steps" "sim_sec" "wall_sec"
} > "${OUT}"

status="PASS"
for pair in "1.0:${SLOW_FLEET}:${SLOW_JUDGE}" "4.0:${FAST_FLEET}:${FAST_JUDGE}"; do
    scale="${pair%%:*}"
    rest="${pair#*:}"
    fleet="${rest%%:*}"
    judge="${rest##*:}"
    completed=$(readValue "${fleet}" orders_completed)
    contacts=$(readValue "${judge}" contacts)
    deadlocked=$(readBool "${fleet}" deadlocked)
    grid=$(readValue "${fleet}" collisions)
    steps=$(readValue "${fleet}" steps)
    sim=$(readValue "${fleet}" sim_seconds)
    wall=$(readValue "${fleet}" wall_seconds)
    printf '%-11s %9s %8s %9s %5s %6s %10s %12s\n' \
        "${scale}" "${completed}" "${contacts}" "${deadlocked}" "${grid}" "${steps}" \
        "${sim}" "${wall}" >> "${OUT}"
    if [[ "${completed}" -ne "${ORDERS}" || "${contacts}" -ne 0 || "${deadlocked}" != "false" ||
        "${grid}" -ne 0 ]]; then
        status="FAIL"
    fi
done

slow_steps=$(readValue "${SLOW_FLEET}" steps)
fast_steps=$(readValue "${FAST_FLEET}" steps)
slow_sim=$(readValue "${SLOW_FLEET}" sim_seconds)
fast_sim=$(readValue "${FAST_FLEET}" sim_seconds)

# The safety outcome must match exactly; the finished run may differ by a
# fraction of a percent because the ROS executor schedules the fixed-rate
# timers with wall jitter. Two percent keeps the test meaningful: wall time
# leaking into the logic would move simulated time by a factor near 4.
if ! awk -v a="${slow_sim}" -v b="${fast_sim}" 'BEGIN {
    d = a - b; if (d < 0) d = -d; exit !(a > 0 && d / a <= 0.02)
}'; then
    status="FAIL"
fi

{
    echo
    echo "simulated seconds: ${slow_sim} (1.0x) vs ${fast_sim} (4.0x); grid steps: ${slow_steps} vs ${fast_steps}"
    echo "agreement tolerance: 2 percent of simulated time"
    if [[ "${status}" == "PASS" ]]; then
        echo "PASS: the same seeded scenario completes with the same outcome in simulated time."
    else
        echo "FAIL: the two time scales disagree."
    fi
} >> "${OUT}"

cat "${OUT}"
if [[ "${status}" != "PASS" ]]; then
    exit 1
fi
