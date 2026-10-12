#!/usr/bin/env bash
# Runs the A1 clock test inside the development container. The same seeded
# stand-in scenario runs a fixed number of times at time scale 1.0 and 4.0;
# every run must finish all orders safely, and the simulated finish times must
# not differ between scales by more than the spread seen within one scale.
# Builds the packages and writes results/clock_test.txt.

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
REPEATS=3
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
# domain: the ROS domain id, to keep the runs apart
# dir: directory for the launch log and result files
runOnce() {
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
    for repeat in $(seq 1 "${REPEATS}"); do
        runOnce 1.0 $((60 + repeat)) ".build/clock-test-slow-${repeat}"
    done
    for repeat in $(seq 1 "${REPEATS}"); do
        runOnce 4.0 $((70 + repeat)) ".build/clock-test-fast-${repeat}"
    done
fi

{
    echo "A1 clock test: one seeded scenario at two time scales, ${REPEATS} repeats each"
    echo "Command: ./scripts/clock_test.sh"
    echo "Scenario: warehouse_small, 4 robots, seed ${SEED}, ${ORDERS} orders,"
    echo "slowdown 0.15, stop 0.01. Simulated time comes from the stand-in /clock."
    echo
    printf '%-11s %3s %9s %8s %9s %5s %6s %10s %12s\n' \
        "time_scale" "rep" "completed" "contacts" "deadlocks" "grid" "steps" "sim_sec" "wall_sec"
} > "${OUT}"

status="PASS"
slow_sims=""
fast_sims=""

# Records one run's counters and its simulated finish time.
# scale: the time scale of the run
# repeat: the repeat number
# fleet: the fleet result file
# judge: the judge result file
collectRun() {
    local scale="$1" repeat="$2" fleet="$3" judge="$4"
    local completed contacts deadlocked grid steps sim wall
    completed=$(readValue "${fleet}" orders_completed)
    contacts=$(readValue "${judge}" contacts)
    deadlocked=$(readBool "${fleet}" deadlocked)
    grid=$(readValue "${fleet}" collisions)
    steps=$(readValue "${fleet}" steps)
    sim=$(readValue "${fleet}" sim_seconds)
    wall=$(readValue "${fleet}" wall_seconds)
    printf '%-11s %3s %9s %8s %9s %5s %6s %10s %12s\n' \
        "${scale}" "${repeat}" "${completed}" "${contacts}" "${deadlocked}" "${grid}" \
        "${steps}" "${sim}" "${wall}" >> "${OUT}"
    if [[ "${completed}" -ne "${ORDERS}" || "${contacts}" -ne 0 || "${deadlocked}" != "false" ||
        "${grid}" -ne 0 ]]; then
        status="FAIL"
    fi
    if [[ "${scale}" == "1.0" ]]; then
        slow_sims="${slow_sims} ${sim}"
    else
        fast_sims="${fast_sims} ${sim}"
    fi
}

for repeat in $(seq 1 "${REPEATS}"); do
    dir=".build/clock-test-slow-${repeat}"
    if [[ ! -f "${dir}/fleet.json" || ! -f "${dir}/judge.json" ]]; then
        echo "clock test: missing result files in ${dir}" >&2
        exit 1
    fi
    collectRun 1.0 "${repeat}" "${dir}/fleet.json" "${dir}/judge.json"
done
for repeat in $(seq 1 "${REPEATS}"); do
    dir=".build/clock-test-fast-${repeat}"
    if [[ ! -f "${dir}/fleet.json" || ! -f "${dir}/judge.json" ]]; then
        echo "clock test: missing result files in ${dir}" >&2
        exit 1
    fi
    collectRun 4.0 "${repeat}" "${dir}/fleet.json" "${dir}/judge.json"
done

slow_sims="${slow_sims# }"
fast_sims="${fast_sims# }"
read -r slow_median fast_median slow_range fast_range < <(awk -v s="${slow_sims}" -v f="${fast_sims}" 'BEGIN {
    ns = split(s, a, " "); nf = split(f, b, " ");
    for (i = 1; i <= ns; i++) for (j = i + 1; j <= ns; j++) if (a[j] < a[i]) { t = a[i]; a[i] = a[j]; a[j] = t }
    for (i = 1; i <= nf; i++) for (j = i + 1; j <= nf; j++) if (b[j] < b[i]) { t = b[i]; b[i] = b[j]; b[j] = t }
    sm = a[int((ns + 1) / 2)]; fm = b[int((nf + 1) / 2)];
    print sm, fm, a[ns] - a[1], b[nf] - b[1]
}')
read -r difference spread < <(awk -v sm="${slow_median}" -v fm="${fast_median}" -v sr="${slow_range}" -v fr="${fast_range}" 'BEGIN {
    d = sm - fm; if (d < 0) d = -d;
    sp = sr > fr ? sr : fr;
    print d, sp
}')

if ! awk -v d="${difference}" -v sp="${spread}" 'BEGIN { exit !(d <= sp) }'; then
    status="FAIL"
fi

{
    echo
    echo "simulated finish times, 1.0x:${slow_sims}"
    echo "simulated finish times, 4.0x:${fast_sims}"
    echo "medians: ${slow_median} s (1.0x) vs ${fast_median} s (4.0x); difference ${difference} s"
    echo "spread within a scale: 1.0x ${slow_range} s, 4.0x ${fast_range} s; tolerance ${spread} s"
    echo "criterion: difference <= larger within-scale spread, with every run safe"
    if [[ "${status}" == "PASS" ]]; then
        echo "PASS: the same seeded scenario finishes in the same simulated time window at both scales."
    else
        echo "FAIL: the two time scales disagree beyond the within-scale spread."
    fi
} >> "${OUT}"

cat "${OUT}"
if [[ "${status}" != "PASS" ]]; then
    exit 1
fi
