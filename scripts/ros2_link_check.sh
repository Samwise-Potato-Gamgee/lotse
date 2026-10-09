#!/usr/bin/env bash
# Stage 0 smoke test: drives one robot in Isaac Sim from a C++ ROS 2 node and
# checks from the simulator's odometry that the robot moved. Exit status 0 means
# moved, 2 means the GPU was busy, 3 means Isaac Sim did not come up.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

ISAAC_IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
ISAAC_NAME="lotse-isaac"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
FASTDDS_BUILTIN_TRANSPORTS="UDPv4"
DIST="${REPO_ROOT}/.build/link-check"
ISAAC_LOG="${DIST}/isaac.log"
NODE_LOG="${DIST}/node.log"

mkdir -p "${DIST}"

# Removes the Lotse Isaac Sim container if it exists.
cleanup() {
    if docker ps -a --format '{{.Names}}' | grep -qx "${ISAAC_NAME}"; then
        docker rm -f "${ISAAC_NAME}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

busy="$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader || true)"
if [[ -n "${busy}" ]]; then
    echo "REFUSING TO RUN: another GPU compute job is active:" >&2
    echo "${busy}" >&2
    exit 2
fi

cleanup

lotse::build_image
lotse::colcon_build

echo "starting ${ISAAC_NAME} from ${ISAAC_IMAGE} ..."
docker run -d --name "${ISAAC_NAME}" \
    --gpus all \
    --network host \
    --shm-size 2g \
    --entrypoint /bin/bash \
    --env ACCEPT_EULA=Y \
    --env PRIVACY_CONSENT=Y \
    --env OMNI_KIT_ACCEPT_EULA=YES \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
    --env LOTSE_ISAAC_HEADLESS=1 \
    --env LOTSE_LINK_SECONDS=120 \
    --volume "${REPO_ROOT}:/workspace" \
    --workdir /workspace \
    "${ISAAC_IMAGE}" -lc \
    'source /isaac-sim/setup_ros_env.sh && exec /isaac-sim/python.sh sim/isaac/ros2_link_check_scene.py' \
    >/dev/null

echo "waiting for the Isaac Sim scene (up to 300 s) ..."
ready=0
for _ in $(seq 1 150); do
    if docker logs "${ISAAC_NAME}" >"${ISAAC_LOG}" 2>&1; then :; fi
    if grep -q "LOTSE_ISAAC_READY" "${ISAAC_LOG}" 2>/dev/null; then
        ready=1
        break
    fi
    if ! docker ps --format '{{.Names}}' | grep -qx "${ISAAC_NAME}"; then
        break
    fi
    sleep 2
done

if [[ "${ready}" -ne 1 ]]; then
    echo "Isaac Sim did not reach the ready state. Last 40 log lines:" >&2
    tail -n 40 "${ISAAC_LOG}" >&2
    exit 3
fi
echo "Isaac Sim scene ready."

echo "running the C++ link check node ..."
node_status=0
docker run --rm \
    --network host \
    --user "$(id -u):$(id -g)" \
    --env HOME=/tmp \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
    --volume "${REPO_ROOT}:/workspace" \
    --workdir /workspace \
    "${LOTSE_DEV_IMAGE}" \
    bash -lc '
        set -eo pipefail
        source /opt/ros/jazzy/setup.bash
        source .build/ros2/install/setup.bash
        set -u
        exec ros2 run lotse_link_check link_check_node \
            --ros-args -p wait_timeout_seconds:=60.0
    ' >"${NODE_LOG}" 2>&1 || node_status=$?

distance="$(sed -n 's/.* distance_m=\([0-9.]*\) min_distance_m=.*/\1/p' "${NODE_LOG}" | tail -n 1)"

echo
if [[ "${node_status}" -eq 0 && -n "${distance}" ]]; then
    echo "LOTSE LINK CHECK: PASS -- robot moved ${distance} m (odometry, Isaac Sim)"
    exit 0
fi

echo "LOTSE LINK CHECK: FAIL" >&2
echo "node exit status: ${node_status}" >&2
tail -n 30 "${NODE_LOG}" >&2 || true
exit 1
