#!/usr/bin/env bash
# One command for the Isaac Sim demo: it checks the GPU is free, builds the
# images and the ROS 2 packages, starts the lotse-isaac scene container, runs
# the manager, drivers, judge and dashboard, writes the results and shuts
# everything down. Arguments: seed [normal|obstacle] [record 0|1].

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SEED="${1:-1}"
MODE="${2:-normal}"
RECORD="${3:-0}"
ISAAC_IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
ISAAC_NAME="lotse-isaac"
MANAGER_NAME="lotse-manager"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
RUN_NAME="${LOTSE_ISAAC_RUN_NAME:-run-${SEED}-${MODE}}"
DIST="${REPO_ROOT}/.build/isaac"
RESULT_DIR="${DIST}/${RUN_NAME}"
CONTAINER_RESULT_DIR="/workspace/.build/isaac/${RUN_NAME}"
WALL_TIMEOUT="${LOTSE_ISAAC_WALL_TIMEOUT:-3600}"
ORDERS="${LOTSE_ISAAC_ORDERS:-}"

# Reads one setting from the Isaac demo configuration.
# name: the key to read
conf_value() {
    grep -E "^$1[[:space:]]*=" "${REPO_ROOT}/config/lotse-isaac.conf" | head -1 \
        | cut -d= -f2- | xargs
}

GPU_BUSY="$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader || true)"
if [[ -n "${GPU_BUSY}" ]]; then
    echo "REFUSING TO RUN: another GPU compute job is active:" >&2
    echo "${GPU_BUSY}" >&2
    exit 2
fi

# Removes the Lotse Isaac Sim container if it exists. The owner's containers are
# never touched: only the name lotse-isaac is removed.
cleanup() {
    if [[ -n "${SAMPLER_PID:-}" ]]; then
        kill "${SAMPLER_PID}" >/dev/null 2>&1 || true
    fi
    if [[ -n "${WATCHDOG_PID:-}" ]]; then
        kill "${WATCHDOG_PID}" >/dev/null 2>&1 || true
    fi
    if docker ps -a --format '{{.Names}}' | grep -qx "${ISAAC_NAME}"; then
        docker rm -f "${ISAAC_NAME}" >/dev/null 2>&1 || true
    fi
    if docker ps -a --format '{{.Names}}' | grep -qx "${MANAGER_NAME}"; then
        docker rm -f "${MANAGER_NAME}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

cleanup
mkdir -p "${RESULT_DIR}"
chmod 777 "${RESULT_DIR}"
rm -f "${RESULT_DIR}/fleet.json" "${RESULT_DIR}/judge.json" \
    "${RESULT_DIR}/scene.json" "${RESULT_DIR}/full.mp4" "${RESULT_DIR}/full_raw.mp4" \
    "${RESULT_DIR}/clip.mp4" "${RESULT_DIR}/clip.gif" "${RESULT_DIR}/run_log.jsonl" \
    "${RESULT_DIR}/frames.jsonl" "${RESULT_DIR}/camera.json" "${RESULT_DIR}/stop"

lotse::build_image
if [[ ! -f "${REPO_ROOT}/.build/_deps/include/httplib.h" ]]; then
    echo "fetching the pinned third-party headers ..."
    lotse::run bash -lc 'cmake -S . -B .build/deps -G Ninja -DCMAKE_BUILD_TYPE=Release >/dev/null'
fi
lotse::run bash -lc '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon --log-base .build/ros2/log build \
        --base-paths ros2 \
        --build-base .build/ros2/build \
        --install-base .build/ros2/install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+
'

OBSTACLE_START=-1
OBSTACLE_END=-1
if [[ "${MODE}" == "obstacle" ]]; then
    OBSTACLE_START=12
    OBSTACLE_END=24
fi

ORDER_ARG=""
if [[ -n "${ORDERS}" ]]; then
    ORDER_ARG="order_count:=${ORDERS}"
fi

VIDEO_ARGS=()
if [[ "${RECORD}" == "1" ]]; then
    VIDEO_ARGS=(
        --env "LOTSE_VIDEO_PATH=/workspace/.build/isaac/${RUN_NAME}/full_raw.mp4"
        --env "LOTSE_VIDEO_FPS=$(conf_value video_fps)"
        --env "LOTSE_VIDEO_WIDTH=$(conf_value video_width)"
        --env "LOTSE_VIDEO_HEIGHT=$(conf_value video_height)"
    )
fi

echo "starting ${ISAAC_NAME} (seed ${SEED}, ${MODE}) ..."
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
    --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    --env LOTSE_ISAAC_HEADLESS=1 \
    --env LOTSE_ISAAC_SECONDS=7200 \
    --env "LOTSE_MAP_PATH=$(conf_value map_path)" \
    --env "LOTSE_CELL_SIZE=$(conf_value cell_size)" \
    --env "LOTSE_FLEET_SIZE=$(conf_value fleet_size)" \
    --env "LOTSE_OBSTACLE_START=${OBSTACLE_START}" \
    --env "LOTSE_OBSTACLE_END=${OBSTACLE_END}" \
    --env "LOTSE_ISAAC_RESULT=/workspace/.build/isaac/${RUN_NAME}/scene.json" \
    "${VIDEO_ARGS[@]}" \
    --volume "${REPO_ROOT}:/workspace" \
    --workdir /workspace \
    "${ISAAC_IMAGE}" -lc \
    'source /isaac-sim/setup_ros_env.sh && exec /isaac-sim/python.sh sim/isaac/lotse_warehouse_scene.py' \
    >/dev/null

MEM_LOG="${RESULT_DIR}/gpu_memory.txt"
: > "${MEM_LOG}"
(
    while docker ps --format '{{.Names}}' | grep -qx "${ISAAC_NAME}"; do
        nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits >>"${MEM_LOG}"
        sleep 2
    done
) &
SAMPLER_PID=$!

echo "waiting for the Isaac Sim scene (up to 300 s) ..."
ready=0
for _ in $(seq 1 150); do
    if ! docker ps --format '{{.Names}}' | grep -qx "${ISAAC_NAME}"; then
        break
    fi
    if docker logs "${ISAAC_NAME}" 2>&1 | grep -q "LOTSE_ISAAC_READY"; then
        ready=1
        break
    fi
    sleep 2
done
docker logs "${ISAAC_NAME}" >"${RESULT_DIR}/isaac.log" 2>&1 || true
if [[ "${ready}" -ne 1 ]]; then
    echo "Isaac Sim did not reach the ready state. Last 40 log lines:" >&2
    tail -n 40 "${RESULT_DIR}/isaac.log" >&2
    exit 3
fi
echo "Isaac Sim scene ready."

echo "running the manager, drivers, judge and dashboard (wall limit ${WALL_TIMEOUT}s) ..."
mapfile -d '' -t DOCKER_ARGS < <(lotse::_docker_args)
# Stops the manager if the scene dies, so a simulator crash cannot hang the run.
(
    while docker ps --format '{{.Names}}' | grep -qx "${ISAAC_NAME}"; do
        sleep 2
    done
    docker rm -f "${MANAGER_NAME}" >/dev/null 2>&1 || true
) &
WATCHDOG_PID=$!
timeout "${WALL_TIMEOUT}" docker run --rm --network host --name "${MANAGER_NAME}" "${DOCKER_ARGS[@]}" \
    "${LOTSE_DEV_IMAGE}" bash -lc "
        set -eo pipefail
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
        source /opt/ros/jazzy/setup.bash
        source .build/ros2/install/setup.bash
        set -u
        ros2 launch lotse_ros isaac.launch.py \
            result_dir:=${CONTAINER_RESULT_DIR} seed:=${SEED} ${ORDER_ARG}
    " >"${RESULT_DIR}/manager.log" 2>&1 || true

touch "${RESULT_DIR}/stop"
for _ in $(seq 1 60); do
    if ! docker ps --format '{{.Names}}' | grep -qx "${ISAAC_NAME}"; then
        break
    fi
    sleep 2
done

docker logs "${ISAAC_NAME}" >"${RESULT_DIR}/isaac.log" 2>&1 || true
cleanup

echo
echo "results in ${RESULT_DIR}:"
for file in fleet.json judge.json scene.json; do
    if [[ -f "${RESULT_DIR}/${file}" ]]; then
        echo "  ${file}: $(cat "${RESULT_DIR}/${file}")"
    else
        echo "  ${file}: MISSING"
    fi
done
if [[ -s "${MEM_LOG}" ]]; then
    echo "  peak GPU memory MiB: $(sort -n "${MEM_LOG}" | tail -1)"
fi

if [[ -f "${RESULT_DIR}/full_raw.mp4" ]]; then
    MEDIA_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
    mkdir -p "${MEDIA_ROOT}/media" "${REPO_ROOT}/media"
    echo "encoding H.264 video, clip, GIF and stills ..."
    python3 "${REPO_ROOT}/scripts/encode_video.py" \
        --input "${RESULT_DIR}/full_raw.mp4" \
        --full "${RESULT_DIR}/full.mp4" \
        --clip "${RESULT_DIR}/clip.mp4" \
        --gif "${RESULT_DIR}/clip.gif" \
        --camera "${RESULT_DIR}/camera.json" \
        --run-log "${RESULT_DIR}/run_log.jsonl" \
        --frames-log "${RESULT_DIR}/frames.jsonl" \
        --map "$(conf_value map_path)" \
        --stills-dir "${MEDIA_ROOT}/media" \
        --stills-prefix isaac_demo_still \
        --fps "$(conf_value video_fps)" || true
    cp "${RESULT_DIR}/full.mp4" "${MEDIA_ROOT}/media/isaac_demo_full.mp4"
    cp "${RESULT_DIR}/clip.mp4" "${REPO_ROOT}/media/isaac_demo_clip.mp4"
    cp "${RESULT_DIR}/clip.gif" "${REPO_ROOT}/media/isaac_demo_wait.gif"
    cp "${MEDIA_ROOT}/media/isaac_demo_still_1.png" \
        "${REPO_ROOT}/media/isaac_demo_still.png"
    echo "  video: ${MEDIA_ROOT}/media/isaac_demo_full.mp4"
    echo "  clip:  ${REPO_ROOT}/media/isaac_demo_clip.mp4"
    echo "  gif:   ${REPO_ROOT}/media/isaac_demo_wait.gif"
    echo "  still: ${REPO_ROOT}/media/isaac_demo_still.png"
fi
