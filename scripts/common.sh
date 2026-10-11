# shellcheck shell=bash
# Shared helpers for the Lotse scripts: build the development image and run
# commands inside it, with the repository mounted at /workspace.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOTSE_DEV_IMAGE="${LOTSE_DEV_IMAGE:-lotse-dev:jazzy}"

# Builds the development image if it is not present yet.
lotse::build_image() {
    if docker image inspect "${LOTSE_DEV_IMAGE}" >/dev/null 2>&1; then
        return 0
    fi
    echo "building ${LOTSE_DEV_IMAGE} ..."
    docker build \
        -f "${REPO_ROOT}/docker/lotse-dev.Dockerfile" \
        -t "${LOTSE_DEV_IMAGE}" \
        "${REPO_ROOT}/docker"
}

# Prints the docker arguments shared by every run: host uid/gid, HOME and the
# repository mounted at /workspace. NUL separated so mapfile can read them.
lotse::_docker_args() {
    printf '%s\0' \
        --user "$(id -u):$(id -g)" \
        --env HOME=/tmp \
        --volume "${REPO_ROOT}:/workspace" \
        --workdir /workspace
}

# Runs a command in the development image with the repository mounted.
lotse::run() {
    local -a args
    mapfile -d '' args < <(lotse::_docker_args)
    docker run --rm "${args[@]}" "${LOTSE_DEV_IMAGE}" "$@"
}

# Runs a command in the development image on the host network, so ROS 2 nodes
# in the Isaac Sim container can exchange messages with it.
lotse::run_host() {
    local -a args
    mapfile -d '' args < <(lotse::_docker_args)
    docker run --rm --network host "${args[@]}" "${LOTSE_DEV_IMAGE}" "$@"
}

# Same as lotse::run, but with Docker's seccomp profile relaxed. ThreadSanitizer
# calls personality(ADDR_NO_RANDOMIZE), which the default profile blocks, so the
# TSan binary otherwise dies with a CHECK failure. Test containers only.
lotse::run_tsan() {
    local -a args
    mapfile -d '' args < <(lotse::_docker_args)
    docker run --rm --security-opt seccomp=unconfined "${args[@]}" "${LOTSE_DEV_IMAGE}" "$@"
}

# Builds the ROS 2 packages under ros2/ into .build/ros2 with colcon.
lotse::colcon_build() {
    lotse::run bash -lc '
        set -eo pipefail
        source /opt/ros/jazzy/setup.bash
        set -u
        colcon --log-base .build/ros2/log build \
            --base-paths ros2 \
            --build-base .build/ros2/build \
            --install-base .build/ros2/install \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --event-handlers console_direct+
    '
}
