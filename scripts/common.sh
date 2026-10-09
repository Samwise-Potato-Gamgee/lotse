# shellcheck shell=bash
# Shared helpers for the Lotse scripts.
#
# Every build and every check runs inside the Lotse development image, so the
# host needs only Docker (with GPU support for the Isaac Sim half) and nothing
# needs sudo.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOTSE_DEV_IMAGE="${LOTSE_DEV_IMAGE:-lotse-dev:jazzy}"

# Build the development image if it is not present yet.
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

# Common docker arguments: mount the repo at /workspace and run as the host
# user so generated files stay user-owned. Emitted NUL separated for mapfile.
lotse::_docker_args() {
    printf '%s\0' \
        --user "$(id -u):$(id -g)" \
        --env HOME=/tmp \
        --volume "${REPO_ROOT}:/workspace" \
        --workdir /workspace
}

# Run a command in the development image with the repository mounted at
# /workspace.
lotse::run() {
    local -a args
    mapfile -d '' args < <(lotse::_docker_args)
    docker run --rm "${args[@]}" "${LOTSE_DEV_IMAGE}" "$@"
}

# Same, but with Docker's seccomp profile relaxed. ThreadSanitizer calls
# personality(ADDR_NO_RANDOMIZE), which the default seccomp profile blocks;
# without this the TSan binary dies with a CHECK failure in the container.
# It is applied to the disposable test container only.
lotse::run_tsan() {
    local -a args
    mapfile -d '' args < <(lotse::_docker_args)
    docker run --rm --security-opt seccomp=unconfined "${args[@]}" "${LOTSE_DEV_IMAGE}" "$@"
}

# colcon build the ROS 2 packages under ros2/ into .build/ros2.
lotse::colcon_build() {
    lotse::run bash -lc '
        set -eo pipefail
        # ROS 2 setup scripts do not tolerate `set -u`; enable it afterwards.
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
