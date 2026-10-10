#!/usr/bin/env bash
# Starts the ROS 2 variant of Lotse with the stand-in simulator in one command:
# it builds the development image, builds the ROS 2 packages with colcon and
# runs the launch file. Extra arguments are passed to ros2 launch as
# name:=value pairs, for example:
#   ./scripts/run_ros2.sh fleet_size:=10 map_path:=maps/warehouse_large.map

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

lotse::build_image

EXTRA_ARGS="$*"
lotse::run bash -lc "
    set -euo pipefail
    source /opt/ros/jazzy/setup.bash
    colcon --log-base .build/ros2/log build \
        --base-paths ros2 \
        --build-base .build/ros2/build \
        --install-base .build/ros2/install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_direct+
    source .build/ros2/install/setup.bash
    mkdir -p .build/ros2/results
    ros2 launch lotse_ros warehouse.launch.py result_dir:=.build/ros2/results ${EXTRA_ARGS}
"
