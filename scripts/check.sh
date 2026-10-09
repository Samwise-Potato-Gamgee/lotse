#!/usr/bin/env bash
# Runs the Lotse quality gate inside the development image: clang-format, build
# with warnings as errors, unit tests, address/undefined and thread sanitizers,
# clang-tidy, the colcon build of the ROS 2 packages and the node self test.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

echo "== 1/9 build the development image =="
lotse::build_image

echo "== 2/9 clang-format =="
lotse::run bash -lc '
    set -euo pipefail
    mapfile -t files < <(find core tests apps ros2 -type f \
        \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) | sort)
    echo "formatting ${#files[@]} C++ files"
    clang-format --dry-run --Werror "${files[@]}"
'

echo "== 3/9 configure and build (clang++, Debug, -Werror) =="
lotse::run bash -lc '
    set -euo pipefail
    cmake -S . -B .build/check -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    cmake --build .build/check
'

echo "== 4/9 unit tests =="
lotse::run bash -lc '
    set -euo pipefail
    ctest --test-dir .build/check --output-on-failure
'

echo "== 5/9 address + undefined sanitizers =="
lotse::run bash -lc '
    set -euo pipefail
    cmake -S . -B .build/asan -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DLOTSE_SANITIZERS=address,undefined
    cmake --build .build/asan
    ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 \
    UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
        ctest --test-dir .build/asan --output-on-failure
'

echo "== 6/9 thread sanitizer =="
lotse::run_tsan bash -lc '
    set -euo pipefail
    cmake -S . -B .build/tsan -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DLOTSE_SANITIZERS=thread
    cmake --build .build/tsan
    TSAN_OPTIONS=halt_on_error=1 \
        ctest --test-dir .build/tsan --output-on-failure
'

echo "== 7/9 clang-tidy (core + tests + apps) =="
lotse::run bash -lc '
    set -euo pipefail
    mapfile -t files < <(find core tests apps -type f -name "*.cpp" | sort)
    clang-tidy -p .build/check "${files[@]}"
'

echo "== 8/9 colcon build (ROS 2 packages) =="
lotse::colcon_build

echo "== 9/9 clang-tidy (ROS 2) + node self test =="
lotse::run bash -lc '
    set -eo pipefail
    mapfile -t files < <(find ros2 -type f -name "*.cpp" | sort)
    clang-tidy -p .build/ros2/build/lotse_link_check "${files[@]}"

    source /opt/ros/jazzy/setup.bash
    source .build/ros2/install/setup.bash
    set -u
    ros2 run lotse_link_check link_check_node --ros-args -p selftest:=true
'

echo
echo "all checks passed"
