# Lotse development image.
#
# One image serves both halves of stage 0:
#   * the quality gate (build, tests, sanitizers, clang-tidy, clang-format) for
#     the plain C++20 core library, and
#   * the ROS 2 Jazzy side of the project, built with colcon.
#
# It is based on the official ros:jazzy image so no ROS 2 packages are needed
# on the host. The Isaac Sim image is separate and is never rebuilt here.

FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        git \
        clang \
        clang-tidy \
        clang-format \
        libgtest-dev \
        python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# The address/undefined/thread sanitizer runtimes ship separately from clang.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        libclang-rt-18-dev \
 && rm -rf /var/lib/apt/lists/*

# The core library is built with clang (it matches clang-tidy). The ROS 2
# packages are built with colcon, which uses the default compiler in this image.
