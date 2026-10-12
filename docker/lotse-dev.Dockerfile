# Lotse development image: the C++20 toolchain, GoogleTest, clang-tidy and
# clang-format, plus ROS 2 Jazzy, colcon and matplotlib for the result charts.
# One image runs the quality gate and builds the ROS 2 packages, so the host
# needs only Docker.

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
        python3-matplotlib \
 && rm -rf /var/lib/apt/lists/*

# The address/undefined/thread sanitizer runtimes ship separately from clang.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        libclang-rt-18-dev \
 && rm -rf /var/lib/apt/lists/*
