# Lotse

Lotse is a multi-robot fleet manager for a warehouse, written in C++20. It takes
transport orders ("bring a box from shelf 12 to packing station 3"), assigns each
order to a robot, plans collision-free routes for the whole fleet, and keeps the
fleet safe when robots run late. It is demonstrated with several robots in
NVIDIA Isaac Sim.

> **Stage 0.** This repository currently contains the build-and-test skeleton and
> a ROS 2 <-> Isaac Sim link check. The planning and fleet-management code is not
> written yet.

## Repository layout

| Path | Contents |
|---|---|
| `core/` | The plain C++20 library with no ROS dependency. All fleet logic lives here. |
| `tests/` | GoogleTest unit tests for the core library. |
| `ros2/` | ROS 2 Jazzy packages (`ament_cmake`). Thin wrappers around `core/`. |
| `sim/isaac/` | The Isaac Sim scene scripts (the only Python in the project). |
| `docker/` | Dockerfiles for the Lotse images. |
| `scripts/` | `check.sh` is the quality gate; the other scripts build and run smoke tests. |
| `docs/` | Longer documentation (added in a later stage). |
| `.github/workflows/` | CI that runs the same checks as `scripts/check.sh`. |

## Building and testing

Everything runs in Docker; the host needs Docker with GPU support and nothing
else. No `sudo` and no host packages are required.

```bash
./scripts/check.sh
```

This builds the development image, then runs clang-format, the build, the unit
tests, the address/undefined and thread sanitizers, clang-tidy, and the colcon
build of the ROS 2 packages.

The stage 0 ROS 2 <-> Isaac Sim link check (one robot, real physics, position
read from the simulator) is a separate, slower command:

```bash
./scripts/ros2_link_check.sh
```

## How position is obtained

The robot's true position comes straight from the Isaac Sim simulator, published
as `nav_msgs/Odometry` on `/odom`. This is ground truth, not localization. A real
deployment would need a localization stack (for example AMCL or an Isaac ROS
visual-inertial pipeline) before these positions could be trusted on hardware.

## Licence

Not yet decided; the owner will choose it in a later stage.
