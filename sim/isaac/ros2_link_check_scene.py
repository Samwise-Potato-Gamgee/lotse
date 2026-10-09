#!/usr/bin/env python3
"""Isaac Sim 5.1 scene for the Lotse stage 0 ROS 2 link check.

Loads the Carter differential-drive robot that ships with Isaac Sim, builds the
official ROS 2 bridge OmniGraph (twist in -> wheel commands -> ground-truth
odometry out) inside the simulator, and keeps the simulation running so that a
ROS 2 node in a second container can drive it and read its position.

Topics
------
  /cmd_vel   geometry_msgs/msg/Twist      subscribed by the simulator
  /odom      nav_msgs/msg/Odometry        published by the simulator (ground truth)

The graph mirrors isaacsim.ros2.bridge's own
tests/test_differential_base.py::add_differential_drive from the Isaac Sim 5.1
image, which is the authoritative recipe for this Isaac Sim version.

Run (inside the lotse-isaac container):
    source /isaac-sim/setup_ros_env.sh
    /isaac-sim/python.sh sim/isaac/ros2_link_check_scene.py
"""

import os
import sys
import time

from isaacsim import SimulationApp

HEADLESS = os.environ.get("LOTSE_ISAAC_HEADLESS", "1") != "0"
RUN_SECONDS = float(os.environ.get("LOTSE_LINK_SECONDS", "90"))
ROBOT_PRIM = "/Carter"
ODOM_TOPIC = os.environ.get("LOTSE_ODOM_TOPIC", "odom")
CMD_VEL_TOPIC = os.environ.get("LOTSE_CMD_VEL_TOPIC", "cmd_vel")

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": HEADLESS})

import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, UsdGeom  # noqa: E402


def fail(message: str) -> None:
    print(f"LOTSE_ISAAC_ERROR {message}", flush=True)
    simulation_app.close()
    sys.exit(1)


def build_ros2_diff_drive_graph(graph_path: str, robot_prim: str) -> None:
    """Wire cmd_vel -> differential controller -> wheels, and odometry out."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("subscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                ("diffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("artController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                ("OnPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "artController.inputs:execIn"),
                ("subscribeTwist.outputs:execOut", "diffController.inputs:execIn"),
                ("subscribeTwist.outputs:linearVelocity", "breakLinVel.inputs:tuple"),
                ("breakLinVel.outputs:x", "diffController.inputs:linearVelocity"),
                ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                ("breakAngVel.outputs:z", "diffController.inputs:angularVelocity"),
                ("diffController.outputs:velocityCommand", "artController.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                ("subscribeTwist.inputs:topicName", CMD_VEL_TOPIC),
                ("publishOdom.inputs:topicName", ODOM_TOPIC),
                # Carter's geometry and wheel joint names; same values the Isaac
                # Sim 5.1 bridge tests use.
                ("diffController.inputs:wheelRadius", 0.24),
                ("diffController.inputs:wheelDistance", 0.5),
                ("artController.inputs:jointNames", ["left_wheel", "right_wheel"]),
                ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_prim)]),
                ("artController.inputs:targetPrim", [usdrt.Sdf.Path(robot_prim)]),
            ],
        },
    )


def main() -> int:
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    assets_root = get_assets_root_path()
    if assets_root is None:
        fail("could not find the Isaac Sim assets root")

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")

    robot_usd = assets_root + "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd"
    print(f"LOTSE_ISAAC_LOADING {robot_usd}", flush=True)
    add_reference_to_stage(robot_usd, ROBOT_PRIM)
    PhysicsSchemaTools.addGroundPlane(
        stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5)
    )
    simulation_app.update()

    simulation_context = SimulationContext(stage_units_in_meters=1.0)
    simulation_app.update()

    build_ros2_diff_drive_graph("/LotseActionGraph", ROBOT_PRIM)
    simulation_app.update()

    simulation_context.play()
    simulation_app.update()

    print(f"LOTSE_ISAAC_READY cmd_vel={CMD_VEL_TOPIC} odom={ODOM_TOPIC}", flush=True)

    start = time.monotonic()
    while simulation_app.is_running() and (time.monotonic() - start) < RUN_SECONDS:
        simulation_app.update()

    print(f"LOTSE_ISAAC_DONE ran={time.monotonic() - start:.1f}s", flush=True)
    simulation_context.stop()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
