"""Builds the Lotse warehouse in Isaac Sim from a Lotse grid map and runs it
headless: floor, obstacle boxes, markers, one Carter per parking cell, the ROS 2
bridge (cmd_vel in; odom, ground_truth and a forward raycast scan out), /clock
and robot contacts. Run with /isaac-sim/python.sh after setup_ros_env.sh.
"""

import json
import math
import os
import sys
import time

import cv2
import numpy as np

from isaacsim import SimulationApp

HEADLESS = os.environ.get("LOTSE_ISAAC_HEADLESS", "1") != "0"
RUN_SECONDS = float(os.environ.get("LOTSE_ISAAC_SECONDS", "1200"))
RESULT_PATH = os.environ.get("LOTSE_ISAAC_RESULT", "/tmp/lotse-isaac.json")
STOP_PATH = os.path.join(os.path.dirname(RESULT_PATH), "stop")

MAP_PATH = os.environ.get("LOTSE_MAP_PATH", "maps/warehouse_small.map")
CELL_SIZE = float(os.environ.get("LOTSE_CELL_SIZE", "1.0"))
FLEET_SIZE = int(os.environ.get("LOTSE_FLEET_SIZE", "4"))
OBSTACLE_START = float(os.environ.get("LOTSE_OBSTACLE_START", "-1"))
OBSTACLE_END = float(os.environ.get("LOTSE_OBSTACLE_END", "-1"))
OBSTACLE_DISTANCE = float(os.environ.get("LOTSE_OBSTACLE_DISTANCE", "0.48"))
ROBOT_RADIUS = float(os.environ.get("LOTSE_ROBOT_RADIUS", "0.25"))
VIDEO_PATH = os.environ.get("LOTSE_VIDEO_PATH", "")
VIDEO_FPS = float(os.environ.get("LOTSE_VIDEO_FPS", "10"))
VIDEO_WIDTH = int(os.environ.get("LOTSE_VIDEO_WIDTH", "640"))
VIDEO_HEIGHT = int(os.environ.get("LOTSE_VIDEO_HEIGHT", "480"))

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": HEADLESS})

import carb
import omni.graph.core as og
import omni.physx
import omni.replicator.core as rep
import omni.usd
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.physics import ContactSensor
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysicsSchemaTools, UsdGeom, UsdLux, UsdPhysics

ROBOT_USD = "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd"


def fail(message):
    """Prints an error marker, closes the app and exits with status 1.
    @param message the reason to print
    """
    print(f"LOTSE_ISAAC_ERROR {message}", flush=True)
    simulation_app.close()
    sys.exit(1)


def read_map(path):
    """Reads a Lotse grid map into its rows, stations and parking cells.
    @param path the map file to read
    """
    with open(path) as handle:
        lines = [line.rstrip("\n") for line in handle]
    height = int(next(line for line in lines if line.startswith("height")).split()[1])
    width = int(next(line for line in lines if line.startswith("width")).split()[1])
    rows = lines[lines.index("map") + 1:lines.index("map") + 1 + height]
    stations = []
    for line in lines[lines.index("stations") + 1:lines.index("parking")]:
        parts = line.split()
        if len(parts) == 3:
            stations.append((parts[0], int(parts[1]), int(parts[2])))
    parking = []
    for line in lines[lines.index("parking") + 1:]:
        parts = line.split()
        if len(parts) == 2:
            parking.append((int(parts[0]), int(parts[1])))
    return {"width": width, "height": height, "rows": rows, "stations": stations, "parking": parking}


def cell_centre(cell):
    """Returns the world (x, y) of a cell centre.
    @param cell an (x, y) grid cell
    """
    return ((cell[0] + 0.5) * CELL_SIZE, (cell[1] + 0.5) * CELL_SIZE)


def add_marker(stage, path, x, y, colour, size):
    """Adds a flat coloured marker that carries no collision.
    @param stage the USD stage
    @param path the prim path
    @param x the marker x in metres
    @param y the marker y in metres
    @param colour the display colour
    @param size the marker side in metres
    """
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(size)
    UsdGeom.Xformable(cube.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.02))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*colour)])


def build_warehouse(stage, map_data):
    """Adds the floor, obstacle boxes and station and parking markers.
    @param stage the USD stage
    @param map_data the map returned by read_map
    """
    PhysicsSchemaTools.addGroundPlane(
        stage, "/World/groundPlane", "Z", 1000, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5)
    )
    for y, row in enumerate(map_data["rows"]):
        for x, cell in enumerate(row):
            if cell != "#":
                continue
            cx, cy = cell_centre((x, y))
            cube = UsdGeom.Cube.Define(stage, f"/World/wall_{x}_{y}")
            cube.GetSizeAttr().Set(CELL_SIZE)
            UsdGeom.Xformable(cube.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(cx, cy, CELL_SIZE / 2.0)
            )
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    for name, x, y in map_data["stations"]:
        colour = (0.32, 0.79, 0.54) if name.startswith("pickup") else (0.94, 0.64, 0.29)
        add_marker(stage, f"/World/station_{name}", *cell_centre((x, y)), colour, CELL_SIZE * 0.6)
    for index, (x, y) in enumerate(map_data["parking"]):
        add_marker(
            stage, f"/World/parking_{index}", *cell_centre((x, y)), (0.44, 0.50, 0.57),
            CELL_SIZE * 0.6,
        )


def build_robot_graph(graph_path, robot_prim, namespace):
    """Wires cmd_vel to the wheels and the wheel joints of one robot.
    @param graph_path the OmniGraph action graph path
    @param robot_prim the prim path of the differential-drive robot
    @param namespace the per-robot topic namespace, e.g. robot_0
    """
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("subscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                ("diffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("artController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "artController.inputs:execIn"),
                ("subscribeTwist.outputs:execOut", "diffController.inputs:execIn"),
                ("subscribeTwist.outputs:linearVelocity", "breakLinVel.inputs:tuple"),
                ("breakLinVel.outputs:x", "diffController.inputs:linearVelocity"),
                ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                ("breakAngVel.outputs:z", "diffController.inputs:angularVelocity"),
                ("diffController.outputs:velocityCommand", "artController.inputs:velocityCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("subscribeTwist.inputs:topicName", f"{namespace}/cmd_vel"),
                ("diffController.inputs:wheelRadius", 0.24),
                ("diffController.inputs:wheelDistance", 0.5),
                ("artController.inputs:jointNames", ["left_wheel", "right_wheel"]),
                ("artController.inputs:targetPrim", [usdrt.Sdf.Path(robot_prim)]),
            ],
        },
    )


def build_clock_graph(graph_path):
    """Publishes the simulator clock on /clock for use_sim_time nodes.
    @param graph_path the OmniGraph action graph path
    """
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("publishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "publishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("publishClock.inputs:topicName", "clock"),
            ],
        },
    )


def set_translation(prim, x, y, z):
    """Sets the prim translation, reusing the op when the asset already has one.
    @param prim the prim to move
    @param x the world x in metres
    @param y the world y in metres
    @param z the world z in metres
    """
    xform = UsdGeom.Xformable(prim)
    op = xform.GetXformOp(UsdGeom.XformOp.TypeTranslate)
    if not op:
        op = xform.AddTranslateOp()
    op.Set(Gf.Vec3d(x, y, z))


def place_robot(stage, robot_prim, x, y):
    """Places a robot at a cell centre with zero heading.
    @param stage the USD stage
    @param robot_prim the prim path of the robot
    @param x the world x in metres
    @param y the world y in metres
    """
    set_translation(stage.GetPrimAtPath(robot_prim), x, y, 0.0)


def robot_world_pose(stage, robot_prim):
    """Returns the moving chassis position and forward direction in the world.
    @param stage the USD stage
    @param robot_prim the prim path of the robot
    """
    chassis = f"{robot_prim}/chassis_link"
    matrix = UsdGeom.Xformable(stage.GetPrimAtPath(chassis)).ComputeLocalToWorldTransform(0.0)
    position = matrix.ExtractTranslation()
    forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetNormalized()
    return position, forward


def start_robot_publishers(namespaces):
    """Creates odom, ground_truth and scan publishers for every robot.
    @param namespaces the per-robot topic namespaces
    """
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    if not rclpy.ok():
        rclpy.init(args=None)
    node = rclpy.create_node("lotse_isaac_truth")
    publishers = []
    for name in namespaces:
        odom = node.create_publisher(Odometry, f"{name}/odom", 10)
        truth = node.create_publisher(Odometry, f"{name}/ground_truth", 10)
        scan = node.create_publisher(LaserScan, f"{name}/scan", 10)
        publishers.append((odom, truth, scan))
    return node, publishers


def publish_robot_poses(stage, robots, publishers, seconds):
    """Publishes each robot's true world pose on its odom and ground_truth.
    @param stage the USD stage
    @param robots the robot prim paths
    @param publishers the (odom, ground_truth, scan) publisher triples
    @param seconds the simulated seconds since play
    """
    from nav_msgs.msg import Odometry

    stamp_seconds = int(seconds)
    stamp_nanoseconds = int((seconds - stamp_seconds) * 1.0e9)
    for index, robot_prim in enumerate(robots):
        chassis = f"{robot_prim}/chassis_link"
        matrix = UsdGeom.Xformable(stage.GetPrimAtPath(chassis)).ComputeLocalToWorldTransform(0.0)
        position = matrix.ExtractTranslation()
        quaternion = matrix.ExtractRotationQuat()
        imaginary = quaternion.GetImaginary()
        message = Odometry()
        message.header.frame_id = "map"
        message.header.stamp.sec = stamp_seconds
        message.header.stamp.nanosec = stamp_nanoseconds
        message.child_frame_id = f"robot_{index}/base_link"
        message.pose.pose.position.x = position[0]
        message.pose.pose.position.y = position[1]
        message.pose.pose.position.z = position[2]
        message.pose.pose.orientation.w = quaternion.GetReal()
        message.pose.pose.orientation.x = imaginary[0]
        message.pose.pose.orientation.y = imaginary[1]
        message.pose.pose.orientation.z = imaginary[2]
        publishers[index][0].publish(message)
        publishers[index][1].publish(message)


def raycast_range(query, origin, direction, max_range, robot_prefix):
    """Returns the distance to the first hit along one ray, skipping the robot.
    @param query the PhysX scene query interface
    @param origin the ray origin
    @param direction the unit ray direction
    @param max_range the maximum distance
    @param robot_prefix the prim path prefix of the robot casting the ray
    """
    found = {"distance": max_range}

    def report(hit):
        """Keeps the first hit that is not on the robot and stops traversal."""
        if hit.collision and hit.collision.startswith(robot_prefix):
            return True
        found["distance"] = hit.distance
        return False

    query.raycast_all(origin, direction, max_range, report, False)
    return found["distance"]


def publish_robot_scans(query, stage, robots, publishers, seconds):
    """Casts a forward range sensor of 20 physics ray casts and publishes it.
    @param query the PhysX scene query interface
    @param stage the USD stage
    @param robots the robot prim paths
    @param publishers the (odom, ground_truth, scan) publisher triples
    @param seconds the simulated seconds since play
    """
    from sensor_msgs.msg import LaserScan

    beam_count = 20
    half_angle = math.radians(25.0)
    max_range = 15.0
    min_range = 0.05
    increment = (2.0 * half_angle) / (beam_count - 1)
    stamp_seconds = int(seconds)
    stamp_nanoseconds = int((seconds - stamp_seconds) * 1.0e9)
    for index, robot_prim in enumerate(robots):
        chassis = f"{robot_prim}/chassis_link"
        matrix = UsdGeom.Xformable(stage.GetPrimAtPath(chassis)).ComputeLocalToWorldTransform(0.0)
        position = matrix.ExtractTranslation()
        forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetNormalized()
        left = matrix.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0)).GetNormalized()
        origin = (position[0], position[1], position[2] + 0.38)
        ranges = []
        for beam in range(beam_count):
            angle = -half_angle + increment * beam
            direction = forward * math.cos(angle) + left * math.sin(angle)
            distance = raycast_range(
                query,
                origin,
                (direction[0], direction[1], direction[2]),
                max_range,
                f"/World/robot_{index}",
            )
            ranges.append(max(min_range, min(max_range, distance)))
        message = LaserScan()
        message.header.frame_id = f"robot_{index}/base_link"
        message.header.stamp.sec = stamp_seconds
        message.header.stamp.nanosec = stamp_nanoseconds
        message.angle_min = float(-half_angle)
        message.angle_max = float(half_angle)
        message.angle_increment = float(increment)
        message.range_min = float(min_range)
        message.range_max = float(max_range)
        message.ranges = ranges
        publishers[index][2].publish(message)


def update_obstacle(stage, prim, seconds, position, forward):
    """Turns the dynamic box on in front of a robot and off again later.
    @param stage the USD stage
    @param prim the obstacle prim
    @param seconds the elapsed simulated seconds
    @param position the robot world position
    @param forward the robot forward direction
    """
    if OBSTACLE_START < 0 or seconds < OBSTACLE_START or seconds >= OBSTACLE_END:
        prim.SetActive(False)
        return
    target = position + forward * OBSTACLE_DISTANCE
    set_translation(prim, target[0], target[1], 0.5)
    prim.SetActive(True)


def create_overhead_camera(stage, map_data):
    """Adds a fixed overhead camera that frames the whole warehouse.
    @param stage the USD stage
    @param map_data the map returned by read_map
    """
    camera = UsdGeom.Camera.Define(stage, "/World/OverheadCamera")
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))
    camera.GetFocalLengthAttr().Set(20.0)
    camera.GetHorizontalApertureAttr().Set(36.0)
    centre_x = map_data["width"] * CELL_SIZE / 2.0
    centre_y = map_data["height"] * CELL_SIZE / 2.0
    height = max(map_data["width"], map_data["height"]) * CELL_SIZE * 0.9 + 4.0
    UsdGeom.Xformable(camera.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(centre_x, centre_y, height))
    return "/World/OverheadCamera"


def add_light(stage):
    """Adds a distant light so the overhead camera sees a lit warehouse.
    @param stage the USD stage
    """
    light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    light.GetIntensityAttr().Set(3000.0)
    light.GetColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))


def start_video(camera_path):
    """Starts writing overhead frames to an mp4 and returns the writer and annotator.
    @param camera_path the overhead camera prim path
    """
    render_product = rep.create.render_product(camera_path, [VIDEO_WIDTH, VIDEO_HEIGHT])
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(render_product)
    writer = cv2.VideoWriter(
        VIDEO_PATH, cv2.VideoWriter_fourcc(*"mp4v"), VIDEO_FPS, (VIDEO_WIDTH, VIDEO_HEIGHT)
    )
    return writer, annotator


def real_contact_count(sensor, robot_prim):
    """Counts contacts with another robot or the obstacle, ignoring the ground.
    @param sensor the robot's contact sensor
    @param robot_prim the prim path of the robot that owns the sensor
    """
    count = 0
    for contact in sensor.get_current_frame().get("contacts", []):
        for body in (contact["body0"], contact["body1"]):
            if body.startswith("/World/robot_") and not body.startswith(robot_prim):
                count += 1
            elif body.startswith("/World/obstacle"):
                count += 1
    return count


def write_result(contacts, contact_frames, frames, sim_seconds, wall_seconds):
    """Writes the simulator contact and duration counters as one JSON object.
    @param contacts how many robot-robot or robot-obstacle contacts were seen
    @param contact_frames how many frames had at least one such contact
    @param frames how many frames were checked
    @param sim_seconds the simulated seconds the scene ran
    @param wall_seconds the wall seconds the scene ran
    """
    with open(RESULT_PATH, "w") as handle:
        json.dump(
            {
                "contacts": contacts,
                "contact_frames": contact_frames,
                "frames": frames,
                "sim_seconds": sim_seconds,
                "wall_seconds": wall_seconds,
            },
            handle,
        )


def main():
    """Builds the scene, runs it and writes the result file."""
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.physics")
    enable_extension("isaacsim.sensors.physx")
    simulation_app.update()

    assets_root = get_assets_root_path()
    if assets_root is None:
        fail("could not find the Isaac Sim assets root")

    map_data = read_map(MAP_PATH)
    if len(map_data["parking"]) < FLEET_SIZE:
        fail("not enough parking cells for the fleet")

    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    build_warehouse(stage, map_data)
    add_light(stage)
    camera_path = create_overhead_camera(stage, map_data)

    robots = []
    contact_sensors = []
    for index in range(FLEET_SIZE):
        robot_prim = f"/World/robot_{index}"
        add_reference_to_stage(assets_root + ROBOT_USD, robot_prim)
        place_robot(stage, robot_prim, *cell_centre(map_data["parking"][index]))
        robots.append(robot_prim)
    simulation_app.update()

    simulation_context = SimulationContext(stage_units_in_meters=1.0)
    simulation_app.update()

    for index, robot_prim in enumerate(robots):
        build_robot_graph(f"/World/robot_{index}/LotseGraph", robot_prim, f"robot_{index}")
        sensor = ContactSensor(
            prim_path=f"{robot_prim}/chassis_link/contact_sensor",
            name=f"contact_{index}",
            min_threshold=0,
            max_threshold=10000000,
            radius=ROBOT_RADIUS,
        )
        sensor.add_raw_contact_data_to_frame()
        contact_sensors.append(sensor)
    build_clock_graph("/LotseClockGraph")

    obstacle = UsdGeom.Cube.Define(stage, "/World/obstacle")
    obstacle.GetSizeAttr().Set(1.0)
    obstacle_xform = UsdGeom.Xformable(obstacle.GetPrim())
    obstacle_xform.AddTranslateOp()
    obstacle_xform.AddScaleOp().Set(Gf.Vec3f(0.2, 1.0, 1.0))
    UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim())
    obstacle.GetPrim().SetActive(False)
    simulation_app.update()

    simulation_context.play()
    simulation_app.update()

    print(
        f"LOTSE_ISAAC_READY fleet={FLEET_SIZE} cell={CELL_SIZE} map={MAP_PATH}",
        flush=True,
    )

    namespaces = [f"robot_{index}" for index in range(FLEET_SIZE)]
    truth_node, pose_publishers = start_robot_publishers(namespaces)
    scene_query = omni.physx.get_physx_scene_query_interface()

    video_writer = None
    rgb_annotator = None
    if VIDEO_PATH:
        video_writer, rgb_annotator = start_video(camera_path)

    contacts = 0
    contact_frames = 0
    frames = 0
    next_capture = 0.5
    next_result = 0.0
    start = time.monotonic()
    while (
        simulation_app.is_running()
        and (time.monotonic() - start) < RUN_SECONDS
        and not os.path.exists(STOP_PATH)
    ):
        seconds = simulation_context.current_time
        position, forward = robot_world_pose(stage, robots[0])
        update_obstacle(stage, obstacle.GetPrim(), seconds, position, forward)
        if video_writer is not None:
            rep.orchestrator.step(pause_timeline=False)
        else:
            simulation_app.update()
        publish_robot_poses(stage, robots, pose_publishers, simulation_context.current_time)
        publish_robot_scans(scene_query, stage, robots, pose_publishers,
                            simulation_context.current_time)
        if video_writer is not None and seconds >= next_capture:
            frame = rgb_annotator.get_data()
            if frame is not None and np.asarray(frame).size > 0:
                video_writer.write(cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGBA2BGR))
            next_capture = seconds + 1.0 / VIDEO_FPS
        frames += 1
        step_contacts = sum(
            real_contact_count(sensor, robot)
            for sensor, robot in zip(contact_sensors, robots)
        )
        contacts += step_contacts
        if step_contacts:
            contact_frames += 1
        if seconds >= next_result:
            write_result(contacts, contact_frames, frames, seconds, time.monotonic() - start)
            next_result = seconds + 5.0

    seconds = simulation_context.current_time
    if video_writer is not None:
        video_writer.release()
    write_result(contacts, contact_frames, frames, seconds, time.monotonic() - start)
    print(f"LOTSE_ISAAC_DONE contacts={contacts} frames={frames} sim={seconds:.2f}", flush=True)
    truth_node.destroy_node()
    simulation_context.stop()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
