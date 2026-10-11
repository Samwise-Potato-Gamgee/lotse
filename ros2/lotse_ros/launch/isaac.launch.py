"""Launch the Lotse Isaac Sim demo operator side: one driver per robot, the
independent contact judge and the dashboard fleet node. The Isaac Sim scene runs
in its own container and provides the simulator clock, odometry, scans and true
poses; every node here follows /clock, so simulated time drives the fleet."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def read_config(path):
    """Reads a key = value configuration file into a dictionary.
    @param path the configuration file
    """
    values = {}
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def _make_nodes(context):
    """Builds the driver, judge and dashboard nodes for one fleet.
    @param context the launch context with the substituted arguments
    """
    config_path = LaunchConfiguration('config').perform(context)
    result_dir = LaunchConfiguration('result_dir').perform(context)
    config = read_config(config_path)
    seed_override = LaunchConfiguration('seed').perform(context)
    order_override = LaunchConfiguration('order_count').perform(context)

    fleet_size = int(config['fleet_size'])
    cell_size = float(config['cell_size'])
    namespaces = ['robot_' + str(robot) for robot in range(fleet_size)]

    nodes = []
    for robot in range(fleet_size):
        nodes.append(Node(
            package='lotse_ros',
            executable='driver',
            name='driver',
            namespace=namespaces[robot],
            parameters=[{
                'use_sim_time': True,
                'cell_size': cell_size,
                'stop_distance': float(config['driver_stop_distance']),
                'max_speed': float(config['driver_max_speed']),
                'turn_gain': float(config['driver_turn_gain']),
                'turn_limit': float(config['driver_turn_limit']),
                'goal_tolerance': float(config['driver_goal_tolerance']),
                'control_rate_hz': float(config['driver_control_rate_hz']),
            }],
            output='screen',
        ))
    nodes.append(Node(
        package='lotse_ros',
        executable='judge_node',
        name='judge_node',
        parameters=[{
            'use_sim_time': True,
            'namespaces': namespaces,
            'robot_radius': float(config['judge_robot_radius']),
            'result_path': result_dir + '/judge.json',
            'rate_hz': 20.0,
        }],
        output='screen',
    ))
    nodes.append(Node(
        package='lotse_ros',
        executable='fleet_api',
        name='fleet_api',
        parameters=[{
            'use_sim_time': True,
            'map_path': config['map_path'],
            'fleet_size': fleet_size,
            'cell_size': cell_size,
            'tick_hz': float(config['tick_hz']),
            'order_count': int(order_override) if order_override else int(config['order_count']),
            'arrival_interval': int(config['arrival_interval']),
            'seed': int(seed_override) if seed_override else int(config['seed']),
            'plan_time_limit_ms': int(config['plan_time_limit_ms']),
            'replan_window_seconds': float(config['replan_window_seconds']),
            'priority_ageing_seconds': float(config['priority_ageing_seconds']),
            'deadlock_patience_seconds': float(config['deadlock_patience_seconds']),
            'timeout_seconds': float(config['timeout_seconds']),
            'dashboard_port': int(config['dashboard_port']),
            'bind_address': config['dashboard_bind_address'],
            'dashboard_dir': config['dashboard_dir'],
            'result_path': result_dir + '/fleet.json',
            'database_path': result_dir + '/orders.sqlite',
        }],
        output='screen',
        on_exit=Shutdown(),
    ))
    return nodes


def generate_launch_description():
    """Declares the launch arguments and the node-building function."""
    arguments = [
        DeclareLaunchArgument('config', default_value=os.path.join('config', 'lotse-isaac.conf')),
        DeclareLaunchArgument('result_dir', default_value='/tmp/lotse-isaac'),
        DeclareLaunchArgument('seed', default_value=''),
        DeclareLaunchArgument('order_count', default_value=''),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_make_nodes)])
