"""Launch the ROS 2 variant of Lotse: the stand-in differential-drive simulator,
one driver node per robot, the independent contact judge and the fleet manager
node. Every node shares one map and one cell size, so a grid cell is the same
number of metres everywhere. When the fleet node finishes it shuts the launch
down, so the integration script gets its exit code back."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context):
    fleet_size = int(LaunchConfiguration('fleet_size').perform(context))
    map_path = LaunchConfiguration('map_path').perform(context)
    cell_size = float(LaunchConfiguration('cell_size').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    time_scale = float(LaunchConfiguration('time_scale').perform(context))
    slowdown = float(LaunchConfiguration('slowdown_probability').perform(context))
    stop = float(LaunchConfiguration('stop_probability').perform(context))
    order_count = int(LaunchConfiguration('order_count').perform(context))
    timeout = float(LaunchConfiguration('timeout_seconds').perform(context))
    result_dir = LaunchConfiguration('result_dir').perform(context)
    obstacle_robot = int(LaunchConfiguration('obstacle_robot').perform(context))
    obstacle_start = float(LaunchConfiguration('obstacle_start').perform(context))
    obstacle_end = float(LaunchConfiguration('obstacle_end').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    namespaces = ['robot_' + str(robot) for robot in range(fleet_size)]
    nodes = [
        Node(
            package='lotse_ros',
            executable='standin_sim',
            name='standin_sim',
            parameters=[{
                'map_path': map_path,
                'fleet_size': fleet_size,
                'cell_size': cell_size,
                'rate_hz': 50.0,
                'time_scale': time_scale,
                'seed': seed,
                'slowdown_probability': slowdown,
                'stop_probability': stop,
                'slowdown_min_factor': 0.3,
                'obstacle_robot': obstacle_robot,
                'obstacle_start': obstacle_start,
                'obstacle_end': obstacle_end,
                'obstacle_distance': 0.35,
            }],
            output='screen',
        )
    ]
    for robot in range(fleet_size):
        nodes.append(Node(
            package='lotse_ros',
            executable='driver',
            name='driver',
            namespace=namespaces[robot],
            parameters=[{
                'use_sim_time': use_sim_time,
                'cell_size': cell_size,
                'stop_distance': 0.5,
                'max_speed': 0.8,
                'turn_gain': 2.0,
                'turn_limit': 1.6,
                'goal_tolerance': 0.06,
                'control_rate_hz': 20.0,
            }],
            output='screen',
        ))
    nodes.append(Node(
        package='lotse_ros',
        executable='judge_node',
        name='judge_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'namespaces': namespaces,
            'robot_radius': 0.25,
            'result_path': result_dir + '/judge.json',
            'rate_hz': 20.0,
        }],
        output='screen',
    ))
    nodes.append(Node(
        package='lotse_ros',
        executable='fleet_node',
        name='fleet_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_path': map_path,
            'fleet_size': fleet_size,
            'cell_size': cell_size,
            'tick_hz': 20.0,
            'order_count': order_count,
            'arrival_interval': 10,
            'seed': seed,
            'plan_time_limit_ms': 500,
            'replan_window_seconds': 6.0,
            'priority_ageing_seconds': 3.75,
            'deadlock_patience_seconds': 450.0,
            'result_path': result_dir + '/fleet.json',
            'timeout_seconds': timeout,
        }],
        output='screen',
        on_exit=Shutdown(),
    ))
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('map_path', default_value='maps/warehouse_large.map'),
        DeclareLaunchArgument('fleet_size', default_value='4'),
        DeclareLaunchArgument('cell_size', default_value='1.0'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('time_scale', default_value='3.0'),
        DeclareLaunchArgument('slowdown_probability', default_value='0.15'),
        DeclareLaunchArgument('stop_probability', default_value='0.02'),
        DeclareLaunchArgument('order_count', default_value='20'),
        DeclareLaunchArgument('timeout_seconds', default_value='200.0'),
        DeclareLaunchArgument('result_dir', default_value='/tmp/lotse-ros'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('obstacle_robot', default_value='-1'),
        DeclareLaunchArgument('obstacle_start', default_value='-1.0'),
        DeclareLaunchArgument('obstacle_end', default_value='-1.0'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_make_nodes)])
