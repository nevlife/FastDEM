"""Launch file for FastDEM elevation mapping node (SCV bag)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    input_scan = LaunchConfiguration('input_scan').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    pkg_share = FindPackageShare('fastdem_ros2')

    # Config baked to the SCV bag (map_frame=odom, input=/velodyne_points).
    config_file = PathJoinSubstitution([pkg_share, 'config', 'scv.yaml'])

    node_params = {'config_file': config_file, 'use_sim_time': use_sim_time}
    if input_scan:
        node_params['input_scan'] = input_scan

    fastdem_node = Node(
        package='fastdem_ros2',
        executable='fastdem_node',
        name='fastdem',
        output='screen',
        parameters=[node_params],
    )

    return [fastdem_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock (bag playback)'),
        DeclareLaunchArgument(
            'input_scan', default_value='',
            description='Override input topic (empty = use config)'),
        OpaqueFunction(function=_launch_setup),
    ])
