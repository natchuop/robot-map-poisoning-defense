from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    listen_host = LaunchConfiguration('listen_host')
    listen_port = LaunchConfiguration('listen_port')
    map_yaml = LaunchConfiguration('map_yaml')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_checkpoint_patrol = LaunchConfiguration('start_checkpoint_patrol')

    pkg_share = FindPackageShare('robot_patrol_node').find('robot_patrol_node')

    amcl_launch = str(Path(pkg_share) / 'launch' / 'amcl_stack.launch.py')
    nav2_launch = str(Path(pkg_share) / 'launch' / 'nav2_stack.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('listen_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('listen_port', default_value='5005'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_checkpoint_patrol', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_launch),
            launch_arguments={
                'listen_host': listen_host,
                'listen_port': listen_port,
                'map_yaml': map_yaml,
                'initial_pose_x': initial_pose_x,
                'initial_pose_y': initial_pose_y,
                'initial_pose_yaw': initial_pose_yaw,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        Node(
            package='robot_patrol_node',
            executable='checkpoint_patrol',
            output='screen',
            condition=IfCondition(start_checkpoint_patrol),
            parameters=[{
                'frame_id': 'map',
            }],
        ),
    ])