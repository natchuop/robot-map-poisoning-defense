from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    listen_host = LaunchConfiguration('listen_host')
    listen_port = LaunchConfiguration('listen_port')
    scan_frame = LaunchConfiguration('scan_frame')

    return LaunchDescription(
        [
            DeclareLaunchArgument('listen_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('listen_port', default_value='5005'),
            DeclareLaunchArgument('scan_frame', default_value='laser'),
            Node(
                package='robot_patrol_node',
                executable='udp_bridge',
                output='screen',
                parameters=[
                    {
                        'listen_host': listen_host,
                        'listen_port': listen_port,
                        'scan_frame': scan_frame,
                    }
                ],
            ),
            Node(
                package='robot_patrol_node',
                executable='map_builder',
                output='screen',
            ),
        ]
    )
