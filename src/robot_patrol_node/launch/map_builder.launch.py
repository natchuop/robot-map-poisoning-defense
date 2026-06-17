from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    map_topic = LaunchConfiguration('map_topic')
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    resolution = LaunchConfiguration('resolution')
    map_width_m = LaunchConfiguration('map_width_m')
    map_height_m = LaunchConfiguration('map_height_m')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_yaw = LaunchConfiguration('laser_yaw')

    return LaunchDescription(
        [
            DeclareLaunchArgument('scan_topic', default_value='/scan'),
            DeclareLaunchArgument('pose_topic', default_value='/robot_pose'),
            DeclareLaunchArgument('map_topic', default_value='/map'),
            DeclareLaunchArgument('map_frame', default_value='map'),
            DeclareLaunchArgument('base_frame', default_value='base_link'),
            DeclareLaunchArgument('laser_frame', default_value='laser'),
            DeclareLaunchArgument('resolution', default_value='0.05'),
            DeclareLaunchArgument('map_width_m', default_value='20.0'),
            DeclareLaunchArgument('map_height_m', default_value='20.0'),
            DeclareLaunchArgument('laser_x', default_value='0.0'),
            DeclareLaunchArgument('laser_y', default_value='0.0'),
            DeclareLaunchArgument('laser_yaw', default_value='0.0'),
            Node(
                package='robot_patrol_node',
                executable='map_builder',
                output='screen',
                parameters=[
                    {
                        'scan_topic': scan_topic,
                        'pose_topic': pose_topic,
                        'map_topic': map_topic,
                        'map_frame': map_frame,
                        'base_frame': base_frame,
                        'laser_frame': laser_frame,
                        'resolution': resolution,
                        'map_width_m': map_width_m,
                        'map_height_m': map_height_m,
                        'laser_x': laser_x,
                        'laser_y': laser_y,
                        'laser_yaw': laser_yaw,
                    }
                ],
            ),
        ]
    )
