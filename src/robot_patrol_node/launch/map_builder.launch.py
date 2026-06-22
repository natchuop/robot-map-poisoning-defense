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
    hit_score_increment = LaunchConfiguration('hit_score_increment')
    free_score_decrement = LaunchConfiguration('free_score_decrement')
    occupied_score_threshold = LaunchConfiguration('occupied_score_threshold')
    free_score_threshold = LaunchConfiguration('free_score_threshold')
    score_min = LaunchConfiguration('score_min')
    score_max = LaunchConfiguration('score_max')
    occupancy_mode = LaunchConfiguration('occupancy_mode')
    require_pose_update = LaunchConfiguration('require_pose_update')

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
            DeclareLaunchArgument('hit_score_increment', default_value='4'),
            DeclareLaunchArgument('free_score_decrement', default_value='1'),
            DeclareLaunchArgument('occupied_score_threshold', default_value='6'),
            DeclareLaunchArgument('free_score_threshold', default_value='-2'),
            DeclareLaunchArgument('score_min', default_value='-12'),
            DeclareLaunchArgument('score_max', default_value='24'),
            DeclareLaunchArgument('occupancy_mode', default_value='direct'),
            DeclareLaunchArgument('require_pose_update', default_value='false'),
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
                        'hit_score_increment': hit_score_increment,
                        'free_score_decrement': free_score_decrement,
                        'occupied_score_threshold': occupied_score_threshold,
                        'free_score_threshold': free_score_threshold,
                        'score_min': score_min,
                        'score_max': score_max,
                        'occupancy_mode': occupancy_mode,
                        'require_pose_update': require_pose_update,
                    }
                ],
            ),
        ]
    )
