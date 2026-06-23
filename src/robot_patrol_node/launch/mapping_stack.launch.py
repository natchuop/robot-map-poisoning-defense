from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    listen_host = LaunchConfiguration('listen_host')
    listen_port = LaunchConfiguration('listen_port')
    scan_frame = LaunchConfiguration('scan_frame')
    hit_score_increment = LaunchConfiguration('hit_score_increment')
    free_score_decrement = LaunchConfiguration('free_score_decrement')
    occupied_score_threshold = LaunchConfiguration('occupied_score_threshold')
    free_score_threshold = LaunchConfiguration('free_score_threshold')
    occupancy_mode = LaunchConfiguration('occupancy_mode')
    require_pose_update = LaunchConfiguration('require_pose_update')
    clear_on_max_range = LaunchConfiguration('clear_on_max_range')
    ray_end_trim_m = LaunchConfiguration('ray_end_trim_m')
    occupied_radius_cells = LaunchConfiguration('occupied_radius_cells')
    auto_expand_map = LaunchConfiguration('auto_expand_map')
    expansion_padding_m = LaunchConfiguration('expansion_padding_m')
    max_map_width_m = LaunchConfiguration('max_map_width_m')
    max_map_height_m = LaunchConfiguration('max_map_height_m')
    max_mapping_angular_speed_rad_s = LaunchConfiguration('max_mapping_angular_speed_rad_s')
    angular_settle_time_s = LaunchConfiguration('angular_settle_time_s')

    return LaunchDescription(
        [
            DeclareLaunchArgument('listen_host', default_value='0.0.0.0'),
            DeclareLaunchArgument('listen_port', default_value='5005'),
            DeclareLaunchArgument('scan_frame', default_value='laser'),
            DeclareLaunchArgument('hit_score_increment', default_value='4'),
            DeclareLaunchArgument('free_score_decrement', default_value='1'),
            DeclareLaunchArgument('occupied_score_threshold', default_value='6'),
            DeclareLaunchArgument('free_score_threshold', default_value='-2'),
            DeclareLaunchArgument('occupancy_mode', default_value='direct'),
            DeclareLaunchArgument('require_pose_update', default_value='false'),
            DeclareLaunchArgument('clear_on_max_range', default_value='false'),
            DeclareLaunchArgument('ray_end_trim_m', default_value='0.08'),
            DeclareLaunchArgument('occupied_radius_cells', default_value='0'),
            DeclareLaunchArgument('auto_expand_map', default_value='true'),
            DeclareLaunchArgument('expansion_padding_m', default_value='1.0'),
            DeclareLaunchArgument('max_map_width_m', default_value='120.0'),
            DeclareLaunchArgument('max_map_height_m', default_value='120.0'),
            DeclareLaunchArgument('max_mapping_angular_speed_rad_s', default_value='0.45'),
            DeclareLaunchArgument('angular_settle_time_s', default_value='0.20'),
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
                parameters=[
                    {
                        'hit_score_increment': hit_score_increment,
                        'free_score_decrement': free_score_decrement,
                        'occupied_score_threshold': occupied_score_threshold,
                        'free_score_threshold': free_score_threshold,
                        'occupancy_mode': occupancy_mode,
                        'require_pose_update': require_pose_update,
                        'clear_on_max_range': clear_on_max_range,
                        'ray_end_trim_m': ray_end_trim_m,
                        'occupied_radius_cells': occupied_radius_cells,
                        'auto_expand_map': auto_expand_map,
                        'expansion_padding_m': expansion_padding_m,
                        'max_map_width_m': max_map_width_m,
                        'max_map_height_m': max_map_height_m,
                        'max_mapping_angular_speed_rad_s': max_mapping_angular_speed_rad_s,
                        'angular_settle_time_s': angular_settle_time_s,
                    }
                ],
            ),
        ]
    )
