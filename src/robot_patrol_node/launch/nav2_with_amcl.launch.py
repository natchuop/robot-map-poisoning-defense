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
    initial_pose_use_odom = LaunchConfiguration('initial_pose_use_odom')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_udp_bridge = LaunchConfiguration('start_udp_bridge')
    start_map_server = LaunchConfiguration('start_map_server')
    start_lifecycle_manager = LaunchConfiguration('start_lifecycle_manager')
    start_checkpoint_patrol = LaunchConfiguration('start_checkpoint_patrol')
    start_checkpoint_metrics = LaunchConfiguration('start_checkpoint_metrics')
    start_navigation_diagnostics = LaunchConfiguration('start_navigation_diagnostics')
    start_live_mapping = LaunchConfiguration('start_live_mapping')
    live_map_width_m = LaunchConfiguration('live_map_width_m')
    live_map_height_m = LaunchConfiguration('live_map_height_m')
    live_map_origin_x = LaunchConfiguration('live_map_origin_x')
    live_map_origin_y = LaunchConfiguration('live_map_origin_y')
    map_id = LaunchConfiguration('map_id')
    pose_topic = LaunchConfiguration('pose_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    amcl_pose_topic = LaunchConfiguration('amcl_pose_topic')
    initial_pose_topic = LaunchConfiguration('initial_pose_topic')
    active_checkpoint_topic = LaunchConfiguration('active_checkpoint_topic')
    webots_checkpoint_event_topic = LaunchConfiguration('webots_checkpoint_event_topic')
    checkpoint_contact_topic = LaunchConfiguration('checkpoint_contact_topic')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    scan_frame = LaunchConfiguration('scan_frame')
    checkpoint_metrics_output_csv = LaunchConfiguration('checkpoint_metrics_output_csv')
    checkpoint_metrics_radius_m = LaunchConfiguration('checkpoint_metrics_radius_m')
    checkpoint_metrics_reset = LaunchConfiguration('checkpoint_metrics_reset')

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
        DeclareLaunchArgument('initial_pose_use_odom', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_udp_bridge', default_value='true'),
        DeclareLaunchArgument('start_map_server', default_value='true'),
        DeclareLaunchArgument('start_lifecycle_manager', default_value='true'),
        DeclareLaunchArgument('start_checkpoint_patrol', default_value='true'),
        DeclareLaunchArgument('start_checkpoint_metrics', default_value='true'),
        DeclareLaunchArgument('start_navigation_diagnostics', default_value='true'),
        DeclareLaunchArgument('start_live_mapping', default_value='true'),
        DeclareLaunchArgument('live_map_width_m', default_value='8.0'),
        DeclareLaunchArgument('live_map_height_m', default_value='8.0'),
        DeclareLaunchArgument('live_map_origin_x', default_value='nan'),
        DeclareLaunchArgument('live_map_origin_y', default_value='nan'),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('amcl_pose_topic', default_value='/amcl_pose'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose'),
        DeclareLaunchArgument('active_checkpoint_topic', default_value='/active_checkpoint'),
        DeclareLaunchArgument('webots_checkpoint_event_topic', default_value='/webots_checkpoint_event'),
        DeclareLaunchArgument('checkpoint_contact_topic', default_value='/webots_checkpoint_contact'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('scan_frame', default_value='laser'),
        DeclareLaunchArgument('checkpoint_metrics_output_csv', default_value=''),
        DeclareLaunchArgument('checkpoint_metrics_radius_m', default_value='0.40'),
        DeclareLaunchArgument('checkpoint_metrics_reset', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amcl_launch),
            launch_arguments={
                'listen_host': listen_host,
                'listen_port': listen_port,
                'pose_topic': pose_topic,
                'scan_topic': scan_topic,
                'odom_topic': odom_topic,
                'map_yaml': map_yaml,
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'initial_pose_x': initial_pose_x,
                'initial_pose_y': initial_pose_y,
                'initial_pose_yaw': initial_pose_yaw,
                'initial_pose_use_odom': initial_pose_use_odom,
                'use_sim_time': use_sim_time,
                'amcl_pose_topic': amcl_pose_topic,
                'start_udp_bridge': start_udp_bridge,
                'start_map_server': start_map_server,
                'start_lifecycle_manager': start_lifecycle_manager,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'robot_base_frame': base_frame,
                'odom_frame_id': odom_frame,
            }.items(),
        ),

        Node(
            package='robot_patrol_node',
            executable='localized_pose2d',
            output='screen',
            condition=IfCondition(start_live_mapping),
            parameters=[{
                'input_topic': amcl_pose_topic,
                'output_topic': '/localized_pose',
            }],
        ),

        Node(
            package='robot_patrol_node',
            executable='map_builder',
            name='live_map_builder',
            output='screen',
            condition=IfCondition(start_live_mapping),
            parameters=[{
                'scan_topic': scan_topic,
                'pose_topic': pose_topic,
                'map_topic': '/live_map',
                'map_frame': 'map',
                'publish_tf': False,
                'occupancy_mode': 'scored',
                'require_pose_update': True,
                'hit_score_increment': 6,
                'free_score_decrement': 1,
                'occupied_score_threshold': 10,
                'free_score_threshold': -4,
                'score_min': -8,
                'score_max': 40,
                'clear_on_max_range': True,
                'ray_end_trim_m': 0.10,
                'lidar_quality_near_m': 1.25,
                'lidar_quality_far_m': 4.0,
                'min_observation_quality': 0.15,
                'max_free_clear_range_m': 3.0,
                'occupied_radius_cells': 1,
                'auto_expand_map': True,
                'expansion_padding_m': 1.0,
                'max_map_width_m': 120.0,
                'max_map_height_m': 120.0,
                'max_mapping_angular_speed_rad_s': 0.45,
                'angular_settle_time_s': 0.20,
                'map_width_m': live_map_width_m,
                'map_height_m': live_map_height_m,
                'map_origin_x': live_map_origin_x,
                'map_origin_y': live_map_origin_y,
                'resolution': 0.05,
            }],
        ),

        Node(
            package='robot_patrol_node',
            executable='checkpoint_patrol',
            output='screen',
            condition=IfCondition(start_checkpoint_patrol),
            parameters=[{
                'frame_id': map_frame,
                'map_id': map_id,
                'robot_pose_topic': pose_topic,
                'amcl_pose_topic': amcl_pose_topic,
                'scan_topic': scan_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'active_checkpoint_topic': active_checkpoint_topic,
                'webots_checkpoint_event_topic': webots_checkpoint_event_topic,
                'checkpoint_contact_topic': checkpoint_contact_topic,
            }],
        ),

        Node(
            package='robot_patrol_node',
            executable='checkpoint_metrics',
            name='checkpoint_metrics',
            output='screen',
            condition=IfCondition(start_checkpoint_metrics),
            parameters=[{
                'robot_id': 'robot_1',
                'map_id': map_id,
                'pose_topic': pose_topic,
                'arrival_radius_m': checkpoint_metrics_radius_m,
                'output_csv': checkpoint_metrics_output_csv,
                'reset_csv_on_start': checkpoint_metrics_reset,
            }],
        ),

        Node(
            package='robot_patrol_node',
            executable='navigation_diagnostics',
            output='screen',
            condition=IfCondition(start_navigation_diagnostics),
            parameters=[{
                'log_interval_sec': 2.0,
                'arrival_radius': 0.50,
            }],
        ),
    ])
