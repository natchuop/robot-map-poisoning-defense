import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _secondary_bridge_enabled() -> bool:
    raw_value = os.getenv('RMPD_START_SECONDARY_BRIDGE', '').strip().lower()
    return raw_value in {'1', 'true', 'yes', 'on'}


def _secondary_bridge_port() -> int:
    raw_value = os.getenv('RMPD_BRIDGE_PORT_SECONDARY', '').strip()
    if not raw_value:
        return 5006

    port = int(raw_value)
    return port if port > 0 else 5006


def _secondary_bridge_topic_prefix() -> str:
    prefix = os.getenv('RMPD_SECONDARY_BRIDGE_TOPIC_PREFIX', 'observer_bot').strip()
    return prefix.strip('/') or 'observer_bot'


def _secondary_bridge_node() -> Node:
    topic_prefix = _secondary_bridge_topic_prefix()
    listen_port = _secondary_bridge_port()
    return Node(
        package='robot_patrol_node',
        executable='udp_bridge',
        name=f'{topic_prefix}_bridge',
        output='screen',
        parameters=[{
            'listen_host': '0.0.0.0',
            'listen_port': listen_port,
            'scan_frame': f'{topic_prefix}/laser',
            'max_publish_hz': 25.0,
            'publish_odom': True,
            'pose_topic': f'/{topic_prefix}/robot_pose',
            'scan_topic': f'/{topic_prefix}/scan',
            'cmd_vel_topic': f'/{topic_prefix}/cmd_vel',
            'checkpoint_contact_topic': f'/{topic_prefix}/webots_checkpoint_contact',
            'checkpoint_event_topic': f'/{topic_prefix}/webots_checkpoint_event',
            'active_checkpoint_topic': f'/{topic_prefix}/active_checkpoint',
            'clicked_point_topic': f'/{topic_prefix}/clicked_point',
            'odom_topic': f'/{topic_prefix}/odom',
            'odom_frame': f'{topic_prefix}/odom',
            'base_frame': f'{topic_prefix}/base_link',
        }],
    )


def _secondary_map_builder_node() -> Node:
    topic_prefix = _secondary_bridge_topic_prefix()
    return Node(
        package='robot_patrol_node',
        executable='map_builder',
        name=f'{topic_prefix}_map_builder',
        output='screen',
        parameters=[{
            'scan_topic': f'/{topic_prefix}/scan',
            'pose_topic': f'/{topic_prefix}/robot_pose',
            'map_topic': f'/{topic_prefix}/live_map',
            'confidence_map_topic': f'/{topic_prefix}/confidence_map',
            'map_frame': 'map',
            'base_frame': f'{topic_prefix}/base_link',
            'laser_frame': f'{topic_prefix}/laser',
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
            'auto_expand_map': False,
            'expansion_padding_m': 1.0,
            'max_map_width_m': 20.0,
            'max_map_height_m': 20.0,
            'max_mapping_angular_speed_rad_s': 0.45,
            'angular_settle_time_s': 0.20,
        }],
    )


def generate_launch_description():
    listen_host = LaunchConfiguration('listen_host')
    listen_port = LaunchConfiguration('listen_port')
    pose_topic = LaunchConfiguration('pose_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    map_yaml = LaunchConfiguration('map_yaml')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_use_odom = LaunchConfiguration('initial_pose_use_odom')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bridge_publish_hz = LaunchConfiguration('bridge_publish_hz')

    actions = [
        DeclareLaunchArgument('listen_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('listen_port', default_value='5005'),
        DeclareLaunchArgument('pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_use_odom', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bridge_publish_hz', default_value='25.0'),

        Node(
            package='robot_patrol_node',
            executable='udp_bridge',
            output='screen',
            parameters=[{
                'listen_host': listen_host,
                'listen_port': listen_port,
                'scan_frame': 'laser',
                'max_publish_hz': bridge_publish_hz,
                'clicked_point_topic': '/clicked_point',
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser',
            ],
            output='screen',
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'topic_name': '/map',
                'frame_id': map_frame,
            }],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'global_frame_id': map_frame,
                'scan_topic': scan_topic,
                'tf_broadcast': False,
                'transform_tolerance': 1.0,
                'set_initial_pose': False,
            }],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),

        Node(
            package='robot_patrol_node',
            executable='initial_pose_publisher',
            output='screen',
            parameters=[{
                'topic': '/initialpose',
                'odom_topic': odom_topic,
                'frame_id': map_frame,
                'use_odom_pose': initial_pose_use_odom,
                'x': initial_pose_x,
                'y': initial_pose_y,
                'yaw': initial_pose_yaw,
                'publish_count': 1,
            }],
        ),
    ]

    if _secondary_bridge_enabled():
        topic_prefix = _secondary_bridge_topic_prefix()
        actions.insert(1, _secondary_bridge_node())
        actions.insert(
            2,
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{topic_prefix}_base_to_laser_tf',
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0',
                    '--frame-id', f'{topic_prefix}/base_link',
                    '--child-frame-id', f'{topic_prefix}/laser',
                ],
                output='screen',
            ),
        )
        actions.insert(3, _secondary_map_builder_node())

    return LaunchDescription(actions)
