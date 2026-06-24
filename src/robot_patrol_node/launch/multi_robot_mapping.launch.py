from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_node(robot_name, listen_port, pose_topic, scan_topic, odom_topic, base_frame, laser_frame):
    return Node(
        package='robot_patrol_node',
        executable='udp_bridge',
        name=f'{robot_name}_bridge',
        output='screen',
        parameters=[{
            'listen_host': '0.0.0.0',
            'listen_port': listen_port,
            'pose_topic': pose_topic,
            'scan_topic': scan_topic,
            'cmd_vel_topic': f'/{robot_name}/cmd_vel',
            'checkpoint_contact_topic': f'/{robot_name}/webots_checkpoint_contact',
            'checkpoint_event_topic': f'/{robot_name}/webots_checkpoint_event',
            'active_checkpoint_topic': f'/{robot_name}/active_checkpoint',
            'scan_frame': laser_frame,
            'odom_topic': odom_topic,
            'odom_frame': f'{robot_name}/odom',
            'base_frame': base_frame,
            'publish_odom': False,
        }],
    )


def map_builder_node(robot_name, pose_topic, scan_topic, map_topic, confidence_map_topic, base_frame, laser_frame):
    return Node(
        package='robot_patrol_node',
        executable='map_builder',
        name=f'{robot_name}_map_builder',
        output='screen',
        parameters=[{
            'pose_topic': pose_topic,
            'scan_topic': scan_topic,
            'map_topic': map_topic,
            'confidence_map_topic': confidence_map_topic,
            'map_frame': 'map',
            'base_frame': base_frame,
            'laser_frame': laser_frame,
            'publish_tf': True,
            'occupancy_mode': 'scored',
            'require_pose_update': True,
            'hit_score_increment': 6,
            'free_score_decrement': 1,
            'occupied_score_threshold': 10,
            'free_score_threshold': -4,
            'score_min': -8,
            'score_max': 40,
            'clear_on_max_range': False,
            'ray_end_trim_m': 0.10,
            'occupied_radius_cells': 1,
            'auto_expand_map': False,
            'expansion_padding_m': 1.0,
            'max_map_width_m': 20.0,
            'max_map_height_m': 20.0,
            'max_mapping_angular_speed_rad_s': 0.45,
            'angular_settle_time_s': 0.20,
            'map_width_m': LaunchConfiguration('live_map_width_m'),
            'map_height_m': LaunchConfiguration('live_map_height_m'),
            'map_origin_x': LaunchConfiguration('live_map_origin_x'),
            'map_origin_y': LaunchConfiguration('live_map_origin_y'),
            'resolution': 0.05,
        }],
    )


def confidence_marker_node(robot_name, input_topic, output_topic, title):
    return Node(
        package='robot_patrol_node',
        executable='confidence_marker',
        name=f'{robot_name}_confidence_marker',
        output='screen',
        parameters=[{
            'map_topic': f'/{robot_name}/shared_live_map',
            'input_topic': input_topic,
            'source_map_topics': ['/robot_1/live_map', '/robot_2/live_map'],
            'output_topic': output_topic,
            'marker_namespace': f'{robot_name}_confidence',
            'overlay_alpha': 1.0,
            'cell_scale_z': 0.02,
            'legend_title': title,
            'occupied_confident_threshold': 70,
            'occupied_possible_threshold': 30,
            'free_confident_threshold': 60,
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('live_map_width_m', default_value='10.0'),
        DeclareLaunchArgument('live_map_height_m', default_value='10.0'),
        DeclareLaunchArgument('live_map_origin_x', default_value='-4.0'),
        DeclareLaunchArgument('live_map_origin_y', default_value='-4.0'),

        bridge_node(
            'robot_1',
            5005,
            '/robot_1/robot_pose',
            '/robot_1/scan',
            '/robot_1/odom',
            'robot_1/base_link',
            'robot_1/laser',
        ),
        bridge_node(
            'robot_2',
            5006,
            '/robot_2/robot_pose',
            '/robot_2/scan',
            '/robot_2/odom',
            'robot_2/base_link',
            'robot_2/laser',
        ),
        map_builder_node(
            'robot_1',
            '/robot_1/robot_pose',
            '/robot_1/scan',
            '/robot_1/live_map',
            '/robot_1/confidence_map',
            'robot_1/base_link',
            'robot_1/laser',
        ),
        map_builder_node(
            'robot_2',
            '/robot_2/robot_pose',
            '/robot_2/scan',
            '/robot_2/live_map',
            '/robot_2/confidence_map',
            'robot_2/base_link',
            'robot_2/laser',
        ),
        Node(
            package='robot_patrol_node',
            executable='map_merge',
            name='robot_1_view_merge',
            output='screen',
            parameters=[{
                'input_map_topics': ['/robot_1/live_map', '/robot_2/live_map'],
                'input_confidence_topics': [
                    '/robot_1/confidence_map',
                    '/robot_2/confidence_map',
                ],
                'output_map_topic': '/robot_1/shared_live_map',
                'output_confidence_topic': '/robot_1/shared_confidence_map',
                'confidence_weights': [1.0, 0.4],
                'confidence_visual_gamma': 2.6,
            }],
        ),
        confidence_marker_node(
            'robot_1',
            '/robot_1/shared_confidence_map',
            '/robot_1/confidence_markers',
            'Robot 1 Confidence',
        ),
        Node(
            package='robot_patrol_node',
            executable='map_merge',
            name='robot_2_view_merge',
            output='screen',
            parameters=[{
                'input_map_topics': ['/robot_1/live_map', '/robot_2/live_map'],
                'input_confidence_topics': [
                    '/robot_1/confidence_map',
                    '/robot_2/confidence_map',
                ],
                'output_map_topic': '/robot_2/shared_live_map',
                'output_confidence_topic': '/robot_2/shared_confidence_map',
                'confidence_weights': [0.8, 1.0],
                'confidence_visual_gamma': 2.6,
            }],
        ),
        confidence_marker_node(
            'robot_2',
            '/robot_2/shared_confidence_map',
            '/robot_2/confidence_markers',
            'Robot 2 Confidence',
        ),
    ])
