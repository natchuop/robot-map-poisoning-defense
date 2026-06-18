from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_patrol_node',
            executable='udp_bridge',
            output='screen',
            parameters=[{
                'listen_host': listen_host,
                'listen_port': listen_port,
                'scan_frame': 'laser',
            }],
        ),

        Node(
            package='robot_patrol_node',
            executable='pose_to_odom',
            output='screen',
            parameters=[{
                'pose_topic': pose_topic,
                'odom_topic': odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            }],
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
                'tf_broadcast': True,
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
                'frame_id': map_frame,
                'x': initial_pose_x,
                'y': initial_pose_y,
                'yaw': initial_pose_yaw,
                'publish_count': 120,
            }],
        ),
    ])