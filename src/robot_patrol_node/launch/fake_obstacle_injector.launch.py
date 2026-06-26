from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip() or 'robot_1'
    reporting_robot = LaunchConfiguration('reporting_robot').perform(context).strip() or robot_id
    all_robot_ids = LaunchConfiguration('all_robot_ids').perform(context)
    compromised = LaunchConfiguration('compromised').perform(context)
    compromise_state_topic = (
        LaunchConfiguration('compromise_state_topic').perform(context).strip()
        or f'/{robot_id}/compromise_state'
    )
    attack_targets = LaunchConfiguration('attack_targets').perform(context)
    target_robot = LaunchConfiguration('target_robot').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    map_updates_topic = LaunchConfiguration('map_updates_topic').perform(context)
    clicked_point_topic = (
        LaunchConfiguration('clicked_point_topic').perform(context).strip()
        or f'/{robot_id}/clicked_point'
    )
    marker_topic = (
        LaunchConfiguration('marker_topic').perform(context).strip()
        or f'/{robot_id}/fake_obstacle_markers'
    )
    occupied = LaunchConfiguration('occupied').perform(context)
    obstacle_x = LaunchConfiguration('obstacle_x').perform(context)
    obstacle_y = LaunchConfiguration('obstacle_y').perform(context)
    source = LaunchConfiguration('source').perform(context)
    publish_delay_sec = LaunchConfiguration('publish_delay_sec').perform(context)
    marker_lifetime_sec = LaunchConfiguration('marker_lifetime_sec').perform(context)

    return [
        Node(
            package='robot_patrol_node',
            executable='fake_obstacle_injector',
            name='fake_obstacle_injector',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'reporting_robot': reporting_robot,
                'all_robot_ids': all_robot_ids,
                'compromised': compromised,
                'compromise_state_topic': compromise_state_topic,
                'attack_targets': attack_targets,
                'target_robot': target_robot,
                'mode': mode,
                'map_updates_topic': map_updates_topic,
                'clicked_point_topic': clicked_point_topic,
                'marker_topic': marker_topic,
                'occupied': occupied,
                'obstacle_x': obstacle_x,
                'obstacle_y': obstacle_y,
                'source': source,
                'publish_delay_sec': publish_delay_sec,
                'marker_lifetime_sec': marker_lifetime_sec,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('reporting_robot', default_value=''),
        DeclareLaunchArgument('all_robot_ids', default_value='robot_1,robot_2'),
        DeclareLaunchArgument('compromised', default_value='false'),
        DeclareLaunchArgument('compromise_state_topic', default_value=''),
        DeclareLaunchArgument('attack_targets', default_value='all_except_self'),
        DeclareLaunchArgument('target_robot', default_value='all'),
        DeclareLaunchArgument('mode', default_value='clicked_point'),
        DeclareLaunchArgument('map_updates_topic', default_value='/map_updates'),
        DeclareLaunchArgument('clicked_point_topic', default_value=''),
        DeclareLaunchArgument('marker_topic', default_value=''),
        DeclareLaunchArgument('occupied', default_value='true'),
        DeclareLaunchArgument('obstacle_x', default_value='1.5'),
        DeclareLaunchArgument('obstacle_y', default_value='-0.8'),
        DeclareLaunchArgument('source', default_value='manual_fixed'),
        DeclareLaunchArgument('publish_delay_sec', default_value='0.5'),
        DeclareLaunchArgument('marker_lifetime_sec', default_value='3.0'),
        OpaqueFunction(function=_launch_setup),
    ])
