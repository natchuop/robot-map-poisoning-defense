from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    pkg_share = FindPackageShare('robot_patrol_node').find('robot_patrol_node')
    nav2_bt_share = FindPackageShare('nav2_bt_navigator').find('nav2_bt_navigator')
    default_params_file = str(Path(pkg_share) / 'config' / 'nav2_params.yaml')
    default_nav_to_pose_bt_xml = str(
        Path(nav2_bt_share) / 'behavior_trees' / 'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    default_nav_through_poses_bt_xml = str(
        Path(nav2_bt_share)
        / 'behavior_trees'
        / 'navigate_through_poses_w_replanning_and_recovery.xml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
            }],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
            }],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
            }],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
            }],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
            }],
        ),
    ])
