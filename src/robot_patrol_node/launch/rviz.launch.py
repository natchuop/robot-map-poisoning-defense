from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')

    default_config = PathJoinSubstitution(
        [FindPackageShare('robot_patrol_node'), 'config', 'default.rviz']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('rviz_config', default_value=default_config),
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                output='screen',
            ),
        ]
    )
