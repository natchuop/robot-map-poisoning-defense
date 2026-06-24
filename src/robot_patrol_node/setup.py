from setuptools import find_packages, setup

package_name = 'robot_patrol_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/amcl_stack.launch.py']),
        ('share/' + package_name + '/launch', ['launch/map_builder.launch.py']),
        ('share/' + package_name + '/launch', ['launch/mapping_stack.launch.py']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_mapping.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rviz.launch.py']),
        ('share/' + package_name + '/launch', ['launch/nav2_stack.launch.py']),
        ('share/' + package_name + '/launch', ['launch/nav2_with_amcl.launch.py']),

        ('share/' + package_name + '/config', ['config/amcl.rviz']),
        ('share/' + package_name + '/config', ['config/default.rviz']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/config', ['config/office_amcl.rviz']),
        ('share/' + package_name + '/config', ['config/multi_robot_shared_map.rviz']),
        ('share/' + package_name + '/config', ['config/multi_robot_robot_1_view.rviz']),
        ('share/' + package_name + '/config', ['config/multi_robot_robot_2_view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_map_poisoning_defense',
    maintainer_email='maintainer@example.com',
    description='Robot patrol and mapping node for Webots + ROS 2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'initial_pose_publisher = robot_patrol_node.initial_pose_publisher_node:main',
            'map_builder = robot_patrol_node.map_builder_node:main',
            'localized_pose2d = robot_patrol_node.localized_pose2d_node:main',
            'confidence_marker = robot_patrol_node.confidence_marker_node:main',
            'map_merge = robot_patrol_node.map_merge_node:main',
            'pose_to_odom = robot_patrol_node.pose_to_odom_node:main',
            'udp_bridge = robot_patrol_node.udp_bridge_node:main',
            'checkpoint_patrol = robot_patrol_node.checkpoint_patrol_node:main',
            'navigation_diagnostics = robot_patrol_node.navigation_diagnostics_node:main',
        ],
    },
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
