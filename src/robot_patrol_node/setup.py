from setuptools import find_packages, setup

package_name = 'robot_patrol_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/map_builder.launch.py']),
        ('share/' + package_name + '/launch', ['launch/mapping_stack.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rviz.launch.py']),
        ('share/' + package_name + '/config', ['config/default.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natch',
    maintainer_email='natch@todo.todo',
    description='Robot patrol and mapping node for Webots + ROS 2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'map_builder = robot_patrol_node.map_builder_node:main',
            'udp_bridge = robot_patrol_node.udp_bridge_node:main',
        ],
    },
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
