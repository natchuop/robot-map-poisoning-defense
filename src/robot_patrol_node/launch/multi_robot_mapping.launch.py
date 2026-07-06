from __future__ import annotations

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def env_bool(name: str, default: bool = False) -> bool:
    raw_value = os.getenv(name)
    if raw_value is None:
        return default
    return raw_value.strip().lower() in {'1', 'true', 'yes', 'on'}


def env_int(name: str, default: int) -> int:
    raw_value = os.getenv(name)
    if raw_value is None or not raw_value.strip():
        return default
    return int(raw_value)


def env_float(name: str, default: float) -> float:
    raw_value = os.getenv(name)
    if raw_value is None or not raw_value.strip():
        return default
    return float(raw_value)


def _package_share_path(*parts: str) -> str:
    return str(Path(get_package_share_directory('robot_patrol_node')).joinpath(*parts))


def _load_configured_robots() -> list[dict]:
    config_path = os.getenv('RMPD_MULTI_ROBOT_CONFIG', '').strip()
    if not config_path:
        config_path = _package_share_path('config', 'multi_robot_config.json')

    try:
        with open(config_path, 'r', encoding='utf-8') as handle:
            payload = json.load(handle)
    except FileNotFoundError:
        payload = {}
    except json.JSONDecodeError as exc:
        raise RuntimeError(f'Invalid multi-robot config JSON: {config_path}') from exc

    robots = payload.get('robots') if isinstance(payload, dict) else None
    if not isinstance(robots, list) or not robots:
        robots = [
            {
                'robot_id': 'robot_1',
                'spawn_x': 2.0,
                'spawn_y': 2.0,
                'spawn_yaw': 0.0,
                'control_scheme': 'wasd',
                'compromised': False,
                'listen_port': 5005,
                'rviz_config': 'multi_robot_robot_1_view.rviz',
                'trust': {
                    'robot_2': {'trust': 0.80, 'trust_confidence': 1.00},
                },
            },
            {
                'robot_id': 'robot_2',
                'spawn_x': -2.0,
                'spawn_y': -2.0,
                'spawn_yaw': 1.5708,
                'control_scheme': 'arrows',
                'compromised': False,
                'listen_port': 5006,
                'rviz_config': 'multi_robot_robot_2_view.rviz',
                'trust': {
                    'robot_1': {'trust': 0.20, 'trust_confidence': 1.00},
                },
            },
        ]

    robot_ids = [str(robot.get('robot_id', '')).strip() for robot in robots]
    robot_ids = [robot_id for robot_id in robot_ids if robot_id]

    configured = []
    for robot in robots:
        robot_id = str(robot.get('robot_id', '')).strip()
        if not robot_id:
            continue

        trust_table = robot.get('trust', {})
        if not isinstance(trust_table, dict):
            trust_table = {}

        fake_obstacle = robot.get('fake_obstacle', {})
        if not isinstance(fake_obstacle, dict):
            fake_obstacle = {}

        configured.append(
            {
                'robot_id': robot_id,
                'spawn_x': float(robot.get('spawn_x', 0.0)),
                'spawn_y': float(robot.get('spawn_y', 0.0)),
                'spawn_yaw': float(robot.get('spawn_yaw', 0.0)),
                'control_scheme': str(robot.get('control_scheme', 'manual')).strip() or 'manual',
                'compromised': bool(robot.get('compromised', False)),
                'listen_port': int(robot.get('listen_port', 5005)),
                'rviz_config': str(robot.get('rviz_config', f'{robot_id}_view.rviz')).strip(),
                'trust': trust_table,
                'fake_obstacle': fake_obstacle,
            }
        )

    for robot in configured:
        robot_key = robot['robot_id'].upper().replace('-', '_')
        robot['compromised'] = env_bool(f'RMPD_{robot_key}_COMPROMISED', robot['compromised'])
        robot['listen_port'] = env_int(
            f'RMPD_{robot_key}_BRIDGE_PORT',
            env_int(
                'RMPD_BRIDGE_PORT_SECONDARY' if robot['robot_id'].endswith('2') else 'RMPD_BRIDGE_PORT',
                robot['listen_port'],
            ),
        )

        trust_override = env_float(f'RMPD_{robot_key}_TRUST_WEIGHT', float('nan'))
        if not (trust_override != trust_override):
            for source_robot_id in robot_ids:
                if source_robot_id == robot['robot_id']:
                    continue
                robot['trust'][source_robot_id] = {
                    'trust': trust_override,
                    'trust_confidence': 1.0,
                }

        robot['trust_table_json'] = json.dumps(robot['trust'], sort_keys=True)

    return configured


def _robot_topic(robot_id: str, suffix: str) -> str:
    return f'/{robot_id}/{suffix}'


def _robot_map_builder_config(robot: dict) -> dict:
    defaults = {
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
    }

    overrides = robot.get('map_builder', {})
    if isinstance(overrides, dict):
        for key, value in overrides.items():
            if key in defaults:
                defaults[key] = value

    return defaults


def _resolve_rviz_config(config_name: str) -> str:
    if os.path.isabs(config_name):
        return config_name
    return _package_share_path('config', config_name)


def _bridge_node(robot: dict) -> Node:
    robot_id = robot['robot_id']
    return Node(
        package='robot_patrol_node',
        executable='udp_bridge',
        name=f'{robot_id}_bridge',
        output='screen',
        parameters=[{
            'listen_host': '0.0.0.0',
            'listen_port': robot['listen_port'],
            'pose_topic': _robot_topic(robot_id, 'robot_pose'),
            'scan_topic': _robot_topic(robot_id, 'scan'),
            'cmd_vel_topic': _robot_topic(robot_id, 'cmd_vel'),
            'checkpoint_contact_topic': _robot_topic(robot_id, 'webots_checkpoint_contact'),
            'checkpoint_event_topic': _robot_topic(robot_id, 'webots_checkpoint_event'),
            'active_checkpoint_topic': _robot_topic(robot_id, 'active_checkpoint'),
            'clicked_point_topic': _robot_topic(robot_id, 'clicked_point'),
            'scan_frame': f'{robot_id}/laser',
            'odom_topic': _robot_topic(robot_id, 'odom'),
            'odom_frame': f'{robot_id}/odom',
            'base_frame': f'{robot_id}/base_link',
            'publish_odom': False,
        }],
    )


def _map_builder_node(robot: dict, live_map_width_m: float, live_map_height_m: float, live_map_origin_x: float, live_map_origin_y: float) -> Node:
    robot_id = robot['robot_id']
    map_builder_config = _robot_map_builder_config(robot)
    return Node(
        package='robot_patrol_node',
        executable='map_builder',
        name=f'{robot_id}_map_builder',
        output='screen',
        parameters=[{
            'pose_topic': _robot_topic(robot_id, 'robot_pose'),
            'scan_topic': _robot_topic(robot_id, 'scan'),
            'map_topic': _robot_topic(robot_id, 'live_map'),
            'confidence_map_topic': _robot_topic(robot_id, 'confidence_map'),
            'current_observation_map_topic': _robot_topic(robot_id, 'current_observation_map'),
            'publish_current_observation_map': True,
            'current_observation_max_age_sec': env_float('RMPD_CURRENT_OBSERVATION_MAX_AGE_SEC', 0.0),
            'map_frame': 'map',
            'base_frame': f'{robot_id}/base_link',
            'laser_frame': f'{robot_id}/laser',
            'publish_tf': True,
            'map_width_m': live_map_width_m,
            'map_height_m': live_map_height_m,
            'map_origin_x': live_map_origin_x,
            'map_origin_y': live_map_origin_y,
            'resolution': 0.05,
            **map_builder_config,
        }],
    )


def _belief_node(robot: dict, all_robot_ids: list[str]) -> Node:
    robot_id = robot['robot_id']
    return Node(
        package='robot_patrol_node',
        executable='map_merge',
        name=f'{robot_id}_view_belief',
        output='screen',
        parameters=[{
            'all_robot_ids': all_robot_ids,
            'view_robot_id': robot_id,
            'map_updates_topic': '/map_updates',
            'current_observation_topic': _robot_topic(robot_id, 'current_observation_map'),
            'current_observation_override_enabled': True,
            'current_observation_free_value': 0,
            'current_observation_occupied_value': 100,
            'current_observation_occupied_threshold': 65,
            'current_observation_force_free_logodds': -4.0,
            'current_observation_force_occupied_logodds': 4.0,
            'current_observation_claim_clear_enabled': True,
            'current_observation_claim_clear_ratio_threshold': 0.50,
            'fusion_mode': 'log_odds',
            'logodds_occ': 0.85,
            'logodds_free': -0.35,
            'logodds_min': -4.0,
            'logodds_max': 4.0,
            'occupied_probability_threshold': 0.60,
            'free_probability_threshold': 0.40,
            'occupied_input_threshold': 65,
            'fake_report_radius_cells': 2,
            'fake_claim_logodds_multiplier': 1.50,
            'suppress_attacker_self_free_evidence': True,
            'shared_map_topic': _robot_topic(robot_id, 'shared_live_map'),
            'shared_confidence_topic': _robot_topic(robot_id, 'shared_confidence_map'),
            'trust_table_json': robot['trust_table_json'],
        }],
    )


def _confidence_marker_node(robot: dict, all_robot_ids: list[str]) -> Node:
    robot_id = robot['robot_id']
    source_weights = []
    trust_table = robot.get('trust', {})
    for source_robot_id in all_robot_ids:
        if source_robot_id == robot_id:
            source_weights.append(1.0)
            continue
        trust_entry = trust_table.get(source_robot_id, {}) if isinstance(trust_table, dict) else {}
        if isinstance(trust_entry, dict):
            try:
                source_weights.append(float(trust_entry.get('trust', 1.0)))
                continue
            except (TypeError, ValueError):
                pass
        source_weights.append(1.0)

    return Node(
        package='robot_patrol_node',
        executable='confidence_marker',
        name=f'{robot_id}_confidence_marker',
        output='screen',
        parameters=[{
            'map_topic': _robot_topic(robot_id, 'shared_live_map'),
            'input_topic': _robot_topic(robot_id, 'shared_confidence_map'),
            'source_map_topics': [_robot_topic(other_robot_id, 'live_map') for other_robot_id in all_robot_ids],
            'source_weights': source_weights,
            'output_topic': _robot_topic(robot_id, 'confidence_markers'),
            'marker_namespace': f'{robot_id}_confidence',
            'color_blending_enabled': True,
            'dispute_overlay_enabled': True,
            'overlay_alpha': 0.95,
            'dispute_overlay_alpha': 0.35,
            'cell_scale_z': 0.01,
            'legend_title': f'{robot_id} trust map',
            'unknown_cells_visible': False,
            'source_occupied_threshold': 65,
            'source_free_threshold': 0,
            'occupied_confident_threshold': 70,
            'occupied_possible_threshold': 30,
            'free_confident_threshold': 60,
        }],
    )


def _fake_obstacle_injector_node(robot: dict, all_robot_ids: list[str]) -> Node:
    robot_id = robot['robot_id']
    fake_obstacle = robot.get('fake_obstacle', {})
    if not isinstance(fake_obstacle, dict):
        fake_obstacle = {}

    def _fake_value(key: str, env_name: str, default):
        env_value = os.getenv(env_name)
        if env_value is not None and str(env_value).strip():
            return env_value
        if key in fake_obstacle and fake_obstacle[key] is not None:
            return fake_obstacle[key]
        if key in robot and robot[key] is not None:
            return robot[key]
        return os.getenv(env_name, default)

    return Node(
        package='robot_patrol_node',
        executable='fake_obstacle_injector',
        name=f'{robot_id}_fake_obstacle_injector',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'reporting_robot': robot_id,
            'all_robot_ids': all_robot_ids,
            'compromised': robot['compromised'],
            'compromise_state_topic': _robot_topic(robot_id, 'compromise_state'),
            'target_robot': 'all',
            'mode': str(_fake_value('mode', 'RMPD_FAKE_OBSTACLE_INJECTOR_MODE', 'clicked_point')).strip(),
            'map_updates_topic': '/map_updates',
            'clicked_point_topic': _robot_topic(robot_id, 'clicked_point'),
            'marker_topic': _robot_topic(robot_id, 'fake_obstacle_markers'),
            'occupied': True,
            'obstacle_x': float(_fake_value('obstacle_x', 'RMPD_FAKE_OBSTACLE_X', '1.5')),
            'obstacle_y': float(_fake_value('obstacle_y', 'RMPD_FAKE_OBSTACLE_Y', '-0.8')),
            'source': str(_fake_value('source', 'RMPD_FAKE_OBSTACLE_SOURCE', 'manual_fixed')).strip(),
            'publish_delay_sec': float(_fake_value('publish_delay_sec', 'RMPD_FAKE_OBSTACLE_PUBLISH_DELAY_SEC', '0.5')),
            'marker_lifetime_sec': float(_fake_value('marker_lifetime_sec', 'RMPD_FAKE_OBSTACLE_MARKER_LIFETIME_SEC', '3.0')),
        }],
    )


def _rviz_node(robot: dict) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=['rviz2', '-d', _resolve_rviz_config(robot['rviz_config'])],
        output='screen',
    )


def _launch_setup(context, *args, **kwargs):
    robots = _load_configured_robots()
    all_robot_ids = [robot['robot_id'] for robot in robots]
    start_rviz = env_bool('RMPD_START_RVIZ', False)
    static_map_yaml = LaunchConfiguration('static_map_yaml').perform(context).strip()

    nodes = []

    if static_map_yaml:
        nodes.append(
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='static_map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'yaml_filename': static_map_yaml,
                    'topic_name': '/map',
                    'frame_id': 'map',
                }],
            )
        )
        nodes.append(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_static_map',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': ['static_map_server'],
                }],
            )
        )

    live_map_width_m = LaunchConfiguration('live_map_width_m')
    live_map_height_m = LaunchConfiguration('live_map_height_m')
    live_map_origin_x = LaunchConfiguration('live_map_origin_x')
    live_map_origin_y = LaunchConfiguration('live_map_origin_y')

    for robot in robots:
        nodes.append(_bridge_node(robot))
        nodes.append(
            _map_builder_node(
                robot,
                live_map_width_m,
                live_map_height_m,
                live_map_origin_x,
                live_map_origin_y,
            )
        )
        nodes.append(_belief_node(robot, all_robot_ids))
        nodes.append(_confidence_marker_node(robot, all_robot_ids))
        nodes.append(_fake_obstacle_injector_node(robot, all_robot_ids))
        if start_rviz:
            nodes.append(_rviz_node(robot))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('live_map_width_m', default_value='10.0'),
            DeclareLaunchArgument('live_map_height_m', default_value='10.0'),
            DeclareLaunchArgument('live_map_origin_x', default_value='-4.0'),
            DeclareLaunchArgument('live_map_origin_y', default_value='-4.0'),
            DeclareLaunchArgument('static_map_yaml', default_value=os.getenv('RMPD_STATIC_MAP_YAML', '').strip()),
            OpaqueFunction(function=_launch_setup),
        ]
    )
