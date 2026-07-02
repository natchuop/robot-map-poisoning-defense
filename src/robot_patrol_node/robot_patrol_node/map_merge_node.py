from __future__ import annotations

from copy import deepcopy
import json
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid
from robot_patrol_msgs.msg import MapUpdate
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .fusion_policy import FusionPolicy
from .map_evidence import MapEvidence


class MapMergeNode(Node):
    """Fuse robot maps and shared claims into a log-odds belief map."""

    def __init__(self) -> None:
        super().__init__('map_merge')

        self.declare_parameter('all_robot_ids', ['robot_1', 'robot_2'])
        self.declare_parameter('input_map_topics', ['/robot_1/live_map', '/robot_2/live_map'])
        self.declare_parameter('shared_map_topic', '')
        self.declare_parameter('shared_confidence_topic', '')
        self.declare_parameter('output_map_topic', '/shared_live_map')
        self.declare_parameter('output_confidence_topic', '/shared_confidence_map')
        self.declare_parameter('view_robot_id', 'robot_1')
        self.declare_parameter('trust_table_json', '{}')
        self.declare_parameter('map_updates_topic', '/map_updates')
        self.declare_parameter('current_observation_topic', '')
        self.declare_parameter('current_observation_override_enabled', True)
        self.declare_parameter('current_observation_free_value', 0)
        self.declare_parameter('current_observation_occupied_value', 100)
        self.declare_parameter('current_observation_occupied_threshold', 65)
        self.declare_parameter('current_observation_force_free_logodds', float('nan'))
        self.declare_parameter('current_observation_force_occupied_logodds', float('nan'))
        self.declare_parameter('current_observation_claim_clear_enabled', True)
        self.declare_parameter('current_observation_claim_clear_ratio_threshold', 0.50)
        self.declare_parameter('current_observation_claim_clear_radius_cells', -1)
        self.declare_parameter('fusion_mode', 'log_odds')
        self.declare_parameter('logodds_occ', 0.85)
        self.declare_parameter('logodds_free', -0.35)
        self.declare_parameter('logodds_min', -4.0)
        self.declare_parameter('logodds_max', 4.0)
        self.declare_parameter('occupied_probability_threshold', 0.60)
        self.declare_parameter('free_probability_threshold', 0.40)
        self.declare_parameter('occupied_input_threshold', 65)
        self.declare_parameter('fake_report_radius_cells', 2)
        self.declare_parameter('fake_claim_logodds_multiplier', 1.5)
        self.declare_parameter('suppress_attacker_self_free_evidence', True)

        self.view_robot_id = str(self.get_parameter('view_robot_id').value).strip() or 'robot_1'
        self.all_robot_ids = self._normalize_string_list(self.get_parameter('all_robot_ids').value)
        if self.view_robot_id not in self.all_robot_ids:
            self.all_robot_ids.append(self.view_robot_id)

        self.input_map_topics = self._resolve_topic_list('input_map_topics', suffix='live_map')
        self.shared_map_topic = self._resolve_topic('shared_map_topic', 'output_map_topic', f'/{self.view_robot_id}/shared_live_map')
        self.shared_confidence_topic = self._resolve_topic(
            'shared_confidence_topic',
            'output_confidence_topic',
            f'/{self.view_robot_id}/shared_confidence_map',
        )
        self.current_observation_topic = (
            str(self.get_parameter('current_observation_topic').value).strip()
            or f'/{self.view_robot_id}/current_observation_map'
        )
        self.map_updates_topic = str(self.get_parameter('map_updates_topic').value).strip() or '/map_updates'
        self.current_observation_override_enabled = bool(
            self.get_parameter('current_observation_override_enabled').value
        )
        self.current_observation_free_value = int(self.get_parameter('current_observation_free_value').value)
        self.current_observation_occupied_value = int(self.get_parameter('current_observation_occupied_value').value)
        self.current_observation_occupied_threshold = int(
            self.get_parameter('current_observation_occupied_threshold').value
        )
        current_force_free = self.get_parameter('current_observation_force_free_logodds').value
        current_force_occupied = self.get_parameter('current_observation_force_occupied_logodds').value
        self.current_observation_force_free_logodds = (
            float(current_force_free)
            if math.isfinite(float(current_force_free))
            else self.logodds_min
        )
        self.current_observation_force_occupied_logodds = (
            float(current_force_occupied)
            if math.isfinite(float(current_force_occupied))
            else self.logodds_max
        )
        self.current_observation_claim_clear_enabled = bool(
            self.get_parameter('current_observation_claim_clear_enabled').value
        )
        self.current_observation_claim_clear_ratio_threshold = float(
            self.get_parameter('current_observation_claim_clear_ratio_threshold').value
        )
        self.current_observation_claim_clear_radius_cells = int(
            self.get_parameter('current_observation_claim_clear_radius_cells').value
        )
        self.fusion_mode = str(self.get_parameter('fusion_mode').value).strip() or 'log_odds'
        self.logodds_occ = float(self.get_parameter('logodds_occ').value)
        self.logodds_free = float(self.get_parameter('logodds_free').value)
        self.logodds_min = float(self.get_parameter('logodds_min').value)
        self.logodds_max = float(self.get_parameter('logodds_max').value)
        self.occupied_probability_threshold = float(self.get_parameter('occupied_probability_threshold').value)
        self.free_probability_threshold = float(self.get_parameter('free_probability_threshold').value)
        self.occupied_input_threshold = int(self.get_parameter('occupied_input_threshold').value)
        self.fake_report_radius_cells = int(self.get_parameter('fake_report_radius_cells').value)
        self.fake_claim_logodds_multiplier = float(self.get_parameter('fake_claim_logodds_multiplier').value)
        self.suppress_attacker_self_free_evidence = bool(
            self.get_parameter('suppress_attacker_self_free_evidence').value
        )
        self.trust_table = self._parse_trust_table(self.get_parameter('trust_table_json').value)
        self.fusion_policy = FusionPolicy(self.fusion_mode)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        updates_qos = QoSProfile(depth=10)
        updates_qos.reliability = ReliabilityPolicy.RELIABLE
        updates_qos.durability = DurabilityPolicy.VOLATILE

        self.local_maps: dict[str, OccupancyGrid] = {}
        self.active_claims: dict[str, MapEvidence] = {}
        self.current_observation_map: OccupancyGrid | None = None
        self.topic_subscriptions = []

        for topic in self.input_map_topics:
            self.topic_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, topic_name=topic: self.map_callback(topic_name, msg),
                    map_qos,
                )
            )

        self.topic_subscriptions.append(
            self.create_subscription(
                MapUpdate,
                self.map_updates_topic,
                self.map_update_callback,
                updates_qos,
            )
        )
        if self.current_observation_topic:
            self.topic_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    self.current_observation_topic,
                    self.current_observation_callback,
                    map_qos,
                )
            )

        self.map_pub = self.create_publisher(OccupancyGrid, self.shared_map_topic, map_qos)
        self.confidence_pub = self.create_publisher(OccupancyGrid, self.shared_confidence_topic, map_qos)

        self.get_logger().info(
            'Map merge ready: '
            f'view={self.view_robot_id} '
            f'mode={self.fusion_mode} '
            f'maps={self.input_map_topics} -> {self.shared_map_topic}; '
            f'confidence={self.shared_confidence_topic}; '
            f'current_observation={self.current_observation_topic or "disabled"}; '
            f'updates={self.map_updates_topic}; '
            f'active_claims=0'
        )

    @staticmethod
    def _normalize_string_list(raw_value) -> list[str]:
        if isinstance(raw_value, (list, tuple, set)):
            return [str(value).strip() for value in raw_value if str(value).strip()]
        if raw_value is None:
            return []
        if isinstance(raw_value, str):
            parts = raw_value.split(',')
            return [part.strip() for part in parts if part.strip()]
        return [str(raw_value).strip()]

    @staticmethod
    def _parse_trust_table(raw_value) -> dict:
        if isinstance(raw_value, dict):
            return raw_value
        if isinstance(raw_value, str):
            raw_value = raw_value.strip()
            if not raw_value:
                return {}
            try:
                parsed = json.loads(raw_value)
            except json.JSONDecodeError:
                return {}
            if isinstance(parsed, dict):
                return parsed
        return {}

    def _resolve_topic(self, preferred_parameter: str, legacy_parameter: str, default_topic: str) -> str:
        preferred_value = str(self.get_parameter(preferred_parameter).value).strip()
        if preferred_value:
            return preferred_value

        legacy_value = str(self.get_parameter(legacy_parameter).value).strip()
        if legacy_value:
            return legacy_value

        return default_topic

    def _resolve_topic_list(self, parameter_name: str, suffix: str) -> list[str]:
        topics = self._normalize_string_list(self.get_parameter(parameter_name).value)
        if topics:
            return topics
        return [f'/{robot_id}/{suffix}' for robot_id in self.all_robot_ids]

    @staticmethod
    def _topic_to_robot_id(topic: str) -> str:
        parts = [part for part in str(topic).split('/') if part]
        if len(parts) >= 2:
            return parts[-2]
        return str(topic).strip('/') or 'robot'

    def map_callback(self, topic_name: str, msg: OccupancyGrid) -> None:
        robot_id = self._topic_to_robot_id(topic_name)
        self.local_maps[robot_id] = msg
        self.publish_merged_map()

    def current_observation_callback(self, msg: OccupancyGrid) -> None:
        self.current_observation_map = msg
        self.publish_merged_map()

    def map_update_callback(self, msg: MapUpdate) -> None:
        if str(msg.target_robot_id).strip() and str(msg.target_robot_id).strip() != self.view_robot_id:
            return

        stamp_sec = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        claim_id = str(msg.claim_id).strip()
        if not claim_id:
            claim_id = (
                f'{msg.reporting_robot_id}_{msg.target_robot_id}_'
                f'{msg.world_x:.3f}_{msg.world_y:.3f}_{stamp_sec:.6f}'
            )

        evidence = MapEvidence(
            claim_id=claim_id,
            reporting_robot_id=str(msg.reporting_robot_id).strip() or 'unknown',
            target_robot_id=str(msg.target_robot_id).strip(),
            cell_x=int(msg.cell_x),
            cell_y=int(msg.cell_y),
            world_x=float(msg.world_x),
            world_y=float(msg.world_y),
            occupied=bool(msg.occupied),
            source=str(msg.source).strip(),
            attack_type=str(msg.attack_type).strip(),
            is_attack_report=bool(msg.is_attack_report),
            stamp_sec=stamp_sec,
        )

        if evidence.claim_id in self.active_claims:
            self.get_logger().info(f'[{self.view_robot_id}] ignored duplicate claim_id={evidence.claim_id}')
            return

        self.active_claims[evidence.claim_id] = evidence
        self.get_logger().info(
            f'[{self.view_robot_id}] accepted MapUpdate claim_id={evidence.claim_id} '
            f'from {evidence.reporting_robot_id} target={evidence.target_robot_id or "all"} '
            f'occupied={evidence.occupied} world=({evidence.world_x:.3f}, {evidence.world_y:.3f})'
        )
        self.publish_merged_map()

    def publish_merged_map(self) -> None:
        merged = self.create_reference_grid_from_local_maps()
        if merged is None:
            return

        height = int(merged.info.height)
        width = int(merged.info.width)
        if height <= 0 or width <= 0:
            return

        log_odds = np.zeros((height, width), dtype=np.float32)
        observed = np.zeros((height, width), dtype=bool)

        self.apply_local_maps_to_log_odds(log_odds, observed, merged.info)
        self.apply_active_claims_to_log_odds(log_odds, observed, merged.info)
        if self.current_observation_override_enabled:
            self.apply_current_observation_precedence(log_odds, observed, merged.info)

        log_odds = np.clip(log_odds, self.logodds_min, self.logodds_max)
        merged.data = self.log_odds_to_grid_data(log_odds, observed)

        confidence = self.build_confidence_grid(merged, log_odds, observed)

        self.map_pub.publish(merged)
        self.confidence_pub.publish(confidence)
        if self.current_observation_override_enabled:
            self.remove_claims_contradicted_by_current_observation(merged.info)
        self.get_logger().info(
            f'[{self.view_robot_id}] published log_odds shared map '
            f'active_claims={len(self.active_claims)} local_maps={len(self.local_maps)}'
        )

    def create_reference_grid_from_local_maps(self) -> OccupancyGrid | None:
        ready_maps = [(robot_id, msg) for robot_id, msg in self.local_maps.items() if msg is not None]
        if not ready_maps:
            return None

        reference_robot_id, reference = ready_maps[0]
        for robot_id, msg in ready_maps[1:]:
            if not self.same_geometry(reference.info, msg.info):
                self.get_logger().warning(
                    f'[{self.view_robot_id}] skipped merge because map geometries differ: '
                    f'{reference_robot_id} vs {robot_id}'
                )
                return None

        merged = OccupancyGrid()
        merged.header = deepcopy(reference.header)
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.info = deepcopy(reference.info)
        merged.data = [-1] * (int(merged.info.width) * int(merged.info.height))
        return merged

    def apply_local_maps_to_log_odds(self, log_odds, observed, info) -> None:
        for robot_id, local_map in self.local_maps.items():
            if local_map is None:
                continue

            weight = self.fusion_policy.weight_for_local_map(robot_id, self.view_robot_id)
            if weight <= 0.0:
                continue

            src_width = int(local_map.info.width)
            src_height = int(local_map.info.height)
            if src_width <= 0 or src_height <= 0:
                continue

            for src_index, raw_value in enumerate(local_map.data):
                value = int(raw_value)
                if value < 0:
                    continue

                src_x = src_index % src_width
                src_y = src_index // src_width
                world_x, world_y = self.cell_to_world(local_map.info, src_x, src_y)
                dst_x, dst_y = self.world_to_cell(info, world_x, world_y)

                if not self.cell_in_bounds(info, dst_x, dst_y):
                    continue

                if value >= self.occupied_input_threshold:
                    delta = weight * self.logodds_occ
                elif value == 0:
                    if self.should_suppress_local_free_evidence(robot_id, dst_x, dst_y, info):
                        continue
                    delta = weight * self.logodds_free
                else:
                    continue

                log_odds[dst_y, dst_x] += delta
                observed[dst_y, dst_x] = True

    def apply_active_claims_to_log_odds(self, log_odds, observed, info) -> None:
        for evidence in self.active_claims.values():
            weight = self.fusion_policy.weight_for_claim(evidence, self.view_robot_id)
            if weight <= 0.0:
                continue

            if evidence.cell_x >= 0 and evidence.cell_y >= 0:
                center_x = evidence.cell_x
                center_y = evidence.cell_y
            else:
                center_x, center_y = self.world_to_cell(info, evidence.world_x, evidence.world_y)

            if not self.cell_in_bounds(info, center_x, center_y):
                continue

            if evidence.occupied:
                delta = weight * self.logodds_occ * self.fake_claim_logodds_multiplier
            else:
                delta = weight * self.logodds_free

            radius = max(0, int(self.fake_report_radius_cells))
            for y in range(max(0, center_y - radius), min(int(info.height), center_y + radius + 1)):
                for x in range(max(0, center_x - radius), min(int(info.width), center_x + radius + 1)):
                    if (x - center_x) ** 2 + (y - center_y) ** 2 > radius ** 2:
                        continue
                    log_odds[y, x] += delta
                    observed[y, x] = True

            self.get_logger().info(
                f'[{self.view_robot_id}] applied claim_id={evidence.claim_id} '
                f'delta={delta:.3f} radius={radius}'
            )

    def apply_current_observation_precedence(self, log_odds, observed, info) -> None:
        if not self.current_observation_override_enabled:
            return

        current = self.current_observation_map
        if current is None:
            return
        if not self.same_geometry(info, current.info):
            self.get_logger().warning(
                f'[{self.view_robot_id}] skipped current observation override because geometries differ'
            )
            return

        width = int(info.width)
        for index, raw_value in enumerate(current.data):
            value = int(raw_value)
            if value == -1:
                continue

            x = index % width
            y = index // width

            if value >= self.current_observation_occupied_threshold:
                log_odds[y, x] = max(float(log_odds[y, x]), self.current_observation_force_occupied_logodds)
                observed[y, x] = True
            elif value == self.current_observation_free_value or value <= self.current_observation_free_value:
                log_odds[y, x] = min(float(log_odds[y, x]), self.current_observation_force_free_logodds)
                observed[y, x] = True

    def remove_claims_contradicted_by_current_observation(self, info) -> None:
        if not self.current_observation_claim_clear_enabled:
            return

        current = self.current_observation_map
        if current is None:
            return
        if not self.same_geometry(info, current.info):
            return

        claim_ids_to_remove = []
        for claim_id, evidence in self.active_claims.items():
            if evidence.target_robot_id and evidence.target_robot_id != self.view_robot_id:
                continue

            if evidence.cell_x >= 0 and evidence.cell_y >= 0:
                center_x = evidence.cell_x
                center_y = evidence.cell_y
            else:
                center_x, center_y = self.world_to_cell(info, evidence.world_x, evidence.world_y)

            radius = self.current_observation_claim_clear_radius_cells
            if radius < 0:
                radius = self.fake_report_radius_cells
            radius = max(0, int(radius))

            free_count = 0
            occupied_count = 0
            observed_count = 0
            for y in range(max(0, center_y - radius), min(int(info.height), center_y + radius + 1)):
                for x in range(max(0, center_x - radius), min(int(info.width), center_x + radius + 1)):
                    if (x - center_x) ** 2 + (y - center_y) ** 2 > radius ** 2:
                        continue
                    idx = y * int(info.width) + x
                    value = int(current.data[idx])
                    if value == -1:
                        continue
                    observed_count += 1
                    if value >= self.current_observation_occupied_threshold:
                        occupied_count += 1
                    elif value == self.current_observation_free_value or value <= self.current_observation_free_value:
                        free_count += 1

            if observed_count <= 0:
                continue

            ratio_threshold = max(0.0, min(1.0, self.current_observation_claim_clear_ratio_threshold))
            if evidence.occupied:
                contradicted = free_count > 0 and occupied_count == 0 and (free_count / observed_count) >= ratio_threshold
            else:
                contradicted = occupied_count > 0 and free_count == 0 and (occupied_count / observed_count) >= ratio_threshold

            if contradicted:
                claim_ids_to_remove.append(claim_id)

        if not claim_ids_to_remove:
            return

        for claim_id in claim_ids_to_remove:
            removed = self.active_claims.pop(claim_id, None)
            if removed is not None:
                self.get_logger().info(
                    f'[{self.view_robot_id}] cleared active claim_id={claim_id} using own current LiDAR observation'
                )

    def should_suppress_local_free_evidence(self, robot_id: str, cell_x: int, cell_y: int, info) -> bool:
        if not self.suppress_attacker_self_free_evidence:
            return False

        for evidence in self.active_claims.values():
            if evidence.reporting_robot_id != robot_id:
                continue
            if evidence.target_robot_id and evidence.target_robot_id != self.view_robot_id:
                continue
            if not evidence.occupied:
                continue

            if evidence.cell_x >= 0 and evidence.cell_y >= 0:
                center_x = evidence.cell_x
                center_y = evidence.cell_y
            else:
                center_x, center_y = self.world_to_cell(info, evidence.world_x, evidence.world_y)

            radius = max(0, int(self.fake_report_radius_cells))
            if (cell_x - center_x) ** 2 + (cell_y - center_y) ** 2 <= radius ** 2:
                self.get_logger().debug(
                    f'[{self.view_robot_id}] suppressed FREE local evidence from {robot_id} '
                    f'at cell=({cell_x}, {cell_y}) due to active claim_id={evidence.claim_id}'
                )
                return True

        return False

    @staticmethod
    def cell_to_world(info, cell_x: int, cell_y: int) -> tuple[float, float]:
        return (
            info.origin.position.x + (cell_x + 0.5) * info.resolution,
            info.origin.position.y + (cell_y + 0.5) * info.resolution,
        )

    @staticmethod
    def world_to_cell(info, world_x: float, world_y: float) -> tuple[int, int]:
        cell_x = int((world_x - info.origin.position.x) / info.resolution)
        cell_y = int((world_y - info.origin.position.y) / info.resolution)
        return cell_x, cell_y

    @staticmethod
    def cell_in_bounds(info, cell_x: int, cell_y: int) -> bool:
        return 0 <= cell_x < int(info.width) and 0 <= cell_y < int(info.height)

    def log_odds_to_probability(self, value: float) -> float:
        return 1.0 / (1.0 + math.exp(-float(value)))

    def log_odds_to_grid_data(self, log_odds, observed) -> list[int]:
        height, width = log_odds.shape
        data = []

        for y in range(height):
            for x in range(width):
                if not observed[y, x]:
                    data.append(-1)
                    continue

                p_occ = self.log_odds_to_probability(log_odds[y, x])
                if p_occ >= self.occupied_probability_threshold:
                    data.append(100)
                elif p_occ <= self.free_probability_threshold:
                    data.append(0)
                else:
                    data.append(-1)

        return data

    def build_confidence_grid(self, merged: OccupancyGrid, log_odds, observed) -> OccupancyGrid:
        confidence = OccupancyGrid()
        confidence.header = deepcopy(merged.header)
        confidence.info = deepcopy(merged.info)

        data = []
        denom = max(abs(self.logodds_min), abs(self.logodds_max), 1e-6)

        height, width = log_odds.shape
        for y in range(height):
            for x in range(width):
                if not observed[y, x]:
                    data.append(-1)
                    continue
                # Confidence is strongest when the cell is far from the neutral log-odds point.
                conf = int(min(100.0, abs(float(log_odds[y, x])) / denom * 100.0))
                data.append(conf)

        confidence.data = data
        return confidence

    @staticmethod
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    @staticmethod
    def same_geometry(first, second) -> bool:
        return (
            int(first.width) == int(second.width)
            and int(first.height) == int(second.height)
            and abs(float(first.resolution) - float(second.resolution)) <= 1e-9
            and abs(float(first.origin.position.x) - float(second.origin.position.x)) <= 1e-6
            and abs(float(first.origin.position.y) - float(second.origin.position.y)) <= 1e-6
        )


def main() -> None:
    rclpy.init()
    node = MapMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
