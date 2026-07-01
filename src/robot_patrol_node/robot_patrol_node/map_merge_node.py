from copy import deepcopy
import json

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MapMergeNode(Node):
    """Merge per-robot occupancy and confidence grids into shared views."""

    def __init__(self) -> None:
        super().__init__('map_merge')

        self.declare_parameter('all_robot_ids', ['robot_1', 'robot_2'])
        self.declare_parameter('input_map_topics', ['/robot_1/live_map', '/robot_2/live_map'])
        self.declare_parameter(
            'input_confidence_topics',
            ['/robot_1/confidence_map', '/robot_2/confidence_map'],
        )
        self.declare_parameter('shared_map_topic', '')
        self.declare_parameter('shared_confidence_topic', '')
        self.declare_parameter('output_map_topic', '/shared_live_map')
        self.declare_parameter('output_confidence_topic', '/shared_confidence_map')
        self.declare_parameter('view_robot_id', 'robot_1')
        self.declare_parameter('trust_table_json', '{}')
        self.declare_parameter('map_updates_topic', '/map_updates')
        self.declare_parameter('fake_report_radius_cells', 2)
        self.declare_parameter('confidence_weights', [1.0, 1.0])
        self.declare_parameter('confidence_visual_gamma', 2.2)
        self.declare_parameter('confidence_visual_min', 0)

        self.view_robot_id = str(self.get_parameter('view_robot_id').value).strip() or 'robot_1'
        self.all_robot_ids = self._normalize_string_list(self.get_parameter('all_robot_ids').value)
        if self.view_robot_id not in self.all_robot_ids:
            self.all_robot_ids.append(self.view_robot_id)

        self.input_map_topics = self._resolve_topic_list(
            'input_map_topics',
            suffix='live_map',
        )
        self.input_confidence_topics = self._resolve_topic_list(
            'input_confidence_topics',
            suffix='confidence_map',
        )
        self.shared_map_topic = self._resolve_topic(
            'shared_map_topic',
            'output_map_topic',
            f'/{self.view_robot_id}/shared_live_map',
        )
        self.shared_confidence_topic = self._resolve_topic(
            'shared_confidence_topic',
            'output_confidence_topic',
            f'/{self.view_robot_id}/shared_confidence_map',
        )
        self.map_updates_topic = str(self.get_parameter('map_updates_topic').value).strip() or '/map_updates'
        self.fake_report_radius_cells = max(0, int(self.get_parameter('fake_report_radius_cells').value))
        self.trust_table = self._parse_trust_table(self.get_parameter('trust_table_json').value)
        self.confidence_visual_gamma = max(
            0.1,
            float(self.get_parameter('confidence_visual_gamma').value),
        )
        self.confidence_visual_min = max(
            0,
            min(100, int(self.get_parameter('confidence_visual_min').value)),
        )
        self.confidence_weights = [
            max(0.0, min(1.0, float(value)))
            for value in self._normalize_numeric_list(self.get_parameter('confidence_weights').value)
        ]
        self.confidence_weights = self._derive_confidence_weights(self.confidence_weights)
        if self.input_confidence_topics and len(self.confidence_weights) < len(self.input_confidence_topics):
            self.confidence_weights.extend(
                [1.0] * (len(self.input_confidence_topics) - len(self.confidence_weights))
            )
        elif len(self.confidence_weights) > len(self.input_confidence_topics):
            self.confidence_weights = self.confidence_weights[:len(self.input_confidence_topics)]

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        updates_qos = QoSProfile(depth=10)
        updates_qos.reliability = ReliabilityPolicy.RELIABLE
        updates_qos.durability = DurabilityPolicy.VOLATILE

        self.map_messages = {}
        self.confidence_messages = {}
        self.fake_reports = []
        self.fake_report_keys = set()
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

        for topic in self.input_confidence_topics:
            self.topic_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, topic_name=topic: self.confidence_callback(topic_name, msg),
                    map_qos,
                )
            )

        self.topic_subscriptions.append(
            self.create_subscription(
                String,
                self.map_updates_topic,
                self.map_update_callback,
                updates_qos,
            )
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.shared_map_topic, map_qos)
        self.confidence_pub = self.create_publisher(
            OccupancyGrid,
            self.shared_confidence_topic,
            map_qos,
        )

        self.get_logger().info(
            'Map merge ready: '
            f'view={self.view_robot_id} '
            f'all_robot_ids={self.all_robot_ids} '
            f'maps={self.input_map_topics} -> {self.shared_map_topic}; '
            f'confidence={self.input_confidence_topics} -> {self.shared_confidence_topic}; '
            f'updates={self.map_updates_topic}; '
            f'fake_radius={self.fake_report_radius_cells}; '
            f'weights={", ".join(f"{weight:.2f}" for weight in self.confidence_weights)}; '
            f'gamma={self.confidence_visual_gamma:.2f}'
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
    def _normalize_numeric_list(raw_value) -> list[float]:
        if isinstance(raw_value, (list, tuple, set)):
            return [float(value) for value in raw_value]
        if raw_value is None:
            return []
        if isinstance(raw_value, str):
            parts = [part.strip() for part in raw_value.split(',') if part.strip()]
            return [float(part) for part in parts]
        return [float(raw_value)]

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

    def _derive_confidence_weights(self, fallback_weights: list[float]) -> list[float]:
        if not self.all_robot_ids:
            return list(fallback_weights)

        derived_weights: list[float] = []
        for robot_id in self.all_robot_ids:
            if robot_id == self.view_robot_id:
                derived_weights.append(1.0)
                continue

            trust_entry = self.trust_table.get(robot_id, {})
            if isinstance(trust_entry, dict):
                trust_value = trust_entry.get('trust', None)
            else:
                trust_value = None

            try:
                derived_weights.append(max(0.0, min(1.0, float(trust_value))))
            except (TypeError, ValueError):
                derived_weights.append(fallback_weights[len(derived_weights)] if len(fallback_weights) > len(derived_weights) else 1.0)

        if derived_weights:
            return derived_weights
        return list(fallback_weights)

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

    def map_callback(self, topic_name: str, msg: OccupancyGrid) -> None:
        self.map_messages[topic_name] = msg
        self.publish_merged_map()

    def confidence_callback(self, topic_name: str, msg: OccupancyGrid) -> None:
        self.confidence_messages[topic_name] = msg
        self.publish_merged_confidence()

    def publish_merged_map(self) -> None:
        merged = self.merge_messages(
            [self.map_messages.get(topic) for topic in self.input_map_topics],
            lambda values: self.merge_occupancy_values(values, self.confidence_weights),
        )
        if merged is None:
            return

        self.apply_fake_reports(merged)
        self.map_pub.publish(merged)
        self.get_logger().info(f'[{self.view_robot_id}] published {self.shared_map_topic}')

    def publish_merged_confidence(self) -> None:
        if not self.input_confidence_topics:
            return

        confidence_messages = [
            self.confidence_messages.get(topic)
            for topic in self.input_confidence_topics
        ]
        merged = self.merge_messages(
            confidence_messages,
            lambda values: self.merge_confidence_values(values, self.confidence_weights),
        )
        if merged is None:
            return

        self.confidence_pub.publish(merged)
        self.get_logger().info(f'[{self.view_robot_id}] published {self.shared_confidence_topic}')

    def merge_messages(self, messages, merge_value_fn):
        ready_messages = [msg for msg in messages if msg is not None]
        if not ready_messages:
            return None

        reference = ready_messages[0]
        width = int(reference.info.width)
        height = int(reference.info.height)
        resolution = float(reference.info.resolution)
        origin_x = float(reference.info.origin.position.x)
        origin_y = float(reference.info.origin.position.y)

        for msg in ready_messages[1:]:
            if (
                int(msg.info.width) != width
                or int(msg.info.height) != height
                or abs(float(msg.info.resolution) - resolution) > 1e-9
                or abs(float(msg.info.origin.position.x) - origin_x) > 1e-6
                or abs(float(msg.info.origin.position.y) - origin_y) > 1e-6
            ):
                self.get_logger().warning(
                    'Skipped merge because map geometries differ; keep map sizes/origins aligned.'
                )
                return None

        merged = OccupancyGrid()
        merged.header = deepcopy(reference.header)
        merged.info = deepcopy(reference.info)
        merged.data = [merge_value_fn(values) for values in zip(*(msg.data for msg in ready_messages))]
        return merged

    def merge_occupancy_values(self, values, weights) -> int:
        weighted_values = []
        for index, value in enumerate(values):
            numeric_value = int(value)
            if numeric_value < 0:
                continue
            weight = weights[index] if index < len(weights) else 1.0
            weighted_values.append((weight, numeric_value))

        if not weighted_values:
            return -1

        best_weight = max(weight for weight, _value in weighted_values)
        best_values = [value for weight, value in weighted_values if abs(weight - best_weight) <= 1e-9]
        if any(value == 0 for value in best_values):
            return 0
        if any(value >= 65 for value in best_values):
            return 100
        return max(best_values)

    def merge_confidence_values(self, values, weights) -> int:
        weighted_values = []
        for index, value in enumerate(values):
            numeric_value = int(value)
            if numeric_value < 0:
                continue
            weight = weights[index] if index < len(weights) else 1.0
            if numeric_value == 0 or weight <= 0.0:
                weighted_values.append(0)
                continue

            contrasted_weight = pow(weight, self.confidence_visual_gamma)
            contrasted_value = max(
                self.confidence_visual_min,
                int(round(numeric_value * contrasted_weight)),
            )
            weighted_values.append(contrasted_value)
        if not weighted_values:
            return -1
        return max(0, min(100, max(weighted_values)))

    def map_update_callback(self, msg: String) -> None:
        self.get_logger().info(f'[{self.view_robot_id}] received /map_updates payload={msg.data}')

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'[{self.view_robot_id}] ignored malformed /map_updates JSON payload.')
            return

        if not isinstance(payload, dict):
            self.get_logger().warning(f'[{self.view_robot_id}] ignored /map_updates payload that was not a JSON object.')
            return

        if payload.get('type') != 'fake_obstacle':
            return

        reporting_robot = str(payload.get('reporting_robot', '')).strip() or 'unknown'
        target_robot = str(payload.get('target_robot', '')).strip() or 'all'
        self.get_logger().info(
            f'[{self.view_robot_id}] received /map_updates from {reporting_robot} '
            f'target_robot={target_robot}'
        )

        if reporting_robot == self.view_robot_id:
            self.get_logger().info(f'[{self.view_robot_id}] ignoring own report.')
            return

        if target_robot != self.view_robot_id:
            self.get_logger().info(
                f'[{self.view_robot_id}] ignoring report for target_robot={target_robot}'
            )
            return

        report = self._normalize_fake_report(payload)
        if report is None:
            self.get_logger().warning(
                f'[{self.view_robot_id}] ignored /map_updates payload missing usable coordinates.'
            )
            return

        report_key = json.dumps(report, sort_keys=True, separators=(',', ':'))
        if report_key in self.fake_report_keys:
            self.get_logger().info(
                f'[{self.view_robot_id}] duplicate fake report ignored for reporting_robot={report["reporting_robot"]}'
            )
            return

        self.fake_report_keys.add(report_key)
        self.fake_reports.append(report)
        coordinate_label = (
            f'world=({report["x"]:.3f}, {report["y"]:.3f})'
            if 'x' in report
            else f'cell=({report["cell_x"]}, {report["cell_y"]})'
        )
        self.get_logger().warning(
            f'[{self.view_robot_id}] ACCEPTED fake report from {report["reporting_robot"]} '
            f'target_robot={report["target_robot"]} {coordinate_label}'
        )
        self.publish_merged_map()

    def _normalize_fake_report(self, payload: dict) -> dict | None:
        reporting_robot = str(payload.get('reporting_robot', '')).strip() or 'unknown'
        target_robot = str(payload.get('target_robot', '')).strip() or self.view_robot_id
        occupied = bool(payload.get('occupied', True))
        frame_id = str(payload.get('frame_id', 'map')).strip() or 'map'
        source = str(payload.get('source', 'fake_obstacle')).strip() or 'fake_obstacle'
        timestamp = payload.get('timestamp')

        if 'cell_x' in payload and 'cell_y' in payload:
            try:
                cell_x = int(payload['cell_x'])
                cell_y = int(payload['cell_y'])
            except (TypeError, ValueError):
                return None
            return {
                'reporting_robot': reporting_robot,
                'target_robot': target_robot,
                'occupied': occupied,
                'cell_x': cell_x,
                'cell_y': cell_y,
                'frame_id': frame_id,
                'source': source,
                'timestamp': timestamp,
            }

        if 'x' not in payload or 'y' not in payload:
            return None

        try:
            obstacle_x = float(payload['x'])
            obstacle_y = float(payload['y'])
        except (TypeError, ValueError):
            return None

        return {
            'reporting_robot': reporting_robot,
            'target_robot': target_robot,
            'occupied': occupied,
            'x': obstacle_x,
            'y': obstacle_y,
            'frame_id': frame_id,
            'source': source,
            'timestamp': timestamp,
        }

    def _resolve_fake_report_cell(self, merged: OccupancyGrid, report: dict) -> tuple[int, int] | None:
        width = int(merged.info.width)
        height = int(merged.info.height)
        if width <= 0 or height <= 0:
            return None

        if 'cell_x' in report and 'cell_y' in report:
            cell_x = int(report['cell_x'])
            cell_y = int(report['cell_y'])
        else:
            resolution = float(merged.info.resolution)
            origin_x = float(merged.info.origin.position.x)
            origin_y = float(merged.info.origin.position.y)
            cell_x = int((float(report['x']) - origin_x) / resolution)
            cell_y = int((float(report['y']) - origin_y) / resolution)

        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            self.get_logger().warning(
                f'[{self.view_robot_id}] fake obstacle outside map bounds: '
                f'cell=({cell_x}, {cell_y}) size={width}x{height}'
            )
            return None

        return cell_x, cell_y

    def apply_fake_reports(self, merged: OccupancyGrid) -> None:
        if not self.fake_reports:
            return

        remaining_reports = []
        for report in self.fake_reports:
            resolved_cell = self._resolve_fake_report_cell(merged, report)
            if resolved_cell is None:
                continue

            cell_x, cell_y = resolved_cell
            width = int(merged.info.width)
            height = int(merged.info.height)
            if width <= 0 or height <= 0:
                return

            index = (cell_y * width) + cell_x
            if 0 <= index < len(merged.data) and int(merged.data[index]) == 0:
                self.get_logger().info(
                    f'[{self.view_robot_id}] skipping stale fake obstacle at cell=({cell_x}, {cell_y}) '
                    f'because real evidence has cleared it'
                )
                continue

            remaining_reports.append(report)
            coordinate_label = (
                f'world=({report["x"]:.3f}, {report["y"]:.3f})'
                if 'x' in report
                else f'cell=({report["cell_x"]}, {report["cell_y"]})'
            )
            self.get_logger().info(
                f'[{self.view_robot_id}] painting fake obstacle cell=({cell_x}, {cell_y}) '
                f'radius={self.fake_report_radius_cells} {coordinate_label} '
                f'source={report["reporting_robot"]}->{report["target_robot"]}'
            )

            for row in range(
                max(0, cell_y - self.fake_report_radius_cells),
                min(height, cell_y + self.fake_report_radius_cells + 1),
            ):
                for col in range(
                    max(0, cell_x - self.fake_report_radius_cells),
                    min(width, cell_x + self.fake_report_radius_cells + 1),
                ):
                    if (col - cell_x) ** 2 + (row - cell_y) ** 2 > self.fake_report_radius_cells ** 2:
                        continue
                    index = (row * width) + col
                    merged.data[index] = 100 if report['occupied'] else 0

        self.fake_reports = remaining_reports


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
