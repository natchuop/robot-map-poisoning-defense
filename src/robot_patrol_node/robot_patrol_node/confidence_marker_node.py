from dataclasses import dataclass

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class SourceContribution:
    source_index: int
    robot_id: str
    source_value: int
    state: str
    weight: float


class ConfidenceMarkerNode(Node):
    """Render trust-weighted semantic map overlays plus legend."""

    def __init__(self) -> None:
        super().__init__('confidence_marker')

        self.declare_parameter('map_topic', '/shared_live_map')
        self.declare_parameter('input_topic', '/robot_1/occupied_confidence_map')
        self.declare_parameter('source_map_topics', ['/robot_1/live_map', '/robot_2/live_map'])
        self.declare_parameter('source_weights', [1.0])
        self.declare_parameter('output_topic', '/shared_confidence_markers')
        self.declare_parameter('marker_namespace', 'confidence')
        self.declare_parameter('color_blending_enabled', True)
        self.declare_parameter('dispute_overlay_enabled', True)
        self.declare_parameter('overlay_alpha', 0.95)
        self.declare_parameter('dispute_overlay_alpha', 0.35)
        self.declare_parameter('cell_scale_z', 0.01)
        self.declare_parameter('legend_title', 'Trust Map')
        self.declare_parameter('unknown_cells_visible', False)
        self.declare_parameter('source_occupied_threshold', 65)
        self.declare_parameter('source_free_threshold', 0)
        self.declare_parameter('occupied_confident_threshold', 70)
        self.declare_parameter('occupied_possible_threshold', 30)
        self.declare_parameter('free_confident_threshold', 60)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.source_map_topics = list(self.get_parameter('source_map_topics').value)
        self.source_robot_ids = [self._topic_to_robot_id(topic) for topic in self.source_map_topics]
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.marker_namespace = str(self.get_parameter('marker_namespace').value)
        self.color_blending_enabled = bool(self.get_parameter('color_blending_enabled').value)
        self.dispute_overlay_enabled = bool(self.get_parameter('dispute_overlay_enabled').value)
        self.overlay_alpha = max(0.0, min(1.0, float(self.get_parameter('overlay_alpha').value)))
        self.dispute_overlay_alpha = max(0.0, min(1.0, float(self.get_parameter('dispute_overlay_alpha').value)))
        self.cell_scale_z = max(0.002, float(self.get_parameter('cell_scale_z').value))
        self.legend_title = str(self.get_parameter('legend_title').value)
        self.unknown_cells_visible = bool(self.get_parameter('unknown_cells_visible').value)
        self.source_occupied_threshold = int(self.get_parameter('source_occupied_threshold').value)
        self.source_free_threshold = int(self.get_parameter('source_free_threshold').value)
        self.occupied_confident_threshold = max(
            0,
            min(100, int(self.get_parameter('occupied_confident_threshold').value)),
        )
        self.occupied_possible_threshold = max(
            0,
            min(self.occupied_confident_threshold, int(self.get_parameter('occupied_possible_threshold').value)),
        )
        self.free_confident_threshold = max(
            0,
            min(100, int(self.get_parameter('free_confident_threshold').value)),
        )
        self.source_weights = self._normalize_numeric_list(self.get_parameter('source_weights').value)
        if len(self.source_weights) < len(self.source_map_topics):
            self.source_weights.extend([1.0] * (len(self.source_map_topics) - len(self.source_weights)))
        elif len(self.source_weights) > len(self.source_map_topics):
            self.source_weights = self.source_weights[:len(self.source_map_topics)]

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(MarkerArray, self.output_topic, marker_qos)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            marker_qos,
        )
        self.confidence_subscription = self.create_subscription(
            OccupancyGrid,
            self.input_topic,
            self.confidence_callback,
            marker_qos,
        )
        self.source_map_subscriptions = []
        self.source_maps = {}
        self.latest_map = None
        self.latest_confidence = None

        for topic in self.source_map_topics:
            self.source_map_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, topic_name=topic: self.source_map_callback(topic_name, msg),
                    marker_qos,
                )
            )

        self.get_logger().info(
            f'Confidence marker ready: map={self.map_topic}, confidence={self.input_topic} -> {self.output_topic}'
        )

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.publish_if_ready()

    def confidence_callback(self, msg: OccupancyGrid) -> None:
        self.latest_confidence = msg
        self.publish_if_ready()

    def source_map_callback(self, topic_name: str, msg: OccupancyGrid) -> None:
        self.source_maps[topic_name] = msg
        self.publish_if_ready()

    def publish_if_ready(self) -> None:
        if self.latest_map is None or self.latest_confidence is None:
            return

        if not self.same_geometry(self.latest_map, self.latest_confidence):
            self.get_logger().warning('Map and confidence geometries do not match; skipping marker publish.')
            return

        ready_source_maps = [self.source_maps.get(topic) for topic in self.source_map_topics]

        markers = MarkerArray()
        markers.markers.extend(self.build_cells_markers(self.latest_map, self.latest_confidence, ready_source_maps))
        markers.markers.extend(self.build_legend_markers(self.latest_map))
        self.publisher.publish(markers)

    def build_cells_markers(self, map_msg: OccupancyGrid, confidence_msg: OccupancyGrid, source_maps) -> list[Marker]:
        base_marker = Marker()
        base_marker.header = map_msg.header
        base_marker.ns = f'{self.marker_namespace}_cells'
        base_marker.id = 0
        base_marker.type = Marker.CUBE_LIST
        base_marker.action = Marker.ADD
        base_marker.pose.orientation.w = 1.0
        base_marker.scale.x = map_msg.info.resolution
        base_marker.scale.y = map_msg.info.resolution
        base_marker.scale.z = self.cell_scale_z
        base_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        base_marker.lifetime = Duration(sec=0, nanosec=0)

        dispute_marker = Marker()
        dispute_marker.header = map_msg.header
        dispute_marker.ns = f'{self.marker_namespace}_dispute'
        dispute_marker.id = 1
        dispute_marker.type = Marker.CUBE_LIST
        dispute_marker.action = Marker.ADD
        dispute_marker.pose.orientation.w = 1.0
        dispute_marker.scale.x = map_msg.info.resolution
        dispute_marker.scale.y = map_msg.info.resolution
        dispute_marker.scale.z = self.cell_scale_z
        dispute_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        dispute_marker.lifetime = Duration(sec=0, nanosec=0)

        width = int(map_msg.info.width)
        resolution = float(map_msg.info.resolution)
        origin_x = float(map_msg.info.origin.position.x)
        origin_y = float(map_msg.info.origin.position.y)

        for index, occupancy_value in enumerate(map_msg.data):
            confidence = int(confidence_msg.data[index])
            occupancy_value = int(occupancy_value)
            if occupancy_value < 0 and not self.unknown_cells_visible:
                continue
            if confidence < 0 and occupancy_value < 0:
                continue

            contributions = self.collect_source_contributions(index, source_maps, map_msg)
            disputed = self.has_disputed_contribution(contributions)
            blended_color = self.blend_source_color(contributions)
            cell_color = self.color_for_blended_cell(int(occupancy_value), confidence, blended_color, disputed)
            col = index % width
            row = index // width
            point = Point()
            point.x = origin_x + ((col + 0.5) * resolution)
            point.y = origin_y + ((row + 0.5) * resolution)
            point.z = 0.01
            base_marker.points.append(point)
            base_marker.colors.append(cell_color)
            if disputed and self.dispute_overlay_enabled:
                dispute_point = Point()
                dispute_point.x = point.x
                dispute_point.y = point.y
                dispute_point.z = point.z + (self.cell_scale_z * 0.75)
                dispute_marker.points.append(dispute_point)
                dispute_marker.colors.append(self.dispute_overlay_color(confidence))

        markers = [base_marker]
        if dispute_marker.points:
            markers.append(dispute_marker)
        return markers

    def build_legend_markers(self, msg: OccupancyGrid):
        markers = []
        resolution = float(msg.info.resolution)
        width_m = float(msg.info.width) * resolution
        height_m = float(msg.info.height) * resolution
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        legend_x = origin_x + width_m + (0.55 * resolution * 10.0)
        legend_top_y = origin_y + height_m - (0.9 * resolution * 10.0)
        step_y = resolution * 6.0
        cube_scale = resolution * 2.2

        title = Marker()
        title.header = msg.header
        title.ns = f'{self.marker_namespace}_legend'
        title.id = 1
        title.type = Marker.TEXT_VIEW_FACING
        title.action = Marker.ADD
        title.pose.position.x = legend_x
        title.pose.position.y = legend_top_y + step_y
        title.pose.position.z = 0.20
        title.pose.orientation.w = 1.0
        title.scale.z = 0.22
        title.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        title.text = self.legend_title
        markers.append(title)

        legend_entries = [
            ('Robot hue: source robot', self.robot_hue_legend_color()),
            ('Mixed hue: overlapping robots', self.mixed_hue_legend_color()),
            ('Light shade: free/cleared', self.cleared_color()),
            ('Saturated shade: occupied', self.occupied_color()),
            ('Gray: unknown', self.gray_color()),
            ('Purple overlay: disputed', self.dispute_overlay_color(75)),
            ('Darkness/opacity: confidence', self.confidence_color()),
        ]

        for offset, (label_text, color) in enumerate(legend_entries):
            cube = Marker()
            cube.header = msg.header
            cube.ns = f'{self.marker_namespace}_legend'
            cube.id = 10 + offset
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = legend_x
            cube.pose.position.y = legend_top_y - (offset * step_y)
            cube.pose.position.z = 0.07
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube_scale
            cube.scale.y = cube_scale
            cube.scale.z = self.cell_scale_z * 4.0
            cube.color = color
            markers.append(cube)

            label = Marker()
            label.header = msg.header
            label.ns = f'{self.marker_namespace}_legend'
            label.id = 30 + offset
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = legend_x + (resolution * 4.0)
            label.pose.position.y = legend_top_y - (offset * step_y)
            label.pose.position.z = 0.14
            label.pose.orientation.w = 1.0
            label.scale.z = 0.16
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = label_text
            markers.append(label)

        return markers

    def collect_source_contributions(self, index: int, source_maps, map_msg: OccupancyGrid) -> list[SourceContribution]:
        contributions: list[SourceContribution] = []
        for source_index, source_map in enumerate(source_maps):
            if source_map is None:
                continue
            if not self.same_geometry(map_msg, source_map):
                continue
            source_value = int(source_map.data[index])
            if source_value < 0:
                continue
            robot_id = self.source_robot_ids[source_index] if source_index < len(self.source_robot_ids) else f'robot_{source_index + 1}'
            base_weight = self.source_weights[source_index] if source_index < len(self.source_weights) else 1.0
            if source_value >= self.source_occupied_threshold:
                state = 'occupied'
                value_weight = self._clamp01(source_value / 100.0)
                value_weight = max(0.1, value_weight)
            elif source_value <= self.source_free_threshold:
                state = 'free'
                value_weight = 1.0
            else:
                state = 'known'
                value_weight = 0.5

            contributions.append(
                SourceContribution(
                    source_index=source_index,
                    robot_id=robot_id,
                    source_value=source_value,
                    state=state,
                    weight=max(0.0, base_weight * value_weight),
                )
            )

        return contributions

    def blend_source_color(self, contributions: list[SourceContribution]) -> ColorRGBA:
        if not contributions:
            return self.gray_color()
        if not self.color_blending_enabled:
            color = self.color_for_robot(contributions[0].robot_id)
            return ColorRGBA(r=color.r, g=color.g, b=color.b, a=1.0)

        total_weight = sum(contribution.weight for contribution in contributions)
        if total_weight <= 0.0:
            return self.gray_color()

        sum_r = 0.0
        sum_g = 0.0
        sum_b = 0.0
        for contribution in contributions:
            color = self.color_for_robot(contribution.robot_id)
            sum_r += color.r * contribution.weight
            sum_g += color.g * contribution.weight
            sum_b += color.b * contribution.weight

        return ColorRGBA(
            r=self._clamp01(sum_r / total_weight),
            g=self._clamp01(sum_g / total_weight),
            b=self._clamp01(sum_b / total_weight),
            a=1.0,
        )

    def has_disputed_contribution(self, contributions: list[SourceContribution]) -> bool:
        has_occupied = any(contribution.state == 'occupied' for contribution in contributions)
        has_free = any(contribution.state == 'free' for contribution in contributions)
        return has_occupied and has_free

    def color_for_blended_cell(
        self,
        occupancy: int,
        confidence: int,
        blended_color: ColorRGBA,
        disputed: bool,
    ) -> ColorRGBA:
        if occupancy < 0:
            return self.gray_color()

        if occupancy >= self.occupied_confident_threshold:
            return self.with_confidence(self.mix_with_white(blended_color, 0.04), confidence, occupied=True)

        if occupancy <= self.occupied_possible_threshold:
            return self.with_confidence(self.mix_with_white(blended_color, 0.52), confidence, occupied=False)

        if disputed:
            return self.with_confidence(self.mix_with_white(blended_color, 0.24), confidence, disputed=True)

        shade = 0.18 + (0.34 * self._clamp01(occupancy / 100.0))
        return self.with_confidence(self.mix_with_white(blended_color, shade), confidence, occupied=False)

    def dispute_overlay_color(self, confidence: int) -> ColorRGBA:
        conf = self._clamp01(float(confidence) / 100.0)
        alpha = min(self.dispute_overlay_alpha, 0.18 + (0.42 * conf))
        return ColorRGBA(r=0.72, g=0.00, b=0.95, a=alpha)

    @staticmethod
    def same_geometry(first: OccupancyGrid, second: OccupancyGrid) -> bool:
        return (
            int(first.info.width) == int(second.info.width)
            and int(first.info.height) == int(second.info.height)
            and abs(float(first.info.resolution) - float(second.info.resolution)) <= 1e-9
            and abs(float(first.info.origin.position.x) - float(second.info.origin.position.x)) <= 1e-6
            and abs(float(first.info.origin.position.y) - float(second.info.origin.position.y)) <= 1e-6
        )

    @staticmethod
    def is_disputed(source_values) -> bool:
        has_occupied = any(value >= 65 for value in source_values)
        has_free = any(value == 0 for value in source_values)
        return has_occupied and has_free

    @staticmethod
    def _topic_to_robot_id(topic: str) -> str:
        parts = [part for part in str(topic).split('/') if part]
        if len(parts) >= 2:
            return parts[-2]
        return str(topic).strip('/') or 'robot'

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
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, value))

    def color_for_robot(self, robot_id: str) -> ColorRGBA:
        palette = [
            ColorRGBA(r=0.98, g=0.24, b=0.22, a=self.overlay_alpha),
            ColorRGBA(r=0.16, g=0.72, b=1.00, a=self.overlay_alpha),
            ColorRGBA(r=0.18, g=0.82, b=0.42, a=self.overlay_alpha),
            ColorRGBA(r=1.00, g=0.69, b=0.18, a=self.overlay_alpha),
            ColorRGBA(r=0.80, g=0.34, b=1.00, a=self.overlay_alpha),
        ]
        key = sum(ord(char) for char in robot_id)
        return palette[key % len(palette)]

    def with_confidence(self, color: ColorRGBA, confidence: int, occupied: bool = False, disputed: bool = False) -> ColorRGBA:
        conf = self._clamp01(float(confidence) / 100.0)
        alpha_base = 0.24 if disputed else (0.28 if occupied else 0.20)
        alpha = min(self.overlay_alpha, alpha_base + (0.72 * conf))

        if disputed:
            brightness = 0.78 - (0.30 * conf)
        elif occupied:
            brightness = 0.84 - (0.34 * conf)
        else:
            brightness = 0.88 - (0.26 * conf)

        return ColorRGBA(
            r=self._clamp01(color.r * brightness),
            g=self._clamp01(color.g * brightness),
            b=self._clamp01(color.b * brightness),
            a=alpha,
        )

    def mix_with_white(self, color: ColorRGBA, white_mix: float) -> ColorRGBA:
        mix = self._clamp01(white_mix)
        return ColorRGBA(
            r=self._clamp01((color.r * (1.0 - mix)) + mix),
            g=self._clamp01((color.g * (1.0 - mix)) + mix),
            b=self._clamp01((color.b * (1.0 - mix)) + mix),
            a=color.a,
        )

    def gray_color(self) -> ColorRGBA:
        return ColorRGBA(r=0.22, g=0.22, b=0.22, a=self.overlay_alpha)

    def confidence_color(self) -> ColorRGBA:
        return ColorRGBA(r=1.00, g=1.00, b=1.00, a=self.overlay_alpha)

    def occupied_color(self) -> ColorRGBA:
        return ColorRGBA(r=1.00, g=0.20, b=0.20, a=self.overlay_alpha)

    def cleared_color(self) -> ColorRGBA:
        return ColorRGBA(r=0.20, g=0.70, b=1.00, a=self.overlay_alpha)

    def robot_hue_legend_color(self) -> ColorRGBA:
        return self.color_for_robot('robot_1')

    def mixed_hue_legend_color(self) -> ColorRGBA:
        return self.mix_with_white(
            ColorRGBA(
                r=(self.color_for_robot('robot_1').r + self.color_for_robot('robot_2').r) / 2.0,
                g=(self.color_for_robot('robot_1').g + self.color_for_robot('robot_2').g) / 2.0,
                b=(self.color_for_robot('robot_1').b + self.color_for_robot('robot_2').b) / 2.0,
                a=self.overlay_alpha,
            ),
            0.12,
        )


def main() -> None:
    rclpy.init()
    node = ConfidenceMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
