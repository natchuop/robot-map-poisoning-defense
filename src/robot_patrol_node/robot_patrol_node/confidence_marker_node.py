from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ConfidenceMarkerNode(Node):
    """Render trust-weighted semantic map overlays plus legend."""

    def __init__(self) -> None:
        super().__init__('confidence_marker')

        self.declare_parameter('map_topic', '/shared_live_map')
        self.declare_parameter('input_topic', '/shared_confidence_map')
        self.declare_parameter('source_map_topics', ['/robot_1/live_map', '/robot_2/live_map'])
        self.declare_parameter('output_topic', '/shared_confidence_markers')
        self.declare_parameter('marker_namespace', 'confidence')
        self.declare_parameter('overlay_alpha', 0.95)
        self.declare_parameter('cell_scale_z', 0.01)
        self.declare_parameter('legend_title', 'Trust Map')
        self.declare_parameter('occupied_confident_threshold', 70)
        self.declare_parameter('occupied_possible_threshold', 30)
        self.declare_parameter('free_confident_threshold', 60)

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.source_map_topics = list(self.get_parameter('source_map_topics').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.marker_namespace = str(self.get_parameter('marker_namespace').value)
        self.overlay_alpha = max(0.0, min(1.0, float(self.get_parameter('overlay_alpha').value)))
        self.cell_scale_z = max(0.002, float(self.get_parameter('cell_scale_z').value))
        self.legend_title = str(self.get_parameter('legend_title').value)
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

        ready_source_maps = [
            self.source_maps.get(topic)
            for topic in self.source_map_topics
            if self.source_maps.get(topic) is not None
        ]

        markers = MarkerArray()
        markers.markers.append(
            self.build_cells_marker(self.latest_map, self.latest_confidence, ready_source_maps)
        )
        markers.markers.extend(self.build_legend_markers(self.latest_map))
        self.publisher.publish(markers)

    def build_cells_marker(self, map_msg: OccupancyGrid, confidence_msg: OccupancyGrid, source_maps) -> Marker:
        marker = Marker()
        marker.header = map_msg.header
        marker.ns = f'{self.marker_namespace}_cells'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = map_msg.info.resolution
        marker.scale.y = map_msg.info.resolution
        marker.scale.z = self.cell_scale_z
        marker.lifetime = Duration(sec=0, nanosec=0)

        width = int(map_msg.info.width)
        resolution = float(map_msg.info.resolution)
        origin_x = float(map_msg.info.origin.position.x)
        origin_y = float(map_msg.info.origin.position.y)

        for index, occupancy_value in enumerate(map_msg.data):
            confidence = int(confidence_msg.data[index])
            if confidence < 0 and int(occupancy_value) < 0:
                continue

            col = index % width
            row = index // width
            point = Point()
            point.x = origin_x + ((col + 0.5) * resolution)
            point.y = origin_y + ((row + 0.5) * resolution)
            point.z = 0.01
            marker.points.append(point)
            marker.colors.append(
                self.color_for_cell(
                    int(occupancy_value),
                    confidence,
                    [int(source_map.data[index]) for source_map in source_maps if self.same_geometry(map_msg, source_map)],
                )
            )

        return marker

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
            ('Blue: safe', self.blue_color()),
            ('Gray: unknown', self.gray_color()),
            ('Orange: maybe obstacle', self.orange_color()),
            ('Red: obstacle', self.red_color()),
            ('Purple: suspicious', self.purple_color()),
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

    def color_for_cell(self, occupancy: int, confidence: int, source_values) -> ColorRGBA:
        if self.is_disputed(source_values):
            return self.purple_color()
        if occupancy >= 65:
            if confidence >= self.occupied_confident_threshold:
                return self.red_color()
            if confidence >= self.occupied_possible_threshold:
                return self.orange_color()
            return self.gray_color()
        if occupancy == 0:
            if confidence >= self.free_confident_threshold:
                return self.blue_color()
            return self.gray_color()
        return self.gray_color()

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

    def blue_color(self) -> ColorRGBA:
        return ColorRGBA(r=0.00, g=0.32, b=1.00, a=self.overlay_alpha)

    def gray_color(self) -> ColorRGBA:
        return ColorRGBA(r=0.22, g=0.22, b=0.22, a=self.overlay_alpha)

    def orange_color(self) -> ColorRGBA:
        return ColorRGBA(r=1.00, g=0.55, b=0.00, a=self.overlay_alpha)

    def red_color(self) -> ColorRGBA:
        return ColorRGBA(r=1.00, g=0.00, b=0.00, a=self.overlay_alpha)

    def purple_color(self) -> ColorRGBA:
        return ColorRGBA(r=0.72, g=0.00, b=0.95, a=self.overlay_alpha)


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
