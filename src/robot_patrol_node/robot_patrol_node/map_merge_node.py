from copy import deepcopy

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapMergeNode(Node):
    """Merge per-robot occupancy and confidence grids into trust-weighted views."""

    def __init__(self) -> None:
        super().__init__('map_merge')

        self.declare_parameter('input_map_topics', ['/robot_1/live_map', '/robot_2/live_map'])
        self.declare_parameter(
            'input_confidence_topics',
            ['/robot_1/confidence_map', '/robot_2/confidence_map'],
        )
        self.declare_parameter('output_map_topic', '/shared_live_map')
        self.declare_parameter('output_confidence_topic', '/shared_confidence_map')
        self.declare_parameter('confidence_weights', [1.0, 1.0])
        self.declare_parameter('confidence_visual_gamma', 2.2)
        self.declare_parameter('confidence_visual_min', 0)

        self.input_map_topics = list(self.get_parameter('input_map_topics').value)
        self.input_confidence_topics = list(self.get_parameter('input_confidence_topics').value)
        self.output_map_topic = str(self.get_parameter('output_map_topic').value)
        self.output_confidence_topic = str(self.get_parameter('output_confidence_topic').value)
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
            for value in self.get_parameter('confidence_weights').value
        ]
        if len(self.confidence_weights) < len(self.input_confidence_topics):
            self.confidence_weights.extend(
                [1.0] * (len(self.input_confidence_topics) - len(self.confidence_weights))
            )
        elif len(self.confidence_weights) > len(self.input_confidence_topics):
            self.confidence_weights = self.confidence_weights[:len(self.input_confidence_topics)]

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_messages = {}
        self.confidence_messages = {}
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

        self.map_pub = self.create_publisher(OccupancyGrid, self.output_map_topic, map_qos)
        self.confidence_pub = self.create_publisher(
            OccupancyGrid,
            self.output_confidence_topic,
            map_qos,
        )

        self.get_logger().info(
            'Map merge ready: '
            f'maps={", ".join(self.input_map_topics)} -> {self.output_map_topic}; '
            f'confidence={", ".join(self.input_confidence_topics)} -> {self.output_confidence_topic}; '
            f'weights={", ".join(f"{weight:.2f}" for weight in self.confidence_weights)}; '
            f'gamma={self.confidence_visual_gamma:.2f}'
        )

    def map_callback(self, topic_name: str, msg: OccupancyGrid) -> None:
        self.map_messages[topic_name] = msg
        self.publish_merged_map()

    def confidence_callback(self, topic_name: str, msg: OccupancyGrid) -> None:
        self.confidence_messages[topic_name] = msg
        self.publish_merged_confidence()

    def publish_merged_map(self) -> None:
        merged = self.merge_messages(
            [self.map_messages.get(topic) for topic in self.input_map_topics],
            self.merge_occupancy_values,
        )
        if merged is not None:
            self.map_pub.publish(merged)

    def publish_merged_confidence(self) -> None:
        confidence_messages = [
            self.confidence_messages.get(topic)
            for topic in self.input_confidence_topics
        ]
        merged = self.merge_messages(
            confidence_messages,
            lambda values: self.merge_confidence_values(values, self.confidence_weights),
        )
        if merged is not None:
            self.confidence_pub.publish(merged)

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

    @staticmethod
    def merge_occupancy_values(values) -> int:
        filtered = [int(value) for value in values if int(value) >= 0]
        if not filtered:
            return -1
        if any(value >= 65 for value in filtered):
            return 100
        if any(value == 0 for value in filtered):
            return 0
        return max(filtered)

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
