from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from robot_patrol_msgs.msg import MapUpdate
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class FakeObstacleInjectorNode(Node):
    """Publish fake obstacle claims for manual or click-driven experiments."""

    def __init__(self) -> None:
        super().__init__('fake_obstacle_injector')

        self.declare_parameter('robot_id', 'robot_2')
        self.declare_parameter('reporting_robot', '')
        self.declare_parameter('all_robot_ids', ['robot_1', 'robot_2'])
        self.declare_parameter('compromised', False)
        self.declare_parameter('compromise_state_topic', '')
        self.declare_parameter('attack_targets', ['all_except_self'])
        self.declare_parameter('target_robot', 'all')
        self.declare_parameter('mode', 'manual')
        self.declare_parameter('map_updates_topic', '/map_updates')
        self.declare_parameter('clicked_point_topic', '')
        self.declare_parameter('marker_topic', '')
        self.declare_parameter('occupied', True)
        self.declare_parameter('obstacle_x', 1.5)
        self.declare_parameter('obstacle_y', -0.8)
        self.declare_parameter('source', 'manual_fixed')
        self.declare_parameter('publish_delay_sec', 0.5)
        self.declare_parameter('marker_lifetime_sec', 3.0)

        self.robot_id = self._resolve_robot_id()
        self.reporting_robot = self.robot_id
        self.all_robot_ids = self._resolve_all_robot_ids()
        self.compromised = bool(self.get_parameter('compromised').value)
        self.compromise_state_topic = self._resolve_topic(
            'compromise_state_topic',
            f'/{self.robot_id}/compromise_state',
        )
        self.attack_targets = self._resolve_attack_targets()
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.map_updates_topic = str(self.get_parameter('map_updates_topic').value)
        self.occupied = bool(self.get_parameter('occupied').value)
        self.obstacle_x = float(self.get_parameter('obstacle_x').value)
        self.obstacle_y = float(self.get_parameter('obstacle_y').value)
        self.source = str(self.get_parameter('source').value).strip() or 'manual_fixed'
        self.publish_delay_sec = max(0.0, float(self.get_parameter('publish_delay_sec').value))
        self.clicked_point_topic = self._resolve_topic(
            'clicked_point_topic',
            f'/{self.robot_id}/clicked_point',
        )
        self.marker_topic = self._resolve_topic(
            'marker_topic',
            f'/{self.robot_id}/fake_obstacle_markers',
        )
        self.marker_lifetime_sec = max(0.0, float(self.get_parameter('marker_lifetime_sec').value))

        updates_qos = QoSProfile(depth=10)
        updates_qos.reliability = ReliabilityPolicy.RELIABLE
        updates_qos.durability = DurabilityPolicy.VOLATILE
        self.publisher = self.create_publisher(MapUpdate, self.map_updates_topic, updates_qos)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publisher = self.create_publisher(MarkerArray, self.marker_topic, marker_qos)
        self.compromise_subscription = self.create_subscription(
            Bool,
            self.compromise_state_topic,
            self.compromise_state_callback,
            10,
        )
        self.clicked_point_subscription = None
        self.published_count = 0

        if self.mode == 'clicked_point':
            self.clicked_point_subscription = self.create_subscription(
                PointStamped,
                self.clicked_point_topic,
                self.clicked_point_callback,
                10,
            )
            self.timer = None
        else:
            self.timer = self.create_timer(self.publish_delay_sec, self.publish_manual_report)
        self.get_logger().info(
            'Fake obstacle injector ready: '
            f'robot_id={self.robot_id} compromised={self.compromised} '
            f'click_topic={self.clicked_point_topic} '
            f'updates={self.map_updates_topic} '
            f'marker_topic={self.marker_topic} '
            f'attack_targets={self.attack_targets} '
            f'mode={self.mode} '
            f'compromise_state_topic={self.compromise_state_topic}'
        )

    def marker_color(self) -> ColorRGBA:
        if self.robot_id == 'robot_1':
            return ColorRGBA(r=0.98, g=0.24, b=0.22, a=0.88)
        if self.robot_id == 'robot_2':
            return ColorRGBA(r=0.16, g=0.72, b=1.00, a=0.88)
        return ColorRGBA(r=0.95, g=0.55, b=0.10, a=0.88)

    def _resolve_robot_id(self) -> str:
        robot_id = str(self.get_parameter('robot_id').value).strip()
        if robot_id:
            return robot_id

        reporting_robot = str(self.get_parameter('reporting_robot').value).strip()
        if reporting_robot:
            return reporting_robot

        return 'robot_2'

    def _resolve_all_robot_ids(self) -> list[str]:
        raw_ids = self.get_parameter('all_robot_ids').value
        robot_ids = self._normalize_string_list(raw_ids)
        if self.robot_id not in robot_ids:
            robot_ids.append(self.robot_id)
        return robot_ids

    def _resolve_topic(self, parameter_name: str, default_topic: str) -> str:
        topic = str(self.get_parameter(parameter_name).value).strip()
        if topic:
            return topic
        return default_topic

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

    def _resolve_attack_targets(self) -> list[str]:
        attack_targets = self._normalize_string_list(self.get_parameter('attack_targets').value)
        if not attack_targets:
            legacy_target = str(self.get_parameter('target_robot').value).strip()
            if legacy_target and legacy_target != 'all':
                attack_targets = [legacy_target]
            else:
                attack_targets = ['all_except_self']

        if 'all_except_self' in attack_targets:
            resolved = [robot_id for robot_id in self.all_robot_ids if robot_id != self.robot_id]
        else:
            resolved = [robot_id for robot_id in attack_targets if robot_id != self.robot_id]

        deduped_targets = []
        for target_robot in resolved:
            if target_robot not in deduped_targets:
                deduped_targets.append(target_robot)
        return deduped_targets

    def compromise_state_callback(self, msg: Bool) -> None:
        self.compromised = bool(msg.data)
        self.get_logger().warning(f'[{self.robot_id}] compromised={self.compromised}')

    def make_claim_id(self, target_robot: str) -> str:
        now_ns = self.get_clock().now().nanoseconds
        return f'{self.robot_id}_fake_obstacle_{target_robot}_{self.published_count}_{now_ns}'

    def publish_fake_reports(self, obstacle_x: float, obstacle_y: float, source: str, frame_id: str) -> None:
        if not self.compromised:
            self.get_logger().info(
                f'[{self.robot_id}] received fake obstacle trigger but is not compromised; ignoring.'
            )
            return

        attack_targets = self.resolve_attack_targets()
        self.get_logger().info(f'[{self.robot_id}] attack targets={attack_targets}')

        if not attack_targets:
            self.get_logger().info(f'[{self.robot_id}] no attack targets resolved; nothing to publish.')
            return

        for target_robot in attack_targets:
            msg = MapUpdate()
            msg.claim_id = self.make_claim_id(target_robot)
            msg.reporting_robot_id = self.robot_id
            msg.target_robot_id = target_robot
            msg.cell_x = -1
            msg.cell_y = -1
            msg.world_x = float(obstacle_x)
            msg.world_y = float(obstacle_y)
            msg.reported_state = 'OCCUPIED' if self.occupied else 'FREE'
            msg.occupied = bool(self.occupied)
            msg.source = 'fake_obstacle_injector'
            msg.attack_type = 'fake_obstacle'
            msg.is_attack_report = True
            msg.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            self.get_logger().warning(
                f'[{self.robot_id}] publishing fake obstacle claim_id={msg.claim_id} '
                f'target_robot={target_robot} at x={obstacle_x:.3f} y={obstacle_y:.3f}'
            )

        self.publish_marker(obstacle_x, obstacle_y, frame_id)
        self.published_count += 1
        self.get_logger().info(
            f'[{self.robot_id}] published fake obstacle marker on {self.marker_topic}'
        )

    def resolve_attack_targets(self) -> list[str]:
        return list(self.attack_targets)

    def publish_manual_report(self) -> None:
        self.publish_fake_reports(self.obstacle_x, self.obstacle_y, self.source, 'map')
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def clicked_point_callback(self, msg: PointStamped) -> None:
        frame_id = msg.header.frame_id or 'map'
        self.get_logger().info(
            f'[{self.robot_id}] clicked point received x={msg.point.x:.3f} '
            f'y={msg.point.y:.3f} frame={frame_id} compromised={self.compromised}'
        )
        self.publish_fake_reports(msg.point.x, msg.point.y, 'clicked_point', frame_id)

    def publish_marker(self, obstacle_x: float, obstacle_y: float, frame_id: str) -> None:
        marker = Marker()
        marker.header.frame_id = frame_id or 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fake_obstacle_injector'
        marker.id = self.published_count
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(obstacle_x)
        marker.pose.position.y = float(obstacle_y)
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.20
        marker.color = self.marker_color()
        marker.lifetime = Duration(
            sec=int(self.marker_lifetime_sec),
            nanosec=int((self.marker_lifetime_sec - int(self.marker_lifetime_sec)) * 1_000_000_000),
        )

        markers = MarkerArray()
        markers.markers.append(marker)
        self.marker_publisher.publish(markers)


def main() -> None:
    rclpy.init()
    node = FakeObstacleInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
