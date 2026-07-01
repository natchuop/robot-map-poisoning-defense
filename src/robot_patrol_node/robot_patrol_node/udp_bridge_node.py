import json
import math
import socket
import threading

from geometry_msgs.msg import PointStamped, Pose2D, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def covariance_matrix(xy: float, yaw: float):
    covariance = [0.0] * 36
    covariance[0] = xy
    covariance[7] = xy
    covariance[14] = 1e-6
    covariance[21] = 1e-6
    covariance[28] = 1e-6
    covariance[35] = yaw
    return covariance


class UdpBridgeNode(Node):
    """Receive Webots packets and forward ROS commands back to Webots."""

    def __init__(self) -> None:
        super().__init__('udp_bridge')

        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 5005)
        self.declare_parameter('scan_frame', 'laser')
        self.declare_parameter('max_publish_hz', 25.0)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('checkpoint_contact_topic', '/webots_checkpoint_contact')
        self.declare_parameter('checkpoint_event_topic', '/webots_checkpoint_event')
        self.declare_parameter('active_checkpoint_topic', '/active_checkpoint')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('covariance_xy', 0.05)
        self.declare_parameter('covariance_yaw', 0.1)

        self.listen_host = self.get_parameter('listen_host').value
        self.listen_port = int(self.get_parameter('listen_port').value)
        self.scan_frame = self.get_parameter('scan_frame').value
        self.max_publish_hz = float(self.get_parameter('max_publish_hz').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.checkpoint_contact_topic = self.get_parameter('checkpoint_contact_topic').value
        self.checkpoint_event_topic = self.get_parameter('checkpoint_event_topic').value
        self.active_checkpoint_topic = self.get_parameter('active_checkpoint_topic').value
        self.clicked_point_topic = self.get_parameter('clicked_point_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.covariance_xy = float(self.get_parameter('covariance_xy').value)
        self.covariance_yaw = float(self.get_parameter('covariance_yaw').value)
        self.min_publish_period_ns = int(
            1_000_000_000 / max(self.max_publish_hz, 1.0)
        )

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.pose_pub = self.create_publisher(Pose2D, self.pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.checkpoint_contact_pub = self.create_publisher(
            String,
            self.checkpoint_contact_topic,
            10,
        )
        self.clicked_point_pub = self.create_publisher(PointStamped, self.clicked_point_topic, 10)

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            10,
        )
        self.checkpoint_event_sub = self.create_subscription(
            String,
            self.checkpoint_event_topic,
            self._checkpoint_event_callback,
            10,
        )
        self.active_checkpoint_sub = self.create_subscription(
            String,
            self.active_checkpoint_topic,
            self._active_checkpoint_callback,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self._shutdown = threading.Event()
        self._tcp_clients = []
        self._tcp_clients_lock = threading.Lock()
        self._last_udp_addr = None
        self._last_publish_ns = None
        self._previous_pose = None
        self._previous_pose_stamp_ns = None

        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_socket.bind((self.listen_host, self.listen_port))
        self._udp_socket.settimeout(0.5)

        self._tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._tcp_socket.bind((self.listen_host, self.listen_port))
        self._tcp_socket.listen()
        self._tcp_socket.settimeout(0.5)

        self._udp_thread = threading.Thread(target=self._udp_recv_loop, daemon=True)
        self._tcp_thread = threading.Thread(target=self._tcp_accept_loop, daemon=True)
        self._udp_thread.start()
        self._tcp_thread.start()

        self._packet_count = 0
        self._published_count = 0

        self.get_logger().info(
            f'Listening for Webots packets on udp/tcp://{self.listen_host}:{self.listen_port}'
        )
        self.get_logger().info(
            f'Publishing {self.pose_topic} and {self.scan_topic} at up to '
            f'{self.max_publish_hz:.1f} Hz'
        )
        if self.publish_odom:
            self.get_logger().info(
                f'Publishing synchronized {self.odom_topic} and '
                f'{self.odom_frame}->{self.base_frame} TF from Webots packets'
            )
            self.get_logger().info(
                f'Pose-to-odom ready: pose={self.pose_topic}, odom={self.odom_topic}, '
                f'frames={self.odom_frame}->{self.base_frame}'
            )
        self.get_logger().info(
            f'Forwarding ROS {self.cmd_vel_topic} and checkpoint state back to Webots'
        )

    def _send_to_webots(self, packet: dict) -> bool:
        payload = json.dumps(packet, separators=(',', ':')).encode('utf-8') + b'\n'
        sent_any = False

        with self._tcp_clients_lock:
            dead_clients = []
            for conn in self._tcp_clients:
                try:
                    conn.sendall(payload)
                    sent_any = True
                except OSError:
                    dead_clients.append(conn)

            for conn in dead_clients:
                try:
                    self._tcp_clients.remove(conn)
                except ValueError:
                    pass
                try:
                    conn.close()
                except OSError:
                    pass

        if self._last_udp_addr is not None:
            try:
                self._udp_socket.sendto(payload, self._last_udp_addr)
                sent_any = True
            except OSError:
                pass

        return sent_any

    def _cmd_vel_callback(self, msg: Twist) -> None:
        sent_any = self._send_to_webots({
            'cmd_vel': {
                'linear_x': float(msg.linear.x),
                'angular_z': float(msg.angular.z),
            }
        })

        if not sent_any:
            self.get_logger().warning(
                f'Received {self.cmd_vel_topic}, but no Webots bridge connection is available yet'
            )

    def _active_checkpoint_callback(self, msg: String) -> None:
        self._send_to_webots({
            'active_checkpoint': {
                'name': msg.data,
            }
        })

    def _clicked_point_callback(self, packet: dict) -> None:
        clicked_point = packet.get('clicked_point')
        if not isinstance(clicked_point, dict):
            return

        msg = PointStamped()
        msg.header.frame_id = str(clicked_point.get('frame_id', 'map')).strip() or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(clicked_point.get('x', 0.0))
        msg.point.y = float(clicked_point.get('y', 0.0))
        msg.point.z = float(clicked_point.get('z', 0.0))
        self.clicked_point_pub.publish(msg)

    def _checkpoint_event_callback(self, msg: String) -> None:
        if not msg.data:
            return

        sent_any = self._send_to_webots({
            'checkpoint_event': {
                'message': msg.data,
            }
        })

        if sent_any:
            self.get_logger().info(f'Forwarded checkpoint event to Webots: {msg.data}')
        else:
            self.get_logger().debug(
                'Received checkpoint event, but no Webots bridge connection is available yet'
            )

    def _udp_recv_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                payload, addr = self._udp_socket.recvfrom(65535)
                self._last_udp_addr = addr
            except socket.timeout:
                continue
            except OSError:
                break

            self._handle_payload(payload, 'UDP')

    def _tcp_accept_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                conn, addr = self._tcp_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(f'Accepted TCP Webots bridge connection from {addr}')

            with self._tcp_clients_lock:
                self._tcp_clients.append(conn)

            thread = threading.Thread(
                target=self._tcp_client_loop,
                args=(conn,),
                daemon=True,
            )
            thread.start()

    def _tcp_client_loop(self, conn: socket.socket) -> None:
        buffer = b''

        try:
            conn.settimeout(0.5)

            while not self._shutdown.is_set():
                try:
                    chunk = conn.recv(65535)
                except socket.timeout:
                    continue
                except OSError:
                    break

                if not chunk:
                    break

                buffer += chunk
                while b'\n' in buffer:
                    payload, buffer = buffer.split(b'\n', 1)
                    if payload:
                        self._handle_payload(payload, 'TCP')
        finally:
            with self._tcp_clients_lock:
                try:
                    self._tcp_clients.remove(conn)
                except ValueError:
                    pass

            try:
                conn.close()
            except OSError:
                pass

    def _handle_payload(self, payload: bytes, transport: str) -> None:
        try:
            packet = json.loads(payload.decode('utf-8'))
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Ignored malformed {transport} packet: {exc}')
            return

        checkpoint_contact = packet.get('checkpoint_contact')
        if isinstance(checkpoint_contact, dict):
            msg = String()
            msg.data = json.dumps(checkpoint_contact, separators=(',', ':'))
            self.checkpoint_contact_pub.publish(msg)
            name = str(checkpoint_contact.get('name', '')).strip()
            distance = checkpoint_contact.get('distance')
            if isinstance(distance, (int, float)):
                self.get_logger().info(
                    f'Webots reports checkpoint contact: {name} at {float(distance):.2f} m'
                )
            else:
                self.get_logger().info(f'Webots reports checkpoint contact: {name}')

        self._clicked_point_callback(packet)

        if 'pose' not in packet and 'scan' not in packet:
            return

        self._packet_count += 1
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if (
            self._last_publish_ns is not None
            and now_ns - self._last_publish_ns < self.min_publish_period_ns
        ):
            return

        self._last_publish_ns = now_ns
        self._published_count += 1

        if self._published_count <= 3 or self._published_count % 150 == 0:
            self.get_logger().info(
                f'Published Webots packet #{self._published_count} from {transport}; '
                f'received={self._packet_count}'
            )

        self._publish_packet(packet, now)

    def _publish_packet(self, packet: dict, stamp) -> None:
        pose = packet.get('pose', {})
        scan = packet.get('scan', {})

        x = float(pose.get('x', 0.0))
        y = float(pose.get('y', 0.0))
        theta = float(pose.get('theta', 0.0))

        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = theta
        self.pose_pub.publish(pose_msg)

        if self.publish_odom:
            self._publish_odom(x, y, theta, stamp)

        ranges = scan.get('ranges', [])
        if not ranges:
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp.to_msg()
        scan_msg.header.frame_id = self.scan_frame
        scan_msg.angle_min = float(scan.get('angle_min', -math.pi))
        scan_msg.angle_max = float(scan.get('angle_max', math.pi))
        scan_msg.angle_increment = float(
            scan.get(
                'angle_increment',
                (scan_msg.angle_max - scan_msg.angle_min) / max(len(ranges), 1),
            )
        )
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = float(scan.get('scan_time', 1.0 / max(self.max_publish_hz, 1.0)))
        scan_msg.range_min = float(scan.get('range_min', 0.05))
        scan_msg.range_max = float(scan.get('range_max', 4.0))
        scan_msg.ranges = [float(value) for value in ranges]
        scan_msg.intensities = []
        self.scan_pub.publish(scan_msg)

    def _publish_odom(self, x: float, y: float, theta: float, stamp) -> None:
        stamp_ns = stamp.nanoseconds
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        if self._previous_pose is not None and self._previous_pose_stamp_ns is not None:
            dt = (stamp_ns - self._previous_pose_stamp_ns) / 1_000_000_000.0
            if dt > 1e-4:
                prev_x, prev_y, prev_theta = self._previous_pose
                world_vx = (x - prev_x) / dt
                world_vy = (y - prev_y) / dt
                angular_z = normalize_angle(theta - prev_theta) / dt
                linear_x = (math.cos(theta) * world_vx) + (math.sin(theta) * world_vy)
                linear_y = (-math.sin(theta) * world_vx) + (math.cos(theta) * world_vy)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(theta)
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z
        odom.pose.covariance = covariance_matrix(self.covariance_xy, self.covariance_yaw)
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = yaw_to_quaternion(theta)
        self.tf_broadcaster.sendTransform(transform)

        self._previous_pose = (x, y, theta)
        self._previous_pose_stamp_ns = stamp_ns

    def destroy_node(self) -> bool:
        self._shutdown.set()

        with self._tcp_clients_lock:
            for conn in self._tcp_clients:
                try:
                    conn.close()
                except OSError:
                    pass
            self._tcp_clients.clear()

        try:
            self._udp_socket.close()
        except OSError:
            pass

        try:
            self._tcp_socket.close()
        except OSError:
            pass

        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UdpBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
