import json
import math
import socket
import threading

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class UdpBridgeNode(Node):
    """Receive Webots pose + LiDAR packets and republish ROS 2 topics.

    The original bridge used UDP. It now also listens on TCP because Docker
    Desktop forwards TCP localhost traffic from Windows much more reliably.
    """

    def __init__(self) -> None:
        super().__init__('udp_bridge')

        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 5005)
        self.declare_parameter('scan_frame', 'laser')

        self.listen_host = self.get_parameter('listen_host').value
        self.listen_port = int(self.get_parameter('listen_port').value)
        self.scan_frame = self.get_parameter('scan_frame').value

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/robot_pose', 10)

        self._shutdown = threading.Event()
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

        self.get_logger().info(
            f'Listening for Webots packets on udp/tcp://{self.listen_host}:{self.listen_port}'
        )

    def _udp_recv_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                payload, _addr = self._udp_socket.recvfrom(65535)
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
            thread = threading.Thread(target=self._tcp_client_loop, args=(conn,), daemon=True)
            thread.start()

    def _tcp_client_loop(self, conn: socket.socket) -> None:
        buffer = b''
        with conn:
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

    def _handle_payload(self, payload: bytes, transport: str) -> None:
        try:
            packet = json.loads(payload.decode('utf-8'))
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Ignored malformed {transport} packet: {exc}')
            return

        self._packet_count += 1
        if self._packet_count <= 3 or self._packet_count % 20 == 0:
            self.get_logger().info(
                f'Received {transport} packet #{self._packet_count} with keys: {sorted(packet.keys())}'
            )
        self._publish_packet(packet)

    def _publish_packet(self, packet: dict) -> None:
        pose = packet.get('pose', {})
        scan = packet.get('scan', {})

        pose_msg = Pose2D()
        pose_msg.x = float(pose.get('x', 0.0))
        pose_msg.y = float(pose.get('y', 0.0))
        pose_msg.theta = float(pose.get('theta', 0.0))
        self.pose_pub.publish(pose_msg)

        ranges = scan.get('ranges', [])
        if not ranges:
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.scan_frame
        scan_msg.angle_min = float(scan.get('angle_min', -math.pi))
        scan_msg.angle_max = float(scan.get('angle_max', math.pi))
        scan_msg.angle_increment = float(
            scan.get('angle_increment', (scan_msg.angle_max - scan_msg.angle_min) / max(len(ranges), 1))
        )
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = float(scan.get('scan_time', 0.0))
        scan_msg.range_min = float(scan.get('range_min', 0.05))
        scan_msg.range_max = float(scan.get('range_max', 4.0))
        scan_msg.ranges = [float(value) for value in ranges]
        scan_msg.intensities = []
        self.scan_pub.publish(scan_msg)

    def destroy_node(self) -> bool:
        self._shutdown.set()
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
