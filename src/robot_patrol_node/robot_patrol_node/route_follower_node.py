from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D, Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


@dataclass(frozen=True)
class Checkpoint:
    x: float
    y: float


@dataclass(frozen=True)
class RouteDefinition:
    loop: bool
    checkpoint_order: list[str]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def package_config_path(*parts: str) -> Path:
    return Path(get_package_share_directory('robot_patrol_node')).joinpath(*parts)


def resolve_route_config_path(raw_path: str) -> Path:
    candidate = raw_path.strip()
    if not candidate:
        return package_config_path('config', 'route_multiple_robots_routes.json')

    path = Path(candidate).expanduser()
    if path.is_absolute():
        return path

    package_relative = package_config_path(candidate)
    if package_relative.exists():
        return package_relative

    return path.resolve(strict=False)


def load_route_config(route_config_path: Path) -> tuple[dict[str, Checkpoint], dict[str, RouteDefinition]]:
    with route_config_path.open('r', encoding='utf-8') as handle:
        payload = json.load(handle)

    if not isinstance(payload, dict):
        raise RuntimeError(f'Invalid route config payload: {route_config_path}')

    checkpoints_raw = payload.get('checkpoints', {})
    routes_raw = payload.get('routes', {})
    if not isinstance(checkpoints_raw, dict) or not isinstance(routes_raw, dict):
        raise RuntimeError(f'Invalid route config structure: {route_config_path}')

    checkpoints: dict[str, Checkpoint] = {}
    for checkpoint_name, checkpoint_data in checkpoints_raw.items():
        if not isinstance(checkpoint_data, dict):
            continue
        try:
            checkpoints[str(checkpoint_name).strip()] = Checkpoint(
                x=float(checkpoint_data['x']),
                y=float(checkpoint_data['y']),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                f'Invalid checkpoint definition for {checkpoint_name!r} in {route_config_path}'
            ) from exc

    routes: dict[str, RouteDefinition] = {}
    for route_name, route_data in routes_raw.items():
        if not isinstance(route_data, dict):
            continue

        checkpoint_order = route_data.get('checkpoint_order', [])
        if not isinstance(checkpoint_order, list) or not checkpoint_order:
            continue

        routes[str(route_name).strip()] = RouteDefinition(
            loop=bool(route_data.get('loop', True)),
            checkpoint_order=[str(name).strip() for name in checkpoint_order if str(name).strip()],
        )

    if not checkpoints:
        raise RuntimeError(f'No checkpoints defined in {route_config_path}')
    if not routes:
        raise RuntimeError(f'No routes defined in {route_config_path}')

    return checkpoints, routes


class RouteFollowerNode(Node):
    def __init__(self):
        super().__init__('route_follower_node')

        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('route_config_path', '')
        self.declare_parameter('route_name', '')
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('active_checkpoint_topic', '/active_checkpoint')
        self.declare_parameter('checkpoint_event_topic', '/webots_checkpoint_event')
        self.declare_parameter('goal_tolerance', 0.18)
        self.declare_parameter('max_linear_x', 0.12)
        self.declare_parameter('slow_linear_x', 0.04)
        self.declare_parameter('max_angular_z', 0.9)
        self.declare_parameter('k_turn', 1.8)
        self.declare_parameter('heading_tolerance', 0.45)
        self.declare_parameter('blocked_front_range_m', 0.25)
        self.declare_parameter('control_rate_hz', 10.0)

        self.robot_id = str(self.get_parameter('robot_id').value).strip() or 'robot_1'
        self.route_config_path = resolve_route_config_path(
            str(self.get_parameter('route_config_path').value)
        )
        self.route_name = str(self.get_parameter('route_name').value).strip()
        self.pose_topic = str(self.get_parameter('pose_topic').value).strip()
        self.scan_topic = str(self.get_parameter('scan_topic').value).strip()
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value).strip()
        self.active_checkpoint_topic = str(self.get_parameter('active_checkpoint_topic').value).strip()
        self.checkpoint_event_topic = str(self.get_parameter('checkpoint_event_topic').value).strip()
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.slow_linear_x = float(self.get_parameter('slow_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.k_turn = float(self.get_parameter('k_turn').value)
        self.heading_tolerance = float(self.get_parameter('heading_tolerance').value)
        self.blocked_front_range_m = float(self.get_parameter('blocked_front_range_m').value)
        control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.control_period = 1.0 / control_rate_hz

        self.checkpoints, self.routes = load_route_config(self.route_config_path)
        if self.route_name:
            self.route = self.routes.get(self.route_name)
            if self.route is None:
                raise RuntimeError(
                    f'Route {self.route_name!r} not found in {self.route_config_path}'
                )
        elif len(self.routes) == 1:
            self.route_name, self.route = next(iter(self.routes.items()))
        else:
            available = ', '.join(sorted(self.routes))
            raise RuntimeError(
                f'route_name is required when {self.route_config_path} defines multiple routes '
                f'({available})'
            )

        self.pose_subscription = self.create_subscription(
            Pose2D,
            self.pose_topic,
            self.pose_callback,
            10,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.active_checkpoint_pub = self.create_publisher(
            String,
            self.active_checkpoint_topic,
            10,
        )
        self.checkpoint_event_pub = self.create_publisher(
            String,
            self.checkpoint_event_topic,
            10,
        )

        self.current_pose: Pose2D | None = None
        self.front_range_m: float | None = None
        self.current_checkpoint_index = 0
        self.current_checkpoint_name: str | None = None
        self.route_complete = False
        self.timer = self.create_timer(self.control_period, self.control_tick)

        checkpoint_order = ' -> '.join(self.route.checkpoint_order)
        self.get_logger().info(
            f'{self.robot_id}: loaded route {self.route_name!r} from {self.route_config_path}'
        )
        self.get_logger().info(
            f'{self.robot_id}: checkpoints {checkpoint_order}; '
            f'goal_tolerance={self.goal_tolerance:.2f} m'
        )

    def pose_callback(self, msg: Pose2D) -> None:
        self.current_pose = msg

    def scan_callback(self, msg: LaserScan) -> None:
        front_values: list[float] = []
        angle = float(msg.angle_min)
        for value in msg.ranges:
            if -0.25 <= angle <= 0.25 and math.isfinite(value):
                front_values.append(float(value))
            angle += float(msg.angle_increment)
        self.front_range_m = min(front_values) if front_values else None

    def publish_active_checkpoint(self, checkpoint_name: str) -> None:
        msg = String()
        msg.data = checkpoint_name
        self.active_checkpoint_pub.publish(msg)

    def publish_checkpoint_event(self, message: str) -> None:
        msg = String()
        msg.data = message
        self.checkpoint_event_pub.publish(msg)

    def current_checkpoint(self) -> tuple[str, Checkpoint] | None:
        if self.current_checkpoint_index < 0 or self.current_checkpoint_index >= len(self.route.checkpoint_order):
            return None

        checkpoint_name = self.route.checkpoint_order[self.current_checkpoint_index]
        checkpoint = self.checkpoints.get(checkpoint_name)
        if checkpoint is None:
            self.get_logger().warning(
                f'{self.robot_id}: skipping unknown checkpoint {checkpoint_name!r}'
            )
            self.current_checkpoint_index += 1
            return self.current_checkpoint()

        return checkpoint_name, checkpoint

    def advance_route(self) -> None:
        self.current_checkpoint_index += 1
        if self.current_checkpoint_index < len(self.route.checkpoint_order):
            return

        if self.route.loop:
            self.current_checkpoint_index = 0
            self.publish_checkpoint_event(
                f'{self.robot_id}: route {self.route_name} looped back to the start'
            )
        else:
            self.route_complete = True
            self.publish_active_checkpoint('')
            self.publish_checkpoint_event(
                f'{self.robot_id}: route {self.route_name} completed'
            )

    def control_tick(self) -> None:
        if self.route_complete:
            self.cmd_vel_pub.publish(Twist())
            return

        if self.current_pose is None:
            self.cmd_vel_pub.publish(Twist())
            return

        checkpoint_info = self.current_checkpoint()
        if checkpoint_info is None:
            self.route_complete = True
            self.publish_active_checkpoint('')
            self.publish_checkpoint_event(f'{self.robot_id}: route {self.route_name} completed')
            self.cmd_vel_pub.publish(Twist())
            return

        checkpoint_name, checkpoint = checkpoint_info
        if checkpoint_name != self.current_checkpoint_name:
            self.current_checkpoint_name = checkpoint_name
            self.publish_active_checkpoint(checkpoint_name)
            self.publish_checkpoint_event(
                f'{self.robot_id}: heading to checkpoint {checkpoint_name} '
                f'({checkpoint.x:.2f}, {checkpoint.y:.2f})'
            )
            self.get_logger().info(
                f'{self.robot_id}: targeting checkpoint {checkpoint_name} '
                f'({checkpoint.x:.2f}, {checkpoint.y:.2f})'
            )

        dx = checkpoint.x - float(self.current_pose.x)
        dy = checkpoint.y - float(self.current_pose.y)
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.publish_checkpoint_event(
                f'{self.robot_id}: reached checkpoint {checkpoint_name} at {distance:.2f} m'
            )
            self.get_logger().info(
                f'{self.robot_id}: reached checkpoint {checkpoint_name} at {distance:.2f} m'
            )
            self.cmd_vel_pub.publish(Twist())
            self.current_checkpoint_name = None
            self.publish_active_checkpoint('')
            self.advance_route()
            return

        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - float(self.current_pose.theta))
        angular_z = clamp(self.k_turn * heading_error, -self.max_angular_z, self.max_angular_z)

        if abs(heading_error) < self.heading_tolerance:
            linear_x = self.max_linear_x
        else:
            linear_x = self.slow_linear_x

        if self.front_range_m is not None and self.front_range_m <= self.blocked_front_range_m:
            linear_x = 0.0
            self.get_logger().debug(
                f'{self.robot_id}: front obstacle at {self.front_range_m:.2f} m; slowing to stop'
            )

        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RouteFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
