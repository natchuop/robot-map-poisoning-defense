import math

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


CHECKPOINTS = {
    'A': (-1.49882, 1.84407),
    'B': (1.5267, -0.221987),
    'C': (-0.416565, -1.35783),
    'D': (-2.63149, -0.778393),
    '_B_EXIT': (-1.05, 2.20),
    '_B_CRUISE': (0.65, 1.75),
    '_B_APPROACH': (1.55, 0.65),
    '_D_APPROACH': (-1.65, -1.25),
    '_A_RETURN_0': (-2.20, -0.20),
    '_A_RETURN_1': (-1.45, 0.15),
    '_A_RETURN_2': (-0.85, 0.55),
    '_A_RETURN_3': (-0.90, 1.20),
}

CHECKPOINT_ROUTE = ['A', '_B_EXIT', '_B_CRUISE', '_B_APPROACH', 'B', 'C', '_D_APPROACH', 'D', '_A_RETURN_0', '_A_RETURN_1', '_A_RETURN_2', '_A_RETURN_3', 'A']


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class NavigationDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('navigation_diagnostics')

        self.declare_parameter('log_interval_sec', 2.0)
        self.declare_parameter('stale_after_sec', 1.5)
        self.declare_parameter('arrival_radius', 0.50)

        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.stale_after_sec = float(self.get_parameter('stale_after_sec').value)
        self.arrival_radius = float(self.get_parameter('arrival_radius').value)

        self.robot_pose = None
        self.robot_pose_time = None
        self.odom = None
        self.odom_time = None
        self.cmd_vel = None
        self.cmd_vel_time = None
        self.scan_time = None
        self.scan_min = None
        self.scan_front = None
        self.goal_status = 'none'
        self.active_checkpoint_name = None
        self.closest_route_index = 0

        self.create_subscription(Pose2D, '/robot_pose', self.robot_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.goal_status_callback,
            10,
        )
        self.create_subscription(String, '/active_checkpoint', self.active_checkpoint_callback, 10)

        self.timer = self.create_timer(self.log_interval_sec, self.log_status)
        self.get_logger().info(
            'Navigation diagnostics ready: logging pose, goal distance, cmd_vel, scan, and action status.'
        )

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_time = self.get_clock().now()

    def scan_callback(self, msg):
        self.scan_time = self.get_clock().now()
        finite_ranges = [value for value in msg.ranges if math.isfinite(value)]
        self.scan_min = min(finite_ranges) if finite_ranges else None

        if msg.ranges:
            mid = len(msg.ranges) // 2
            window = msg.ranges[max(0, mid - 6): min(len(msg.ranges), mid + 7)]
            finite_front = [value for value in window if math.isfinite(value)]
            self.scan_front = min(finite_front) if finite_front else None
        else:
            self.scan_front = None

    def active_checkpoint_callback(self, msg):
        self.active_checkpoint_name = msg.data or None

    def goal_status_callback(self, msg):
        if not msg.status_list:
            self.goal_status = 'none'
            return

        status = msg.status_list[-1].status
        names = {
            0: 'unknown',
            1: 'accepted',
            2: 'executing',
            3: 'canceling',
            4: 'succeeded',
            5: 'canceled',
            6: 'aborted',
        }
        self.goal_status = names.get(status, str(status))

    def seconds_since(self, stamp, now):
        if stamp is None:
            return None
        return (now - stamp).nanoseconds / 1_000_000_000.0

    def stale_text(self, name, stamp, now):
        age = self.seconds_since(stamp, now)
        if age is None:
            return f'{name}:missing'
        if age > self.stale_after_sec:
            return f'{name}:stale {age:.1f}s'
        return f'{name}:{age:.1f}s'

    def current_target(self):
        if self.active_checkpoint_name in CHECKPOINTS:
            return self.active_checkpoint_name, CHECKPOINTS[self.active_checkpoint_name]

        if self.robot_pose is None:
            name = CHECKPOINT_ROUTE[self.closest_route_index]
            return name, CHECKPOINTS[name]

        x = self.robot_pose.x
        y = self.robot_pose.y

        while self.closest_route_index < len(CHECKPOINT_ROUTE) - 1:
            name = CHECKPOINT_ROUTE[self.closest_route_index]
            target = CHECKPOINTS[name]
            if math.dist((x, y), target) > self.arrival_radius:
                break
            self.closest_route_index += 1

        name = CHECKPOINT_ROUTE[self.closest_route_index]
        return name, CHECKPOINTS[name]

    def log_status(self):
        now = self.get_clock().now()

        freshness = ' '.join(
            [
                self.stale_text('pose', self.robot_pose_time, now),
                self.stale_text('odom', self.odom_time, now),
                self.stale_text('cmd', self.cmd_vel_time, now),
                self.stale_text('scan', self.scan_time, now),
            ]
        )

        if self.robot_pose is None:
            self.get_logger().info(f'DIAG waiting for /robot_pose | {freshness}')
            return

        target_name, target = self.current_target()
        x = float(self.robot_pose.x)
        y = float(self.robot_pose.y)
        theta = float(self.robot_pose.theta)
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading_error = wrap_angle(bearing - theta)

        linear = 0.0
        angular = 0.0
        if self.cmd_vel is not None:
            linear = float(self.cmd_vel.linear.x)
            angular = float(self.cmd_vel.angular.z)

        scan_min = 'n/a' if self.scan_min is None else f'{self.scan_min:.2f}'
        scan_front = 'n/a' if self.scan_front is None else f'{self.scan_front:.2f}'

        message = (
            f'DIAG pose=({x:.2f},{y:.2f},{theta:.2f}) '
            f'target={target_name} d={distance:.2f} heading_err={heading_error:.2f} '
            f'cmd=({linear:.2f},{angular:.2f}) scan_min={scan_min} front={scan_front} '
            f'nav={self.goal_status} | {freshness}'
        )
        print(message, flush=True)
        self.get_logger().info(message)


def main():
    rclpy.init()
    node = NavigationDiagnosticsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
