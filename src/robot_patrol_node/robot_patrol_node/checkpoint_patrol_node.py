#!/usr/bin/env python3

from dataclasses import dataclass
import json
import math
import os

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


@dataclass(frozen=True)
class Checkpoint:
    x: float
    y: float


DEFAULT_CHECKPOINTS = {
    'A': Checkpoint(-1.49882, 1.84407),
    'B': Checkpoint(1.5267, -0.221987),
    'C': Checkpoint(-0.416565, -1.35783),
    'D': Checkpoint(-2.63149, -0.778393),
}

HELPER_TARGETS = {}

DEFAULT_ROUTE = ['A', 'B', 'C', 'D', 'A']
WORLD_ROUTES = {
    'simple_corridor': (
        {
            'A': Checkpoint(-4.50, 0.00),
            'B': Checkpoint(4.50, 0.00),
        },
        ['A', 'B'],
    ),
    'two_route': (
        {
            'A': Checkpoint(-4.50, 1.00),
            'B': Checkpoint(4.50, 1.00),
        },
        ['A', 'B'],
    ),
}


class CheckpointPatrolNode(Node):
    def __init__(self):
        super().__init__('checkpoint_patrol_node')

        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_reached_radius', 0.52)
        self.declare_parameter('helper_reached_radius', 0.50)
        self.declare_parameter('helper_stalled_pass_radius', 0.65)
        self.declare_parameter('retry_delay_sec', 1.5)
        self.declare_parameter('stalled_timeout_sec', 12.0)
        self.declare_parameter('feedback_log_interval_sec', 5.0)
        self.declare_parameter('blocked_front_range_m', 0.30)
        self.declare_parameter('blocked_progress_timeout_sec', 4.0)
        self.declare_parameter('retry_recovery_duration_sec', 0.0)
        self.declare_parameter('retry_recovery_linear_x', -0.10)
        self.declare_parameter('retry_recovery_angular_z', 0.75)
        self.declare_parameter('contact_wait_timeout_sec', 3.0)
        self.declare_parameter('contact_wait_radius', 0.16)
        self.declare_parameter('contact_assist_timeout_sec', 2.5)
        self.declare_parameter('contact_assist_linear_x', 0.05)
        self.declare_parameter('contact_assist_angular_z_max', 0.8)
        self.declare_parameter('contact_assist_heading_tolerance', 0.30)
        self.declare_parameter('contact_assist_min_front_clearance', 0.10)
        self.declare_parameter('departure_assist_timeout_sec', 2.5)
        self.declare_parameter('departure_assist_front_clearance', 0.20)
        self.declare_parameter('departure_assist_heading_tolerance', 0.35)
        self.declare_parameter('departure_assist_backoff_linear_x', -0.06)
        self.declare_parameter('departure_assist_turn_speed', 0.8)
        self.declare_parameter('map_id', '')

        self.loop = bool(self.get_parameter('loop').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.goal_reached_radius = float(self.get_parameter('goal_reached_radius').value)
        self.helper_reached_radius = float(self.get_parameter('helper_reached_radius').value)
        self.helper_stalled_pass_radius = float(
            self.get_parameter('helper_stalled_pass_radius').value
        )
        self.retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)
        self.stalled_timeout_sec = float(self.get_parameter('stalled_timeout_sec').value)
        self.feedback_log_interval_sec = float(
            self.get_parameter('feedback_log_interval_sec').value
        )
        self.blocked_front_range_m = float(self.get_parameter('blocked_front_range_m').value)
        self.blocked_progress_timeout_sec = float(
            self.get_parameter('blocked_progress_timeout_sec').value
        )
        self.retry_recovery_duration_sec = float(
            self.get_parameter('retry_recovery_duration_sec').value
        )
        self.retry_recovery_linear_x = float(
            self.get_parameter('retry_recovery_linear_x').value
        )
        self.retry_recovery_angular_z = float(
            self.get_parameter('retry_recovery_angular_z').value
        )
        self.contact_wait_timeout_sec = float(
            self.get_parameter('contact_wait_timeout_sec').value
        )
        self.contact_wait_radius = float(self.get_parameter('contact_wait_radius').value)
        self.contact_assist_timeout_sec = float(
            self.get_parameter('contact_assist_timeout_sec').value
        )
        self.contact_assist_linear_x = float(
            self.get_parameter('contact_assist_linear_x').value
        )
        self.contact_assist_angular_z_max = float(
            self.get_parameter('contact_assist_angular_z_max').value
        )
        self.contact_assist_heading_tolerance = float(
            self.get_parameter('contact_assist_heading_tolerance').value
        )
        self.contact_assist_min_front_clearance = float(
            self.get_parameter('contact_assist_min_front_clearance').value
        )
        self.departure_assist_timeout_sec = float(
            self.get_parameter('departure_assist_timeout_sec').value
        )
        self.departure_assist_front_clearance = float(
            self.get_parameter('departure_assist_front_clearance').value
        )
        self.departure_assist_heading_tolerance = float(
            self.get_parameter('departure_assist_heading_tolerance').value
        )
        self.departure_assist_backoff_linear_x = float(
            self.get_parameter('departure_assist_backoff_linear_x').value
        )
        self.departure_assist_turn_speed = float(
            self.get_parameter('departure_assist_turn_speed').value
        )
        self.map_id = str(self.get_parameter('map_id').value).strip().lower()
        self.checkpoints = dict(DEFAULT_CHECKPOINTS)
        self.route = list(DEFAULT_ROUTE)
        self.visible_route = list(DEFAULT_ROUTE)
        if self.map_id in WORLD_ROUTES:
            world_checkpoints, world_route = WORLD_ROUTES[self.map_id]
            self.checkpoints = dict(world_checkpoints)
            self.route = list(world_route)
            self.visible_route = list(world_route)
            if self.map_id == 'simple_corridor':
                self.loop = True
            if self.map_id == 'two_route':
                self.loop = True

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.robot_pose_subscription = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10,
        )
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )
        self.active_checkpoint_pub = self.create_publisher(String, '/active_checkpoint', 10)
        self.webots_checkpoint_event_pub = self.create_publisher(
            String,
            '/webots_checkpoint_event',
            10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.checkpoint_contact_subscription = self.create_subscription(
            String,
            '/webots_checkpoint_contact',
            self.checkpoint_contact_callback,
            10,
        )

        self.current_robot_pose = None
        self.current_amcl_pose = None
        self.scan_front_min = None
        self.scan_left_min = None
        self.scan_right_min = None
        self.current_index = 0
        self.retry_count = 0
        self.started = False
        self.active_checkpoint_name = None
        self.active_goal_sequence = None
        self.goal_sequence = 0
        self.current_goal_handle = None
        self.retry_timer = None
        self.next_goal_timer = None
        self.departure_timer = None
        self.departure_mode = None
        self.contact_wait_timer = None
        self.contact_assist_timer = None
        self.contact_assist_deadline = None
        self.contact_assist_reason = None
        self.departure_end_time = None
        self.departure_twist = None
        self.departure_followup = None
        self.departure_target_name = None
        self.departure_checkpoint_name = None
        self.last_feedback_log_time = None
        self.last_progress_distance = None
        self.last_progress_time = None
        self.stalled_retry_started = False
        self.active_checkpoint_touched = False

        self.start_timer = self.create_timer(1.0, self.start_when_ready)
        self.arrival_timer = self.create_timer(0.5, self.check_arrival_radius)

        self.get_logger().info(
            'AMCL mode: Webots GPS/IMU odom + LiDAR scan + known static map.'
        )
        self.get_logger().info(
            f'Checkpoint route: {" -> ".join(self.visible_route)}; '
            f'arrival radius={self.goal_reached_radius:.2f} m'
        )
        self.get_logger().info(
            'Visible checkpoints complete only after Webots reports robot '
            'center arrival at the colored block.'
        )

    def robot_pose_callback(self, msg):
        self.current_robot_pose = msg

    def amcl_pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_amcl_pose = pose

    def scan_callback(self, msg):
        self.scan_front_min = self.scan_sector_min(msg, -0.20, 0.20)
        self.scan_left_min = self.scan_sector_min(msg, 0.45, 1.20)
        self.scan_right_min = self.scan_sector_min(msg, -1.20, -0.45)

    @staticmethod
    def scan_sector_min(scan, angle_min, angle_max):
        finite = []
        angle = float(scan.angle_min)
        increment = float(scan.angle_increment)
        for value in scan.ranges:
            if angle_min <= angle <= angle_max and math.isfinite(value):
                finite.append(float(value))
            angle += increment
        return min(finite) if finite else None

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def checkpoint_contact_callback(self, msg):
        if self.active_goal_sequence is None or not self.active_checkpoint_name:
            return
        if self.active_checkpoint_name in HELPER_TARGETS:
            return

        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Ignored malformed checkpoint contact: {msg.data}')
            return

        checkpoint_name = str(event.get('name', '')).strip()
        if checkpoint_name != self.active_checkpoint_name:
            return

        distance = event.get('distance')
        if isinstance(distance, (int, float)):
            reason = (
                'centered on colored block in Webots at '
                f'{float(distance):.2f} m from marker center'
            )
        else:
            reason = 'centered on colored block in Webots'

        self.active_checkpoint_touched = True
        self.complete_active_checkpoint(reason)

    def current_pose_xy(self):
        if self.current_robot_pose is not None:
            return self.current_robot_pose.x, self.current_robot_pose.y, 'GPS'
        if self.current_amcl_pose is not None:
            return (
                self.current_amcl_pose.pose.position.x,
                self.current_amcl_pose.pose.position.y,
                'AMCL',
            )
        return None

    def active_checkpoint(self):
        if self.active_checkpoint_name is None:
            return None
        return self.checkpoints.get(self.active_checkpoint_name)

    def active_reached_radius(self):
        if self.active_checkpoint_name in HELPER_TARGETS:
            return self.helper_reached_radius
        return self.goal_reached_radius

    def distance_to_active_checkpoint(self):
        checkpoint = self.active_checkpoint()
        pose = self.current_pose_xy()
        if checkpoint is None or pose is None:
            return None
        x, y, source = pose
        return math.dist((x, y), (checkpoint.x, checkpoint.y)), source

    def heading_error_to_checkpoint(self, checkpoint_name):
        if self.current_robot_pose is None:
            return None
        checkpoint = self.checkpoints.get(checkpoint_name)
        if checkpoint is None:
            return None

        heading = math.atan2(
            checkpoint.y - self.current_robot_pose.y,
            checkpoint.x - self.current_robot_pose.x,
        )
        return self.wrap_angle(heading - self.current_robot_pose.theta)

    def next_route_checkpoint_name(self):
        next_index = self.current_index
        if next_index >= len(self.route):
            if not self.loop:
                return None
            next_index = self.loop_restart_index()
        if next_index < 0 or next_index >= len(self.route):
            return None
        return self.route[next_index]

    def loop_restart_index(self):
        if len(self.route) <= 1:
            return 0
        if len(self.route) == 2:
            return 0
        if self.route[0] == self.route[-1]:
            return 1
        return 0

    def start_when_ready(self):
        if self.started:
            return

        if self.current_pose_xy() is None:
            self.get_logger().info('Waiting for Webots /robot_pose before starting patrol...')
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 /navigate_to_pose...')
            return

        self.started = True
        self.start_timer.cancel()
        self.get_logger().info('Nav2 ready. Starting checkpoint patrol.')
        self.send_next_checkpoint()

    def send_next_checkpoint(self):
        if self.current_index >= len(self.route):
            if not self.loop:
                self.get_logger().info('Checkpoint route finished.')
                return
            self.current_index = self.loop_restart_index()

        checkpoint_name = self.route[self.current_index]
        self.send_checkpoint(checkpoint_name)

    def send_checkpoint(self, checkpoint_name):
        checkpoint = self.checkpoints.get(checkpoint_name)
        if checkpoint is None:
            self.get_logger().warning(f'Skipping unknown checkpoint {checkpoint_name}.')
            self.current_index += 1
            self.schedule_next_goal(0.1)
            return

        self.cancel_timer('retry')
        self.cancel_timer('next')
        self.cancel_timer('contact_wait')
        self.cancel_timer('contact_assist')

        self.goal_sequence += 1
        sequence = self.goal_sequence
        self.active_goal_sequence = sequence
        self.active_checkpoint_name = checkpoint_name
        self.publish_active_checkpoint(checkpoint_name)
        self.current_goal_handle = None
        self.last_feedback_log_time = None
        self.last_progress_distance = None
        self.last_progress_time = self.get_clock().now()
        self.stalled_retry_started = False
        self.active_checkpoint_touched = False

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = checkpoint.x
        pose.pose.position.y = checkpoint.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        distance = self.distance_to_active_checkpoint()
        distance_text = 'distance pending'
        if distance is not None:
            distance_text = f'{distance[0]:.2f} m from {distance[1]}'

        if (
            self.map_id == 'simple_corridor'
            and checkpoint_name == 'A'
            and distance is not None
            and distance[0] <= self.goal_reached_radius
        ):
            self.get_logger().info(
                'Simple corridor start checkpoint already reached; advancing directly to B.'
            )
            self.complete_active_checkpoint(
                f'already within start radius: {distance[0]:.2f} m using {distance[1]}'
            )
            return

        if checkpoint_name in HELPER_TARGETS:
            target_name = HELPER_TARGETS[checkpoint_name]
            message = (
                f'VIA waypoint for checkpoint {target_name}: '
                f'({checkpoint.x:.2f}, {checkpoint.y:.2f}); {distance_text}'
            )
        else:
            checkpoint_number = sum(
                1 for name in self.route[: self.current_index + 1] if name not in HELPER_TARGETS
            )
            total = len(self.visible_route)
            message = (
                f'NEXT checkpoint {checkpoint_number}/{total}: {checkpoint_name} '
                f'({checkpoint.x:.2f}, {checkpoint.y:.2f}); {distance_text}'
            )
        print(message, flush=True)
        self.get_logger().info(message)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda msg, seq=sequence: self.feedback_callback(msg, seq),
        )
        future.add_done_callback(lambda done, seq=sequence: self.goal_response_callback(done, seq))

    def goal_response_callback(self, future, sequence):
        if sequence != self.active_goal_sequence:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Nav2 goal send failed: {exc}')
            self.retry_active_checkpoint('send failure')
            return

        if not goal_handle.accepted:
            self.get_logger().warning(f'Nav2 rejected checkpoint {self.active_checkpoint_name}.')
            self.retry_active_checkpoint('goal rejected')
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info(f'Nav2 accepted checkpoint {self.active_checkpoint_name}.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done, seq=sequence: self.result_callback(done, seq))

    def feedback_callback(self, feedback_msg, sequence):
        if sequence != self.active_goal_sequence:
            return

        now = self.get_clock().now()
        distance = self.distance_to_active_checkpoint()
        nav2_remaining = float(feedback_msg.feedback.distance_remaining)
        progress_distance = distance[0] if distance is not None else nav2_remaining

        if (
            self.last_progress_distance is None
            or progress_distance < self.last_progress_distance - 0.15
        ):
            self.last_progress_distance = progress_distance
            self.last_progress_time = now
            self.stalled_retry_started = False
        elif self.last_progress_time is not None and not self.stalled_retry_started:
            stalled_for = (now - self.last_progress_time).nanoseconds / 1_000_000_000.0
            if (
                self.scan_front_min is not None
                and self.scan_front_min <= self.blocked_front_range_m
                and stalled_for >= self.blocked_progress_timeout_sec
            ):
                self.stalled_retry_started = True
                self.retry_active_checkpoint(
                    f'blocked ahead at {self.scan_front_min:.2f} m for {stalled_for:.0f}s'
                )
                return
            if stalled_for >= self.stalled_timeout_sec:
                self.stalled_retry_started = True
                self.retry_active_checkpoint(f'no progress for {stalled_for:.0f}s')
                return

        should_log = self.last_feedback_log_time is None
        if self.last_feedback_log_time is not None:
            elapsed = (now - self.last_feedback_log_time).nanoseconds / 1_000_000_000.0
            should_log = elapsed >= self.feedback_log_interval_sec

        if should_log:
            self.last_feedback_log_time = now
            if distance is None:
                self.get_logger().info(
                    f'Checkpoint {self.active_checkpoint_name}: '
                    f'Nav2 reports {nav2_remaining:.2f} m remaining'
                )
            else:
                self.get_logger().info(
                    f'Checkpoint {self.active_checkpoint_name}: '
                    f'{distance[0]:.2f} m from marker ({distance[1]})'
                )

    def check_arrival_radius(self):
        if self.active_goal_sequence is None:
            return
        if self.active_checkpoint_name not in HELPER_TARGETS:
            return

        distance = self.distance_to_active_checkpoint()
        if distance is None:
            return

        if distance[0] <= self.active_reached_radius():
            self.complete_active_checkpoint(
                f'within {distance[0]:.2f} m using {distance[1]}'
            )

    def result_callback(self, future, sequence):
        if sequence != self.active_goal_sequence:
            return

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Nav2 result failed: {exc}')
            self.retry_active_checkpoint('result failure')
            return

        distance = self.distance_to_active_checkpoint()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if (
                self.active_checkpoint_name in HELPER_TARGETS
                and distance is not None
                and distance[0] <= self.active_reached_radius()
            ):
                self.complete_active_checkpoint(
                    f'Nav2 succeeded at {distance[0]:.2f} m using {distance[1]}'
                )
                return

            if self.active_checkpoint_name not in HELPER_TARGETS:
                if distance is None:
                    self.retry_active_checkpoint(
                        'Nav2 succeeded before marker pose/contact was available'
                    )
                elif distance[0] <= self.contact_wait_radius:
                    self.start_contact_assist(
                        sequence,
                        f'Nav2 succeeded but Webots has not reported colored-block center arrival; '
                        f'marker is {distance[0]:.2f} m away using {distance[1]}'
                    )
                else:
                    self.retry_active_checkpoint(
                        f'Nav2 succeeded but Webots has not reported colored-block center arrival; '
                        f'marker is {distance[0]:.2f} m away using {distance[1]}'
                    )
                return

            if distance is None:
                self.retry_active_checkpoint('Nav2 succeeded before helper pose was available')
            else:
                self.retry_active_checkpoint(
                    f'Nav2 succeeded but helper is still {distance[0]:.2f} m '
                    f'away using {distance[1]}'
                )
            return

        if (
            self.active_checkpoint_name in HELPER_TARGETS
            and distance is not None
            and distance[0] <= self.active_reached_radius()
        ):
            self.complete_active_checkpoint(
                f'Nav2 status {result.status}, but helper reached at {distance[0]:.2f} m'
            )
            return

        self.retry_active_checkpoint(f'Nav2 status {result.status}')

    def start_contact_assist(self, sequence, reason):
        self.cancel_timer('contact_wait')
        self.cancel_timer('contact_assist')

        if self.current_robot_pose is None:
            self.wait_for_checkpoint_contact(sequence, reason)
            return

        self.contact_assist_reason = reason
        self.contact_assist_deadline = (
            self.get_clock().now() + Duration(seconds=self.contact_assist_timeout_sec)
        )
        self.get_logger().info(
            f'Starting final centering assist for checkpoint {self.active_checkpoint_name}: '
            f'{reason}'
        )
        self.contact_assist_timer = self.create_timer(
            0.1,
            lambda seq=sequence: self.contact_assist_tick(seq),
        )

    def contact_assist_tick(self, sequence):
        if sequence != self.active_goal_sequence:
            self.cancel_timer('contact_assist')
            return

        if self.contact_assist_deadline is None or self.active_checkpoint_name is None:
            self.cancel_timer('contact_assist')
            return

        if self.current_robot_pose is None:
            self.cancel_timer('contact_assist')
            self.retry_active_checkpoint('final centering assist lost robot pose')
            return

        now = self.get_clock().now()
        if now >= self.contact_assist_deadline:
            self.cmd_vel_pub.publish(Twist())
            reason = self.contact_assist_reason or 'final centering assist timed out'
            self.cancel_timer('contact_assist')
            self.wait_for_checkpoint_contact(sequence, reason)
            return

        distance = self.distance_to_active_checkpoint()
        checkpoint = self.active_checkpoint()
        if distance is None or checkpoint is None:
            self.cmd_vel_pub.publish(Twist())
            self.cancel_timer('contact_assist')
            self.retry_active_checkpoint('final centering assist lost checkpoint distance')
            return

        dx = checkpoint.x - self.current_robot_pose.x
        dy = checkpoint.y - self.current_robot_pose.y
        heading = math.atan2(dy, dx)
        heading_error = self.wrap_angle(heading - self.current_robot_pose.theta)

        if (
            self.scan_front_min is not None
            and self.scan_front_min <= self.contact_assist_min_front_clearance
            and distance[0] > self.contact_wait_radius / 2.0
        ):
            self.cmd_vel_pub.publish(Twist())
            self.cancel_timer('contact_assist')
            self.retry_active_checkpoint(
                f'final centering assist blocked ahead at {self.scan_front_min:.2f} m'
            )
            return

        twist = Twist()
        angular_limit = abs(self.contact_assist_angular_z_max)
        if abs(heading_error) > self.contact_assist_heading_tolerance:
            twist.angular.z = max(
                -angular_limit,
                min(angular_limit, 1.5 * heading_error),
            )
        else:
            twist.linear.x = min(
                self.contact_assist_linear_x,
                max(0.025, distance[0] * 0.8),
            )
            twist.angular.z = max(
                -angular_limit,
                min(angular_limit, heading_error),
            )

        self.cmd_vel_pub.publish(twist)

    def wait_for_checkpoint_contact(self, sequence, reason):
        self.cancel_timer('contact_wait')
        self.get_logger().info(
            f'Waiting up to {self.contact_wait_timeout_sec:.1f}s for Webots center arrival: '
            f'{reason}'
        )
        self.contact_wait_timer = self.create_timer(
            self.contact_wait_timeout_sec,
            lambda seq=sequence, why=reason: self.contact_wait_timeout(seq, why),
        )

    def contact_wait_timeout(self, sequence, reason):
        self.cancel_timer('contact_wait')
        if sequence != self.active_goal_sequence:
            return
        self.retry_active_checkpoint(reason)

    def complete_active_checkpoint(self, reason):
        if self.active_goal_sequence is None:
            return

        self.cancel_timer('contact_wait')
        self.cancel_timer('contact_assist')
        checkpoint_name = self.active_checkpoint_name
        if checkpoint_name in HELPER_TARGETS:
            target_name = HELPER_TARGETS[checkpoint_name]
            message = (
                f'PASSED waypoint for checkpoint {target_name}: '
                f'{checkpoint_name} ({reason})'
            )
        else:
            checkpoint_number = sum(
                1 for name in self.route[: self.current_index + 1] if name not in HELPER_TARGETS
            )
            total = len(self.visible_route)
            message = (
                f'REACHED checkpoint {checkpoint_number}/{total}: '
                f'{checkpoint_name} ({reason})'
            )
        print(message, flush=True)
        self.get_logger().info(message)
        self.publish_webots_checkpoint_event(message)

        self.cmd_vel_pub.publish(Twist())
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

        self.active_goal_sequence = None
        self.active_checkpoint_name = None
        self.publish_active_checkpoint('')
        self.current_goal_handle = None
        self.retry_count = 0
        self.current_index += 1

        next_checkpoint_name = self.next_route_checkpoint_name()
        if (
            checkpoint_name not in HELPER_TARGETS
            and next_checkpoint_name is not None
            and self.map_id != 'simple_corridor'
        ):
            self.start_departure_assist(checkpoint_name, next_checkpoint_name)
        else:
            self.schedule_next_goal(0.75)

    def retry_active_checkpoint(self, reason):
        if self.active_goal_sequence is None:
            return

        self.cancel_timer('contact_wait')
        self.cancel_timer('contact_assist')
        checkpoint_name = self.active_checkpoint_name
        distance = self.distance_to_active_checkpoint()
        if (
            checkpoint_name in HELPER_TARGETS
            and distance is not None
            and distance[0] <= self.helper_stalled_pass_radius
        ):
            self.complete_active_checkpoint(
                f'helper close enough after {reason}: {distance[0]:.2f} m using {distance[1]}'
            )
            return

        self.retry_count += 1
        self.get_logger().warning(
            f'Replanning checkpoint {checkpoint_name}: {reason}; retry {self.retry_count}'
        )

        self.cmd_vel_pub.publish(Twist())
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

        self.active_goal_sequence = None
        self.current_goal_handle = None
        self.publish_active_checkpoint('')
        if self.should_run_retry_recovery():
            self.start_retry_recovery(checkpoint_name, reason)
        else:
            self.schedule_retry()

    def start_departure_assist(self, checkpoint_name, next_checkpoint_name):
        self.cancel_timer('departure')
        self.departure_mode = 'assist'
        self.departure_checkpoint_name = checkpoint_name
        self.departure_target_name = next_checkpoint_name
        self.departure_end_time = (
            self.get_clock().now() + Duration(seconds=self.departure_assist_timeout_sec)
        )
        self.departure_followup = 'next'
        self.get_logger().info(
            f'Departing checkpoint {checkpoint_name}: orienting toward {next_checkpoint_name} '
            f'using live pose + LiDAR'
        )
        self.departure_timer = self.create_timer(0.1, self.departure_recovery_tick)

    def should_run_retry_recovery(self):
        if self.map_id == 'simple_corridor':
            return False
        if self.retry_recovery_duration_sec <= 0.0:
            return False
        if self.scan_front_min is None:
            return False
        return self.scan_front_min <= self.blocked_front_range_m

    def start_retry_recovery(self, checkpoint_name, reason):
        self.cancel_timer('departure')
        self.departure_mode = 'open_loop'
        twist = Twist()
        twist.linear.x = self.retry_recovery_linear_x
        turn_direction = self.choose_recovery_turn_direction(checkpoint_name)
        twist.angular.z = turn_direction * abs(self.retry_recovery_angular_z)
        self.departure_twist = twist
        self.departure_end_time = (
            self.get_clock().now() + Duration(seconds=self.retry_recovery_duration_sec)
        )
        self.departure_followup = 'retry'
        self.get_logger().info(
            f'Unsticking before retrying {checkpoint_name}: '
            f'linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f} '
            f'for {self.retry_recovery_duration_sec:.1f}s after {reason}'
        )
        self.departure_timer = self.create_timer(0.1, self.departure_recovery_tick)

    def choose_recovery_turn_direction(self, checkpoint_name):
        heading_error = self.heading_error_to_checkpoint(checkpoint_name)
        if self.scan_left_min is None and self.scan_right_min is None:
            if heading_error is None:
                return 1.0
            return -1.0 if heading_error < 0.0 else 1.0
        if self.scan_left_min is None:
            return 1.0
        if self.scan_right_min is None:
            return -1.0
        return -1.0 if self.scan_left_min >= self.scan_right_min else 1.0

    def departure_assist_tick(self):
        if self.departure_end_time is None or self.departure_target_name is None:
            self.cancel_timer('departure')
            self.schedule_next_goal(0.2)
            return

        if self.get_clock().now() >= self.departure_end_time:
            self.get_logger().info(
                f'Departure assist timed out after checkpoint '
                f'{self.departure_checkpoint_name}; handing off to Nav2'
            )
            self.cmd_vel_pub.publish(Twist())
            self.cancel_timer('departure')
            self.schedule_next_goal(0.2)
            return

        if self.current_robot_pose is None:
            self.cmd_vel_pub.publish(Twist())
            self.cancel_timer('departure')
            self.schedule_next_goal(0.2)
            return

        heading_error = self.heading_error_to_checkpoint(self.departure_target_name)
        if heading_error is None:
            self.cmd_vel_pub.publish(Twist())
            self.cancel_timer('departure')
            self.schedule_next_goal(0.2)
            return

        front_clearance = (
            float('inf') if self.scan_front_min is None else float(self.scan_front_min)
        )

        twist = Twist()
        if front_clearance <= self.departure_assist_front_clearance:
            if self.scan_left_min == self.scan_right_min:
                turn_direction = -1.0 if heading_error < 0.0 else 1.0
            elif self.scan_left_min is None:
                turn_direction = 1.0
            elif self.scan_right_min is None:
                turn_direction = -1.0
            else:
                turn_direction = -1.0 if self.scan_left_min >= self.scan_right_min else 1.0

            twist.linear.x = self.departure_assist_backoff_linear_x
            twist.angular.z = turn_direction * abs(self.departure_assist_turn_speed)
            self.cmd_vel_pub.publish(twist)
            return

        if abs(heading_error) > self.departure_assist_heading_tolerance:
            twist.angular.z = max(
                -abs(self.departure_assist_turn_speed),
                min(abs(self.departure_assist_turn_speed), 1.5 * heading_error),
            )
            self.cmd_vel_pub.publish(twist)
            return

        self.get_logger().info(
            f'Departure assist cleared checkpoint {self.departure_checkpoint_name} '
            f'toward {self.departure_target_name}; handing off to Nav2'
        )
        self.cmd_vel_pub.publish(Twist())
        self.cancel_timer('departure')
        self.schedule_next_goal(0.2)

    def departure_recovery_tick(self):
        if self.departure_mode == 'assist':
            self.departure_assist_tick()
            return

        if self.departure_end_time is None or self.departure_twist is None:
            self.cancel_timer('departure')
            return

        if self.get_clock().now() >= self.departure_end_time:
            self.cmd_vel_pub.publish(Twist())
            followup = self.departure_followup
            self.departure_end_time = None
            self.departure_twist = None
            self.departure_followup = None
            self.cancel_timer('departure')
            if followup == 'retry':
                self.schedule_retry()
            else:
                self.schedule_next_goal(0.2)
            return

        self.cmd_vel_pub.publish(self.departure_twist)

    def publish_active_checkpoint(self, checkpoint_name):
        msg = String()
        msg.data = checkpoint_name
        self.active_checkpoint_pub.publish(msg)

    def publish_webots_checkpoint_event(self, message):
        msg = String()
        msg.data = message
        self.webots_checkpoint_event_pub.publish(msg)

    def schedule_retry(self):
        self.cancel_timer('retry')
        self.retry_timer = self.create_timer(self.retry_delay_sec, self.retry_from_timer)

    def retry_from_timer(self):
        self.cancel_timer('retry')
        if self.current_index < len(self.route):
            self.send_checkpoint(self.route[self.current_index])

    def schedule_next_goal(self, delay):
        self.cancel_timer('next')
        self.next_goal_timer = self.create_timer(delay, self.next_goal_from_timer)

    def next_goal_from_timer(self):
        self.cancel_timer('next')
        self.send_next_checkpoint()

    def cancel_timer(self, timer_name):
        if timer_name == 'retry' and self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        if timer_name == 'next' and self.next_goal_timer is not None:
            self.next_goal_timer.cancel()
            self.destroy_timer(self.next_goal_timer)
            self.next_goal_timer = None
        if timer_name == 'contact_wait' and self.contact_wait_timer is not None:
            self.contact_wait_timer.cancel()
            self.destroy_timer(self.contact_wait_timer)
            self.contact_wait_timer = None
        if timer_name == 'contact_assist':
            if self.contact_assist_timer is not None:
                self.contact_assist_timer.cancel()
                self.destroy_timer(self.contact_assist_timer)
                self.contact_assist_timer = None
            self.contact_assist_deadline = None
            self.contact_assist_reason = None
        if timer_name == 'departure':
            if self.departure_timer is not None:
                self.departure_timer.cancel()
                self.destroy_timer(self.departure_timer)
                self.departure_timer = None
            self.departure_mode = None
            self.departure_end_time = None
            self.departure_twist = None
            self.departure_followup = None
            self.departure_target_name = None
            self.departure_checkpoint_name = None


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointPatrolNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
