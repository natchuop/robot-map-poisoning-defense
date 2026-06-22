#!/usr/bin/env python3

from dataclasses import dataclass
import json
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Checkpoint:
    x: float
    y: float


CHECKPOINTS = {
    'A': Checkpoint(-1.49882, 1.84407),
    'B': Checkpoint(1.5267, -0.221987),
    'C': Checkpoint(-0.416565, -1.35783),
    'D': Checkpoint(-2.63149, -0.778393),
    '_B_EXIT': Checkpoint(-1.05, 2.20),
    '_B_CRUISE': Checkpoint(0.65, 1.75),
    '_B_APPROACH': Checkpoint(1.55, 0.65),
    '_D_APPROACH': Checkpoint(-1.65, -1.25),
    '_A_RETURN_0': Checkpoint(-2.20, -0.20),
    '_A_RETURN_1': Checkpoint(-1.45, 0.15),
    '_A_RETURN_2': Checkpoint(-0.85, 0.55),
    '_A_RETURN_3': Checkpoint(-0.90, 1.20),
}

HELPER_TARGETS = {
    '_B_EXIT': 'B',
    '_B_CRUISE': 'B',
    '_B_APPROACH': 'B',
    '_D_APPROACH': 'D',
    '_A_RETURN_0': 'A',
    '_A_RETURN_1': 'A',
    '_A_RETURN_2': 'A',
    '_A_RETURN_3': 'A',
}

CHECKPOINT_ROUTE = [
    'A',
    '_B_EXIT',
    '_B_CRUISE',
    '_B_APPROACH',
    'B',
    'C',
    '_D_APPROACH',
    'D',
    '_A_RETURN_0',
    '_A_RETURN_1',
    '_A_RETURN_2',
    '_A_RETURN_3',
    'A',
]
VISIBLE_ROUTE = ['A', 'B', 'C', 'D', 'A']

DEPARTURE_RECOVERIES = {
    'A': (3.0, -0.10, -0.55),
}


class CheckpointPatrolNode(Node):
    def __init__(self):
        super().__init__('checkpoint_patrol_node')

        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_reached_radius', 0.52)
        self.declare_parameter('helper_reached_radius', 0.75)
        self.declare_parameter('helper_stalled_pass_radius', 0.95)
        self.declare_parameter('retry_delay_sec', 1.5)
        self.declare_parameter('stalled_timeout_sec', 20.0)
        self.declare_parameter('feedback_log_interval_sec', 5.0)

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
        self.route = list(CHECKPOINT_ROUTE)

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
        self.departure_end_time = None
        self.departure_twist = None
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
            f'Checkpoint route: {" -> ".join(VISIBLE_ROUTE)}; '
            f'arrival radius={self.goal_reached_radius:.2f} m'
        )
        self.get_logger().info(
            'Visible checkpoints complete only after Webots reports robot '
            'footprint contact with the colored block.'
        )

    def robot_pose_callback(self, msg):
        self.current_robot_pose = msg

    def amcl_pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_amcl_pose = pose

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
                'touching colored block in Webots at '
                f'{float(distance):.2f} m from marker center'
            )
        else:
            reason = 'touching colored block in Webots'

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
        return CHECKPOINTS.get(self.active_checkpoint_name)

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
            self.current_index = 1

        checkpoint_name = self.route[self.current_index]
        self.send_checkpoint(checkpoint_name)

    def send_checkpoint(self, checkpoint_name):
        checkpoint = CHECKPOINTS.get(checkpoint_name)
        if checkpoint is None:
            self.get_logger().warning(f'Skipping unknown checkpoint {checkpoint_name}.')
            self.current_index += 1
            self.schedule_next_goal(0.1)
            return

        self.cancel_timer('retry')
        self.cancel_timer('next')

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
            total = len(VISIBLE_ROUTE)
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
                else:
                    self.retry_active_checkpoint(
                        f'Nav2 succeeded but Webots has not reported colored-block contact; '
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

    def complete_active_checkpoint(self, reason):
        if self.active_goal_sequence is None:
            return

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
            total = len(VISIBLE_ROUTE)
            message = (
                f'REACHED checkpoint {checkpoint_number}/{total}: '
                f'{checkpoint_name} ({reason})'
            )
        print(message, flush=True)
        self.get_logger().info(message)
        self.publish_webots_checkpoint_event(message)

        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

        self.active_goal_sequence = None
        self.active_checkpoint_name = None
        self.publish_active_checkpoint('')
        self.current_goal_handle = None
        self.retry_count = 0
        self.current_index += 1

        if checkpoint_name in DEPARTURE_RECOVERIES:
            self.start_departure_recovery(checkpoint_name)
        else:
            self.schedule_next_goal(0.25)

    def retry_active_checkpoint(self, reason):
        if self.active_goal_sequence is None:
            return

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

        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

        self.active_goal_sequence = None
        self.current_goal_handle = None
        self.publish_active_checkpoint('')
        self.schedule_retry()

    def start_departure_recovery(self, checkpoint_name):
        self.cancel_timer('departure')
        duration, linear_x, angular_z = DEPARTURE_RECOVERIES[checkpoint_name]
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.departure_twist = twist
        self.departure_end_time = self.get_clock().now() + Duration(seconds=duration)
        self.get_logger().info(
            f'Departing checkpoint {checkpoint_name}: recovery cmd_vel '
            f'linear={linear_x:.2f}, angular={angular_z:.2f} for {duration:.1f}s'
        )
        self.departure_timer = self.create_timer(0.1, self.departure_recovery_tick)

    def departure_recovery_tick(self):
        if self.departure_end_time is None or self.departure_twist is None:
            self.cancel_timer('departure')
            return

        if self.get_clock().now() >= self.departure_end_time:
            self.cmd_vel_pub.publish(Twist())
            self.departure_end_time = None
            self.departure_twist = None
            self.cancel_timer('departure')
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
        if timer_name == 'departure':
            if self.departure_timer is not None:
                self.departure_timer.cancel()
                self.destroy_timer(self.departure_timer)
                self.departure_timer = None
            self.departure_end_time = None
            self.departure_twist = None


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
