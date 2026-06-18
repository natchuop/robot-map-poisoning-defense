#!/usr/bin/env python3

from dataclasses import dataclass
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


@dataclass
class Checkpoint:
    x: float
    y: float


CHECKPOINTS = {
    'A': Checkpoint(-1.49882, 1.84407),
    'B': Checkpoint(1.5267, -0.221987),
    'C': Checkpoint(-0.416565, -1.35783),
    'D': Checkpoint(-2.63149, -0.778393),
}

CHECKPOINT_ROUTE = ['A', 'B', 'C', 'D', 'A']


class CheckpointPatrolNode(Node):
    def __init__(self):
        super().__init__('checkpoint_patrol_node')

        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('max_retries_per_checkpoint', 3)
        self.declare_parameter('retry_delay_sec', 2.0)
        self.declare_parameter('print_ascii_path', True)
        self.declare_parameter('ascii_grid_width', 41)
        self.declare_parameter('ascii_grid_height', 21)

        self.loop = bool(self.get_parameter('loop').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.max_retries_per_checkpoint = int(
            self.get_parameter('max_retries_per_checkpoint').value
        )
        self.retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)
        self.print_ascii_path = bool(self.get_parameter('print_ascii_path').value)
        self.ascii_grid_width = int(self.get_parameter('ascii_grid_width').value)
        self.ascii_grid_height = int(self.get_parameter('ascii_grid_height').value)
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

        self.current_robot_pose = None
        self.current_amcl_pose = None
        self.current_index = 0
        self.retry_count = 0
        self.started = False
        self.active_checkpoint_name = None
        self.retry_timer = None
        self.last_feedback_log_distance = None
        self.last_progress_distance = None
        self.last_progress_stamp = None
        self.stalled_warning_issued = False

        self.get_logger().info('Checkpoint route: ' + ' -> '.join(self.route) + f' (loop={self.loop})')
        self.get_logger().info('Using /robot_pose GPS stream for debugging and ASCII previews.')
        self.timer = self.create_timer(1.0, self.start_when_ready)

    def robot_pose_callback(self, msg):
        self.current_robot_pose = msg

    def amcl_pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_amcl_pose = pose

    def pose_xy(self, pose_like):
        if isinstance(pose_like, Pose2D):
            return pose_like.x, pose_like.y
        return pose_like.pose.position.x, pose_like.pose.position.y

    def current_pose_source(self):
        if self.current_robot_pose is not None:
            return 'GPS'
        if self.current_amcl_pose is not None:
            return 'AMCL'
        return None

    def current_pose_xy(self):
        if self.current_robot_pose is not None:
            return self.current_robot_pose.x, self.current_robot_pose.y
        if self.current_amcl_pose is not None:
            return self.pose_xy(self.current_amcl_pose)
        return None

    def start_when_ready(self):
        if self.started:
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for /navigate_to_pose action server...')
            return

        self.started = True
        self.get_logger().info('Nav2 is ready. Starting checkpoint patrol.')
        self.send_next_checkpoint()

    def send_next_checkpoint(self):
        if self.current_index >= len(self.route):
            if self.loop:
                self.current_index = 0
            else:
                self.get_logger().info('Finished checkpoint route.')
                return

        checkpoint_name = self.route[self.current_index]
        self.send_checkpoint(checkpoint_name)

    def send_checkpoint(self, checkpoint_name):
        checkpoint = CHECKPOINTS.get(checkpoint_name)

        if checkpoint is None:
            self.get_logger().warning(
                f'Skipping unknown checkpoint "{checkpoint_name}" in route.'
            )
            self.current_index += 1
            self.retry_count = 0
            self.send_next_checkpoint()
            return

        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None

        self.active_checkpoint_name = checkpoint_name
        self.last_feedback_log_distance = None
        self.last_progress_distance = None
        self.last_progress_stamp = None
        self.stalled_warning_issued = False

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = checkpoint.x
        pose.pose.position.y = checkpoint.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        checkpoint_number = self.current_index + 1
        total_checkpoints = len(self.route)
        pose_source = self.current_pose_source() or 'unknown'
        self.get_logger().info(
            f'TARGET {checkpoint_number}/{total_checkpoints} {checkpoint_name}: '
            f'x={checkpoint.x}, y={checkpoint.y} (position only)'
        )
        self.get_logger().info(
            f'Current position source: {pose_source}'
        )

        if self.print_ascii_path:
            self.print_ascii_preview(checkpoint_name, pose)

        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)

    def print_ascii_preview(self, checkpoint_name, goal_pose):
        current_xy = self.current_pose_xy()
        if current_xy is None:
            self.get_logger().info('ASCII preview unavailable yet because /robot_pose has not arrived.')
            return

        start_x, start_y = current_xy
        goal_x, goal_y = self.pose_xy(goal_pose)

        points = [(start_x, start_y), (goal_x, goal_y)]
        for name, checkpoint in CHECKPOINTS.items():
            points.append((checkpoint.x, checkpoint.y))

        xs = [x for x, _ in points]
        ys = [y for _, y in points]
        margin = 0.5
        min_x = min(xs) - margin
        max_x = max(xs) + margin
        min_y = min(ys) - margin
        max_y = max(ys) + margin

        width = max(11, self.ascii_grid_width)
        height = max(7, self.ascii_grid_height)
        grid = [['.' for _ in range(width)] for _ in range(height)]

        def to_grid(x, y):
            if max_x == min_x or max_y == min_y:
                return 0, 0
            gx = int(round((x - min_x) / (max_x - min_x) * (width - 1)))
            gy = int(round((y - min_y) / (max_y - min_y) * (height - 1)))
            gx = max(0, min(width - 1, gx))
            gy = max(0, min(height - 1, gy))
            return gx, gy

        def mark(x, y, symbol):
            gx, gy = to_grid(x, y)
            grid[height - 1 - gy][gx] = symbol

        def draw_line(x0, y0, x1, y1):
            gx0, gy0 = to_grid(x0, y0)
            gx1, gy1 = to_grid(x1, y1)
            dx = abs(gx1 - gx0)
            dy = -abs(gy1 - gy0)
            sx = 1 if gx0 < gx1 else -1
            sy = 1 if gy0 < gy1 else -1
            err = dx + dy
            x = gx0
            y = gy0
            while True:
                if 0 <= x < width and 0 <= y < height:
                    row = height - 1 - y
                    if grid[row][x] == '.':
                        grid[row][x] = '*'
                if x == gx1 and y == gy1:
                    break
                e2 = 2 * err
                if e2 >= dy:
                    err += dy
                    x += sx
                if e2 <= dx:
                    err += dx
                    y += sy

        draw_line(start_x, start_y, goal_x, goal_y)

        for name, checkpoint in CHECKPOINTS.items():
            mark(checkpoint.x, checkpoint.y, name)

        mark(start_x, start_y, 'R')
        mark(goal_x, goal_y, 'G')

        self.get_logger().info(
            f'ASCII planning preview for checkpoint {checkpoint_name} '
            f'(R=robot GPS, G=goal, *=route sketch, A-D=checkpoints)'
        )
        self.get_logger().info(
            f'Bounds x:[{min_x:.2f}, {max_x:.2f}] y:[{min_y:.2f}, {max_y:.2f}] '
            f'robot=({start_x:.2f}, {start_y:.2f}) goal=({goal_x:.2f}, {goal_y:.2f})'
        )
        for row in grid:
            self.get_logger().info(''.join(row))

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning('Checkpoint goal was rejected by Nav2.')
            self.handle_navigation_failure('goal rejection')
            return

        self.get_logger().info('Checkpoint goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_time = self.get_clock().now()

        if self.last_progress_distance is None or feedback.distance_remaining < self.last_progress_distance - 0.2:
            self.last_progress_distance = feedback.distance_remaining
            self.last_progress_stamp = current_time
            self.stalled_warning_issued = False
        elif (
            self.last_progress_stamp is not None
            and not self.stalled_warning_issued
            and (current_time - self.last_progress_stamp).nanoseconds > 15 * 1_000_000_000
        ):
            checkpoint_name = self.active_checkpoint_name or self.route[self.current_index]
            self.stalled_warning_issued = True
            self.get_logger().warning(
                f'Checkpoint {checkpoint_name} looks stalled: '
                f'distance remaining has not improved for 15 seconds.'
            )

        if (
            self.last_feedback_log_distance is None
            or abs(feedback.distance_remaining - self.last_feedback_log_distance) >= 0.5
            or feedback.distance_remaining <= 0.5
        ):
            self.last_feedback_log_distance = feedback.distance_remaining
            checkpoint_name = self.active_checkpoint_name or self.route[self.current_index]
            self.get_logger().info(
                f'EN ROUTE to {checkpoint_name}: {feedback.distance_remaining:.2f} m remaining'
            )

    def schedule_retry(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)

        self.retry_timer = self.create_timer(self.retry_delay_sec, self.retry_current_checkpoint)

    def retry_current_checkpoint(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None

        checkpoint_name = self.active_checkpoint_name or self.route[self.current_index]
        self.get_logger().info(
            f'Retrying checkpoint {self.current_index + 1}/{len(self.route)}: '
            f'{checkpoint_name} (attempt {self.retry_count}/{self.max_retries_per_checkpoint})'
        )
        self.send_checkpoint(checkpoint_name)

    def handle_navigation_failure(self, reason):
        checkpoint_name = self.active_checkpoint_name or self.route[self.current_index]
        checkpoint_number = self.current_index + 1
        total_checkpoints = len(self.route)

        if self.retry_count < self.max_retries_per_checkpoint:
            self.retry_count += 1
            self.get_logger().warning(
                f'Checkpoint {checkpoint_number}/{total_checkpoints}: {checkpoint_name} '
                f'{reason}. Scheduling retry {self.retry_count}/{self.max_retries_per_checkpoint}.'
            )
            self.schedule_retry()
            return

        self.get_logger().warning(
            f'Checkpoint {checkpoint_number}/{total_checkpoints}: {checkpoint_name} '
            f'{reason}. Moving on after {self.max_retries_per_checkpoint} retries.'
        )
        self.current_index += 1
        self.retry_count = 0
        self.send_next_checkpoint()

    def result_callback(self, future):
        result = future.result()
        checkpoint_name = self.active_checkpoint_name or self.route[self.current_index]
        checkpoint_number = self.current_index + 1
        total_checkpoints = len(self.route)

        self.last_feedback_log_distance = None
        self.last_progress_distance = None
        self.last_progress_stamp = None
        self.stalled_warning_issued = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            message = (
                f'ARRIVED at checkpoint {checkpoint_number}/{total_checkpoints}: '
                f'{checkpoint_name}'
            )
            print(message, flush=True)
            self.get_logger().info(message)
            self.current_index += 1
            self.retry_count = 0
            self.send_next_checkpoint()
            return

        self.handle_navigation_failure(f'finished with status {result.status}')


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
