from __future__ import annotations

import csv
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node


WORLD_CHECKPOINTS = {
    'arena': {
        'A': (-1.49882, 1.84407),
        'B': (1.5267, -0.221987),
        'C': (-0.416565, -1.35783),
        'D': (-2.63149, -0.778393),
    },
    'test_rviz_map': {
        'A': (-1.49882, 1.84407),
        'B': (1.5267, -0.221987),
        'C': (-0.416565, -1.35783),
        'D': (-2.63149, -0.778393),
    },
    'testrvizmap': {
        'A': (-1.49882, 1.84407),
        'B': (1.5267, -0.221987),
        'C': (-0.416565, -1.35783),
        'D': (-2.63149, -0.778393),
    },
    'simple_corridor': {
        'A': (-4.50, 0.00),
        'B': (4.50, 0.00),
    },
    'two_route': {
        'A': (-4.50, 1.00),
        'B': (4.50, 1.00),
    },
}


class CheckpointMetricsNode(Node):
    def __init__(self) -> None:
        super().__init__('checkpoint_metrics_node')

        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('map_id', '')
        self.declare_parameter('pose_topic', '/robot_1/robot_pose')
        self.declare_parameter('arrival_radius_m', 0.40)
        self.declare_parameter('min_distance_delta_m', 0.002)
        self.declare_parameter('max_pose_jump_m', 1.00)
        self.declare_parameter('output_csv', '')
        self.declare_parameter('reset_csv_on_start', True)

        self.robot_id = str(self.get_parameter('robot_id').value).strip() or 'robot_1'
        self.map_id = str(self.get_parameter('map_id').value).strip().lower()
        self.pose_topic = str(self.get_parameter('pose_topic').value).strip()
        self.arrival_radius_m = float(self.get_parameter('arrival_radius_m').value)
        self.min_distance_delta_m = float(self.get_parameter('min_distance_delta_m').value)
        self.max_pose_jump_m = float(self.get_parameter('max_pose_jump_m').value)
        self.reset_csv_on_start = bool(self.get_parameter('reset_csv_on_start').value)

        output_csv = str(self.get_parameter('output_csv').value).strip()
        if not output_csv:
            output_csv = f'/tmp/rmpd_checkpoint_metrics/{self.robot_id}_{self.map_id or "map"}_metrics.csv'
        self.output_csv = Path(output_csv)

        self.checkpoints = WORLD_CHECKPOINTS.get(self.map_id, {})
        self.run_id = str(int(time.time()))
        self.start_time_ns: int | None = None
        self.last_checkpoint_time_ns: int | None = None
        self.last_checkpoint_distance_m = 0.0
        self.last_pose: tuple[float, float] | None = None
        self.total_distance_m = 0.0
        self.current_checkpoint_name: str | None = None
        self.sequence = 0

        self._prepare_csv()
        self.create_subscription(Pose2D, self.pose_topic, self.pose_callback, 10)

        if self.checkpoints:
            checkpoint_names = ', '.join(sorted(self.checkpoints))
            self.get_logger().info(
                f'Checkpoint metrics enabled for {self.robot_id} on {self.map_id}: '
                f'{checkpoint_names}; csv={self.output_csv}'
            )
        else:
            self.get_logger().warning(
                f'Checkpoint metrics has no checkpoint table for map_id="{self.map_id}". '
                'Distance will be tracked, but checkpoint arrivals will not be logged.'
            )

    def _prepare_csv(self) -> None:
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        should_write_header = self.reset_csv_on_start or not self.output_csv.exists()
        mode = 'w' if self.reset_csv_on_start else 'a'
        with self.output_csv.open(mode, newline='', encoding='utf-8') as handle:
            if should_write_header:
                writer = csv.DictWriter(handle, fieldnames=self.csv_fields())
                writer.writeheader()

    @staticmethod
    def csv_fields() -> list[str]:
        return [
            'run_id',
            'sequence',
            'robot_id',
            'map_id',
            'checkpoint',
            'elapsed_sec',
            'segment_sec',
            'total_distance_m',
            'segment_distance_m',
            'checkpoint_error_m',
            'x',
            'y',
        ]

    def pose_callback(self, msg: Pose2D) -> None:
        x = float(msg.x)
        y = float(msg.y)
        if not (math.isfinite(x) and math.isfinite(y)):
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.start_time_ns is None:
            self.start_time_ns = now_ns
            self.last_checkpoint_time_ns = now_ns

        if self.last_pose is not None:
            delta = math.dist(self.last_pose, (x, y))
            if self.min_distance_delta_m <= delta <= self.max_pose_jump_m:
                self.total_distance_m += delta
            elif delta > self.max_pose_jump_m:
                self.get_logger().warning(
                    f'Ignoring pose jump of {delta:.2f} m while tracking checkpoint metrics.'
                )
        self.last_pose = (x, y)

        checkpoint = self._nearest_arrived_checkpoint(x, y)
        if checkpoint is None:
            return

        checkpoint_name, checkpoint_error_m = checkpoint
        if self.current_checkpoint_name is None:
            self.current_checkpoint_name = checkpoint_name
            self.last_checkpoint_time_ns = now_ns
            self.last_checkpoint_distance_m = self.total_distance_m
            self.get_logger().info(
                f'{self.robot_id} starting from checkpoint {checkpoint_name}; '
                'first timed segment starts now.'
            )
            return

        if checkpoint_name == self.current_checkpoint_name:
            return

        self._record_checkpoint(checkpoint_name, checkpoint_error_m, x, y, now_ns)

    def _nearest_arrived_checkpoint(self, x: float, y: float) -> tuple[str, float] | None:
        best_name = None
        best_distance = None
        for name, (checkpoint_x, checkpoint_y) in self.checkpoints.items():
            distance = math.dist((x, y), (float(checkpoint_x), float(checkpoint_y)))
            if distance <= self.arrival_radius_m and (
                best_distance is None or distance < best_distance
            ):
                best_name = name
                best_distance = distance
        if best_name is None or best_distance is None:
            return None
        return best_name, best_distance

    def _record_checkpoint(
        self,
        checkpoint_name: str,
        checkpoint_error_m: float,
        x: float,
        y: float,
        now_ns: int,
    ) -> None:
        if self.start_time_ns is None:
            self.start_time_ns = now_ns
        if self.last_checkpoint_time_ns is None:
            self.last_checkpoint_time_ns = self.start_time_ns

        elapsed_sec = (now_ns - self.start_time_ns) / 1e9
        segment_sec = (now_ns - self.last_checkpoint_time_ns) / 1e9
        segment_distance_m = self.total_distance_m - self.last_checkpoint_distance_m

        self.sequence += 1
        row = {
            'run_id': self.run_id,
            'sequence': self.sequence,
            'robot_id': self.robot_id,
            'map_id': self.map_id,
            'checkpoint': checkpoint_name,
            'elapsed_sec': f'{elapsed_sec:.3f}',
            'segment_sec': f'{segment_sec:.3f}',
            'total_distance_m': f'{self.total_distance_m:.3f}',
            'segment_distance_m': f'{segment_distance_m:.3f}',
            'checkpoint_error_m': f'{checkpoint_error_m:.3f}',
            'x': f'{x:.3f}',
            'y': f'{y:.3f}',
        }
        with self.output_csv.open('a', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=self.csv_fields())
            writer.writerow(row)

        self.get_logger().info(
            f'{self.robot_id} reached checkpoint {checkpoint_name}: '
            f'segment={segment_sec:.2f}s/{segment_distance_m:.2f}m, '
            f'total={elapsed_sec:.2f}s/{self.total_distance_m:.2f}m '
            f'(error={checkpoint_error_m:.2f}m)'
        )

        self.current_checkpoint_name = checkpoint_name
        self.last_checkpoint_time_ns = now_ns
        self.last_checkpoint_distance_m = self.total_distance_m


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CheckpointMetricsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
