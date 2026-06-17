import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MapBuilderNode(Node):
    """Build a growing occupancy grid from LiDAR + pose."""

    def __init__(self) -> None:
        super().__init__('map_builder')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('map_width_m', 20.0)
        self.declare_parameter('map_height_m', 20.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', 0.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.map_width_m = float(self.get_parameter('map_width_m').value)
        self.map_height_m = float(self.get_parameter('map_height_m').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)
        self.laser_yaw = float(self.get_parameter('laser_yaw').value)

        self.width_cells = int(self.map_width_m / self.resolution)
        self.height_cells = int(self.map_height_m / self.resolution)
        self.origin_x = -self.map_width_m / 2.0
        self.origin_y = -self.map_height_m / 2.0

        self.grid = np.full((self.height_cells, self.width_cells), -1, dtype=np.int8)
        self.latest_pose = None

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.pose_sub = self.create_subscription(Pose2D, self.pose_topic, self.pose_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_sensor_tf()

        self.get_logger().info(
            f'Map builder ready: scan={self.scan_topic}, pose={self.pose_topic}, map={self.map_topic}'
        )

    def pose_callback(self, msg) -> None:
        # Accept geometry_msgs/Pose2D without importing it into the subscription type above.
        x = float(msg.x)
        y = float(msg.y)
        theta = normalize_angle(float(msg.theta))
        self.latest_pose = (x, y, theta)
        self.publish_base_tf(x, y, theta)

    def scan_callback(self, scan: LaserScan) -> None:
        if self.latest_pose is None:
            return

        robot_x, robot_y, robot_theta = self.latest_pose
        robot_cell = self.world_to_grid(robot_x, robot_y)
        if robot_cell is None:
            return

        start_col, start_row = robot_cell
        angle = scan.angle_min

        for range_value in scan.ranges:
            if range_value < scan.range_min:
                angle += scan.angle_increment
                continue

            if not math.isfinite(range_value):
                valid_range = scan.range_max
                hit_detected = False
            else:
                valid_range = min(range_value, scan.range_max)
                hit_detected = scan.range_min <= range_value < scan.range_max
            beam_angle = robot_theta + angle
            end_x = robot_x + valid_range * math.cos(beam_angle)
            end_y = robot_y + valid_range * math.sin(beam_angle)
            end_cell = self.world_to_grid(end_x, end_y)

            if end_cell is not None:
                self.mark_free_along_ray(start_col, start_row, end_cell[0], end_cell[1])
                if hit_detected:
                    self.set_cell(end_cell[0], end_cell[1], 100)

            angle += scan.angle_increment

        self.publish_map(scan.header.stamp.sec, scan.header.stamp.nanosec)

    def publish_static_sensor_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.laser_frame
        transform.transform.translation.x = self.laser_x
        transform.transform.translation.y = self.laser_y
        transform.transform.translation.z = 0.0
        q = yaw_to_quaternion(self.laser_yaw)
        transform.transform.rotation = q
        self.static_tf_broadcaster.sendTransform(transform)

    def publish_base_tf(self, x: float, y: float, theta: float) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = yaw_to_quaternion(theta)
        self.tf_broadcaster.sendTransform(transform)

    def publish_map(self, sec: int, nanosec: int) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nanosec)
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.reshape(-1).tolist()
        self.map_pub.publish(msg)

    def world_to_grid(self, x: float, y: float):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if col < 0 or row < 0 or col >= self.width_cells or row >= self.height_cells:
            return None
        return col, row

    def set_cell(self, col: int, row: int, value: int) -> None:
        if 0 <= col < self.width_cells and 0 <= row < self.height_cells:
            self.grid[row, col] = np.int8(value)

    def mark_free_along_ray(self, start_col: int, start_row: int, end_col: int, end_row: int) -> None:
        for col, row in self.bresenham(start_col, start_row, end_col, end_row):
            if col == end_col and row == end_row:
                break
            self.set_cell(col, row, 0)

    def bresenham(self, x0: int, y0: int, x1: int, y1: int):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0

        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy


def main() -> None:
    rclpy.init()
    node = MapBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
