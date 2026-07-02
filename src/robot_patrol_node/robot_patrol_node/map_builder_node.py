import math

from geometry_msgs.msg import Pose2D, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter('confidence_map_topic', '')
        self.declare_parameter('current_observation_map_topic', '')
        self.declare_parameter('publish_current_observation_map', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('map_width_m', 20.0)
        self.declare_parameter('map_height_m', 20.0)
        self.declare_parameter('map_origin_x', float('nan'))
        self.declare_parameter('map_origin_y', float('nan'))
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('laser_x', 0.0)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('occupancy_mode', 'scored')
        self.declare_parameter('require_pose_update', True)
        self.declare_parameter('hit_score_increment', 4)
        self.declare_parameter('free_score_decrement', 1)
        self.declare_parameter('occupied_score_threshold', 6)
        self.declare_parameter('free_score_threshold', -2)
        self.declare_parameter('score_min', -12)
        self.declare_parameter('score_max', 24)
        self.declare_parameter('clear_on_max_range', False)
        self.declare_parameter('ray_end_trim_m', 0.08)
        self.declare_parameter('occupied_radius_cells', 0)
        self.declare_parameter('auto_expand_map', True)
        self.declare_parameter('expansion_padding_m', 1.0)
        self.declare_parameter('max_map_width_m', 120.0)
        self.declare_parameter('max_map_height_m', 120.0)
        self.declare_parameter('max_mapping_angular_speed_rad_s', 0.45)
        self.declare_parameter('angular_settle_time_s', 0.20)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.confidence_map_topic = str(self.get_parameter('confidence_map_topic').value).strip()
        self.current_observation_map_topic = str(self.get_parameter('current_observation_map_topic').value).strip()
        self.publish_current_observation_map_enabled = bool(
            self.get_parameter('publish_current_observation_map').value
        )
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.map_width_m = float(self.get_parameter('map_width_m').value)
        self.map_height_m = float(self.get_parameter('map_height_m').value)
        self.map_origin_x = float(self.get_parameter('map_origin_x').value)
        self.map_origin_y = float(self.get_parameter('map_origin_y').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)
        self.laser_yaw = float(self.get_parameter('laser_yaw').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.occupancy_mode = str(self.get_parameter('occupancy_mode').value).strip().lower()
        self.require_pose_update = bool(self.get_parameter('require_pose_update').value)
        self.hit_score_increment = int(self.get_parameter('hit_score_increment').value)
        self.free_score_decrement = int(self.get_parameter('free_score_decrement').value)
        self.occupied_score_threshold = int(self.get_parameter('occupied_score_threshold').value)
        self.free_score_threshold = int(self.get_parameter('free_score_threshold').value)
        self.score_min = int(self.get_parameter('score_min').value)
        self.score_max = int(self.get_parameter('score_max').value)
        self.clear_on_max_range = bool(self.get_parameter('clear_on_max_range').value)
        self.ray_end_trim_m = max(0.0, float(self.get_parameter('ray_end_trim_m').value))
        self.occupied_radius_cells = max(0, int(self.get_parameter('occupied_radius_cells').value))
        self.auto_expand_map = bool(self.get_parameter('auto_expand_map').value)
        self.expansion_padding_m = max(0.0, float(self.get_parameter('expansion_padding_m').value))
        self.max_width_cells = max(
            1,
            int(float(self.get_parameter('max_map_width_m').value) / self.resolution),
        )
        self.max_height_cells = max(
            1,
            int(float(self.get_parameter('max_map_height_m').value) / self.resolution),
        )
        self.max_mapping_angular_speed_rad_s = max(
            0.0,
            float(self.get_parameter('max_mapping_angular_speed_rad_s').value),
        )
        self.angular_settle_time_ns = int(
            max(0.0, float(self.get_parameter('angular_settle_time_s').value)) * 1_000_000_000
        )

        self.width_cells = int(self.map_width_m / self.resolution)
        self.height_cells = int(self.map_height_m / self.resolution)
        self.origin_x = (
            self.map_origin_x
            if math.isfinite(self.map_origin_x)
            else -self.map_width_m / 2.0
        )
        self.origin_y = (
            self.map_origin_y
            if math.isfinite(self.map_origin_y)
            else -self.map_height_m / 2.0
        )

        self.grid = np.full((self.height_cells, self.width_cells), -1, dtype=np.int8)
        self.scores = np.zeros((self.height_cells, self.width_cells), dtype=np.int16)
        self.observed = np.zeros((self.height_cells, self.width_cells), dtype=bool)
        self.current_observation_grid = None
        self.latest_pose = None
        self.previous_pose_for_velocity = None
        self.previous_pose_time_ns = None
        self.angular_speed_rad_s = 0.0
        self.last_turning_time_ns = None
        self.skipped_turn_scans = 0
        self.pose_dirty = False

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.confidence_map_pub = None
        if self.confidence_map_topic:
            self.confidence_map_pub = self.create_publisher(
                OccupancyGrid,
                self.confidence_map_topic,
                map_qos,
            )
        self.current_observation_map_pub = None
        if self.current_observation_map_topic and self.publish_current_observation_map_enabled:
            self.current_observation_map_pub = self.create_publisher(
                OccupancyGrid,
                self.current_observation_map_topic,
                map_qos,
            )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self.pose_sub = self.create_subscription(Pose2D, self.pose_topic, self.pose_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        if self.publish_tf:
            self.publish_static_sensor_tf()

        self.get_logger().info(
            f'Map builder ready: scan={self.scan_topic}, '
            f'pose={self.pose_topic}, map={self.map_topic}, '
            f'confidence_map={self.confidence_map_topic or "disabled"}, '
            f'current_observation_map={self.current_observation_map_topic or "disabled"}'
        )
        self.publish_current_map()
        self.publish_blank_current_observation_map()

    def pose_callback(self, msg) -> None:
        # Accept geometry_msgs/Pose2D without importing it into the subscription type above.
        x = float(msg.x)
        y = float(msg.y)
        theta = normalize_angle(float(msg.theta))
        now_ns = self.get_clock().now().nanoseconds
        if self.previous_pose_for_velocity is not None and self.previous_pose_time_ns is not None:
            dt = (now_ns - self.previous_pose_time_ns) / 1_000_000_000.0
            if dt > 0.0:
                dtheta = normalize_angle(theta - self.previous_pose_for_velocity[2])
                self.angular_speed_rad_s = abs(dtheta) / dt
                if self.is_turning_too_fast():
                    self.last_turning_time_ns = now_ns

        self.previous_pose_for_velocity = (x, y, theta)
        self.previous_pose_time_ns = now_ns
        self.latest_pose = (x, y, theta)
        self.pose_dirty = True
        if self.publish_tf:
            self.publish_base_tf(x, y, theta)

    def scan_callback(self, scan: LaserScan) -> None:
        if self.latest_pose is None:
            return
        if self.require_pose_update and not self.pose_dirty:
            return
        self.pose_dirty = False
        if self.should_skip_scan_for_turning():
            return

        self.current_observation_grid = np.full((self.height_cells, self.width_cells), -1, dtype=np.int8)

        robot_x, robot_y, robot_theta = self.latest_pose
        laser_theta = normalize_angle(robot_theta + self.laser_yaw)
        laser_x = (
            robot_x
            + (self.laser_x * math.cos(robot_theta))
            - (self.laser_y * math.sin(robot_theta))
        )
        laser_y = (
            robot_y
            + (self.laser_x * math.sin(robot_theta))
            + (self.laser_y * math.cos(robot_theta))
        )
        self.ensure_world_point_in_map(laser_x, laser_y)
        laser_cell = self.world_to_grid(laser_x, laser_y)
        if laser_cell is None:
            return

        start_col, start_row = laser_cell
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
                hit_detected = scan.range_min <= range_value < (scan.range_max - 1e-3)

            if not hit_detected and not self.clear_on_max_range:
                angle += scan.angle_increment
                continue

            beam_angle = laser_theta + angle
            free_range = valid_range
            if hit_detected:
                free_range = max(scan.range_min, valid_range - self.ray_end_trim_m)

            end_x = laser_x + free_range * math.cos(beam_angle)
            end_y = laser_y + free_range * math.sin(beam_angle)
            self.ensure_world_point_in_map(end_x, end_y)
            end_cell = self.world_to_grid(end_x, end_y)

            if end_cell is not None:
                laser_cell = self.world_to_grid(laser_x, laser_y)
                if laser_cell is None:
                    angle += scan.angle_increment
                    continue
                start_col, start_row = laser_cell
                self.mark_free_along_ray(start_col, start_row, end_cell[0], end_cell[1])
                if hit_detected:
                    hit_x = laser_x + valid_range * math.cos(beam_angle)
                    hit_y = laser_y + valid_range * math.sin(beam_angle)
                    self.ensure_world_point_in_map(hit_x, hit_y)
                    hit_cell = self.world_to_grid(hit_x, hit_y)
                    if hit_cell is not None:
                        self.mark_occupied(hit_cell[0], hit_cell[1])

            angle += scan.angle_increment

        self.publish_map(scan.header.stamp.sec, scan.header.stamp.nanosec)

    def is_turning_too_fast(self) -> bool:
        return (
            self.max_mapping_angular_speed_rad_s > 0.0
            and self.angular_speed_rad_s > self.max_mapping_angular_speed_rad_s
        )

    def should_skip_scan_for_turning(self) -> bool:
        if self.max_mapping_angular_speed_rad_s <= 0.0:
            return False

        now_ns = self.get_clock().now().nanoseconds
        turning_now = self.is_turning_too_fast()
        settling = (
            self.last_turning_time_ns is not None
            and now_ns - self.last_turning_time_ns < self.angular_settle_time_ns
        )
        if not turning_now and not settling:
            if self.skipped_turn_scans:
                self.get_logger().debug(
                    f'Resumed map updates after skipping {self.skipped_turn_scans} turning scans'
                )
                self.skipped_turn_scans = 0
            return False

        self.skipped_turn_scans += 1
        if self.skipped_turn_scans == 1 or self.skipped_turn_scans % 50 == 0:
            self.get_logger().info(
                f'Pausing map update during turn '
                f'(angular_speed={self.angular_speed_rad_s:.2f} rad/s)'
            )
        return True

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
        if self.confidence_map_pub is not None:
            confidence_msg = OccupancyGrid()
            confidence_msg.header = msg.header
            confidence_msg.info = msg.info
            confidence_msg.data = self.confidence_grid().reshape(-1).tolist()
            self.confidence_map_pub.publish(confidence_msg)
        if self.current_observation_map_pub is not None and self.current_observation_grid is not None:
            current_observation_msg = OccupancyGrid()
            current_observation_msg.header = msg.header
            current_observation_msg.info = msg.info
            current_observation_msg.data = self.current_observation_grid.reshape(-1).tolist()
            self.current_observation_map_pub.publish(current_observation_msg)

    def publish_current_map(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.publish_map(stamp.sec, stamp.nanosec)

    def publish_blank_current_observation_map(self) -> None:
        if self.current_observation_map_pub is None:
            return

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [-1] * (self.width_cells * self.height_cells)
        self.current_observation_map_pub.publish(msg)

    def confidence_grid(self):
        confidence = np.full((self.height_cells, self.width_cells), -1, dtype=np.int8)
        if not np.any(self.observed):
            return confidence

        score_span = max(1, self.score_max - self.score_min)
        normalized_scores = (self.scores.astype(np.float32) - float(self.score_min)) / float(score_span)
        scaled_scores = np.clip(np.rint(normalized_scores * 100.0), 0, 100).astype(np.int8)
        confidence[self.observed] = scaled_scores[self.observed]
        return confidence

    def world_to_grid(self, x: float, y: float):
        col = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        if col < 0 or row < 0 or col >= self.width_cells or row >= self.height_cells:
            return None
        return col, row

    def ensure_world_point_in_map(self, x: float, y: float) -> None:
        if not self.auto_expand_map or not (math.isfinite(x) and math.isfinite(y)):
            return

        padding_cells = int(math.ceil(self.expansion_padding_m / self.resolution))
        add_left = max(0, int(math.ceil((self.origin_x - x) / self.resolution)) + padding_cells)
        add_bottom = max(0, int(math.ceil((self.origin_y - y) / self.resolution)) + padding_cells)

        max_x = self.origin_x + (self.width_cells * self.resolution)
        max_y = self.origin_y + (self.height_cells * self.resolution)
        add_right = max(0, int(math.ceil((x - max_x) / self.resolution)) + 1 + padding_cells)
        add_top = max(0, int(math.ceil((y - max_y) / self.resolution)) + 1 + padding_cells)

        if add_left == 0 and add_right == 0 and add_bottom == 0 and add_top == 0:
            return

        add_left, add_right = self.fit_expansion(self.width_cells, add_left, add_right, self.max_width_cells)
        add_bottom, add_top = self.fit_expansion(self.height_cells, add_bottom, add_top, self.max_height_cells)

        if add_left == 0 and add_right == 0 and add_bottom == 0 and add_top == 0:
            return

        self.grid = np.pad(
            self.grid,
            ((add_bottom, add_top), (add_left, add_right)),
            mode='constant',
            constant_values=-1,
        )
        self.scores = np.pad(
            self.scores,
            ((add_bottom, add_top), (add_left, add_right)),
            mode='constant',
            constant_values=0,
        )
        self.observed = np.pad(
            self.observed,
            ((add_bottom, add_top), (add_left, add_right)),
            mode='constant',
            constant_values=False,
        )
        if self.current_observation_grid is not None:
            self.current_observation_grid = np.pad(
                self.current_observation_grid,
                ((add_bottom, add_top), (add_left, add_right)),
                mode='constant',
                constant_values=-1,
            )
        self.width_cells += add_left + add_right
        self.height_cells += add_bottom + add_top
        self.origin_x -= add_left * self.resolution
        self.origin_y -= add_bottom * self.resolution

        self.get_logger().info(
            f'Expanded {self.map_topic}: '
            f'{self.width_cells}x{self.height_cells} cells, '
            f'origin=({self.origin_x:.2f}, {self.origin_y:.2f})'
        )

    @staticmethod
    def fit_expansion(current: int, low: int, high: int, maximum: int):
        available = maximum - current
        if available <= 0:
            return 0, 0
        total = low + high
        if total <= available:
            return low, high
        fitted_low = min(low, available)
        fitted_high = min(high, available - fitted_low)
        return fitted_low, fitted_high

    def set_cell(self, col: int, row: int, value: int) -> None:
        if 0 <= col < self.width_cells and 0 <= row < self.height_cells:
            self.grid[row, col] = np.int8(value)

    def set_current_observation_cell(self, col: int, row: int, value: int) -> None:
        if self.current_observation_grid is None:
            return
        if 0 <= col < self.width_cells and 0 <= row < self.height_cells:
            if value == 100:
                self.current_observation_grid[row, col] = np.int8(100)
            elif value == 0 and self.current_observation_grid[row, col] != np.int8(100):
                self.current_observation_grid[row, col] = np.int8(0)

    def refresh_cell_from_score(self, col: int, row: int) -> None:
        score = self.scores[row, col]
        if not self.observed[row, col]:
            self.grid[row, col] = np.int8(-1)
        elif score >= self.occupied_score_threshold:
            self.grid[row, col] = np.int8(100)
        elif score <= self.free_score_threshold:
            self.grid[row, col] = np.int8(0)
        else:
            self.grid[row, col] = np.int8(-1)

    def adjust_score(self, col: int, row: int, delta: int) -> None:
        if self.occupancy_mode != 'scored':
            return
        if 0 <= col < self.width_cells and 0 <= row < self.height_cells:
            self.observed[row, col] = True
            updated = int(self.scores[row, col]) + delta
            self.scores[row, col] = np.int16(min(self.score_max, max(self.score_min, updated)))
            self.refresh_cell_from_score(col, row)

    def mark_occupied(self, col: int, row: int) -> None:
        for ncol, nrow in self.neighbor_cells(col, row, self.occupied_radius_cells):
            self.set_current_observation_cell(ncol, nrow, 100)
            if self.occupancy_mode == 'direct':
                self.observed[nrow, ncol] = True
                self.set_cell(ncol, nrow, 100)
                continue
            self.adjust_score(ncol, nrow, self.hit_score_increment)

    def set_free_cell(self, col: int, row: int) -> None:
        self.set_current_observation_cell(col, row, 0)
        if self.occupancy_mode == 'direct':
            self.observed[row, col] = True
            self.set_cell(col, row, 0)
            return
        self.adjust_score(col, row, -self.free_score_decrement)

    def mark_free_along_ray(
        self,
        start_col: int,
        start_row: int,
        end_col: int,
        end_row: int,
    ) -> None:
        for col, row in self.bresenham(start_col, start_row, end_col, end_row):
            if col == end_col and row == end_row:
                break
            self.set_free_cell(col, row)

    def neighbor_cells(self, col: int, row: int, radius: int):
        for drow in range(-radius, radius + 1):
            for dcol in range(-radius, radius + 1):
                if dcol * dcol + drow * drow > radius * radius:
                    continue
                ncol = col + dcol
                nrow = row + drow
                if 0 <= ncol < self.width_cells and 0 <= nrow < self.height_cells:
                    yield ncol, nrow

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
