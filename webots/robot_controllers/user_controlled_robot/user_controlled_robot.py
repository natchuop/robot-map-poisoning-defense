import json
import math
import os
import socket
import traceback

from controller import Keyboard, Robot


TIME_STEP = 64
DEFAULT_TURN_SPEED_RATIO = float(os.environ.get('WEBOTS_TURN_SPEED_RATIO', '0.67'))
LOG_INTERVAL_STEPS = int(os.environ.get('WEBOTS_LOG_INTERVAL_STEPS', '20'))
ROBOT_ID = os.environ.get('WEBOTS_ROBOT_ID', os.environ.get('ROBOT_NAME', 'robot_1'))
MAP_ID = os.environ.get('WEBOTS_MAP_ID', '').strip()
BRIDGE_PROTOCOL = os.environ.get('WEBOTS_BRIDGE_PROTOCOL', 'tcp').strip().lower()
BRIDGE_TARGETS = [
    target.strip()
    for target in os.environ.get(
        'WEBOTS_BRIDGE_TARGETS',
        '172.28.64.1,127.0.0.1',
    ).split(',')
    if target.strip()
]
BRIDGE_PORT = int(os.environ.get('WEBOTS_BRIDGE_PORT', '5005'))
HEADING_SIGN = float(os.environ.get('WEBOTS_HEADING_SIGN', '1.0'))
HEADING_OFFSET = float(os.environ.get('WEBOTS_HEADING_OFFSET', '0.0'))
FORWARD_AXIS = os.environ.get('WEBOTS_FORWARD_AXIS', 'x').strip().lower()
HEADING_SOURCE = os.environ.get('WEBOTS_HEADING_SOURCE', 'rpy_z').strip().lower()
DEBUG_FRAME = os.environ.get('WEBOTS_DEBUG_FRAME', '').strip().lower() in {'1', 'true', 'yes', 'on'}


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def env_float(name, default):
    value = os.environ.get(name)
    if value is None or not str(value).strip():
        return default
    return float(value)


def local_forward_vector():
    axes = {
        'x': (1.0, 0.0, 0.0),
        '+x': (1.0, 0.0, 0.0),
        '-x': (-1.0, 0.0, 0.0),
        'z': (0.0, 0.0, 1.0),
        '+z': (0.0, 0.0, 1.0),
        '-z': (0.0, 0.0, -1.0),
    }
    return axes.get(FORWARD_AXIS, axes['x'])


def rotate_vector_by_quaternion(vector, quaternion):
    qx, qy, qz, qw = (float(value) for value in quaternion)
    vx, vy, vz = vector

    # Quaternion-vector rotation without requiring scipy in the Webots controller.
    tx = 2.0 * ((qy * vz) - (qz * vy))
    ty = 2.0 * ((qz * vx) - (qx * vz))
    tz = 2.0 * ((qx * vy) - (qy * vx))

    return (
        vx + (qw * tx) + ((qy * tz) - (qz * ty)),
        vy + (qw * ty) + ((qz * tx) - (qx * tz)),
        vz + (qw * tz) + ((qx * ty) - (qy * tx)),
    )


def quaternion_heading_from_imu(imu):
    forward_x, _forward_y, forward_z = rotate_vector_by_quaternion(
        local_forward_vector(),
        imu.getQuaternion(),
    )
    return normalize_angle((HEADING_SIGN * math.atan2(forward_z, forward_x)) + HEADING_OFFSET)


def planar_heading_from_imu(imu, rpy):
    if HEADING_SOURCE == 'quaternion':
        return quaternion_heading_from_imu(imu)

    # TurtleBot3Burger in these Webots worlds reports planar yaw in rpy[2].
    return normalize_angle((HEADING_SIGN * float(rpy[2])) + HEADING_OFFSET)


def candidate_headings(quaternion):
    headings = {}
    for axis in ('x', '-x', 'z', '-z'):
        forward_x, _forward_y, forward_z = rotate_vector_by_quaternion(
            {
                'x': (1.0, 0.0, 0.0),
                '-x': (-1.0, 0.0, 0.0),
                'z': (0.0, 0.0, 1.0),
                '-z': (0.0, 0.0, -1.0),
            }[axis],
            quaternion,
        )
        headings[axis] = normalize_angle(math.atan2(forward_z, forward_x))
    return headings


class BridgeSender:
    """Send newline-delimited JSON packets to the ROS 2 bridge."""

    def __init__(self, targets, port):
        self.targets = targets
        self.port = port
        self.sock = None
        self.connected_target = None
        self.failure_count = 0
        self.protocol = BRIDGE_PROTOCOL
        self._udp_socket = None
        self._udp_connected = False

    def _rotate_failed_target(self, target):
        if target in self.targets and len(self.targets) > 1:
            remaining = [candidate for candidate in self.targets if candidate != target]
            self.targets = remaining + [target]

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.connected_target = None
        self._udp_connected = False
        if self._udp_socket is not None:
            try:
                self._udp_socket.close()
            except OSError:
                pass
        self._udp_socket = None

    def _ensure_udp_socket(self):
        if self._udp_socket is None:
            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return self._udp_socket

    def _connect(self):
        if self.protocol == 'udp':
            if self._udp_socket is None:
                self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.connected_target = self.targets[0] if self.targets else '127.0.0.1'
            if not self._udp_connected:
                print(
                    f'connected to ROS bridge at udp://{self.connected_target}:{self.port}',
                    flush=True,
                )
            self._udp_connected = True
            return True

        self.close()
        for target in self.targets:
            try:
                sock = socket.create_connection((target, self.port), timeout=0.2)
                sock.settimeout(0.2)
                self.sock = sock
                self.connected_target = target
                print(f'connected to ROS bridge at tcp://{target}:{self.port}', flush=True)
                return True
            except OSError:
                continue
        return False

    def send(self, payload):
        if self.protocol == 'udp':
            try:
                if not self._udp_connected:
                    self._connect()
                self._ensure_udp_socket().sendto(payload + b'\n', (self.connected_target, self.port))
                self.failure_count = 0
                return True
            except OSError as exc:
                print(f'ROS bridge send failed ({self.connected_target}:{self.port}): {exc}', flush=True)
                self._rotate_failed_target(self.connected_target)
                self.close()
                return False

        if self.sock is None and not self._connect():
            self.failure_count += 1
            if self.failure_count == 1 or self.failure_count % 50 == 0:
                targets = ', '.join(f'tcp://{target}:{self.port}' for target in self.targets)
                print(f'waiting for ROS bridge; tried {targets}', flush=True)
            return False

        try:
            self.sock.sendall(payload + b'\n')
            self.failure_count = 0
            return True
        except OSError as exc:
            print(f'ROS bridge send failed ({self.connected_target}:{self.port}): {exc}', flush=True)
            self._rotate_failed_target(self.connected_target)
            self.close()
            return False


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep()) or TIME_STEP

    print(f'user_controlled_robot starting robot_id={ROBOT_ID} map_id={MAP_ID or "unknown"}', flush=True)

    try:
        gps = robot.getDevice('gps')
        gps.enable(timestep)

        imu = robot.getDevice('inertial unit')
        imu.enable(timestep)

        lidar = robot.getDevice('LDS-01')
        lidar.enable(timestep)
        lidar.enablePointCloud()

        lidar_main_motor = robot.getDevice('LDS-01_main_motor')
        lidar_secondary_motor = robot.getDevice('LDS-01_secondary_motor')
        lidar_main_motor.setPosition(float('inf'))
        lidar_secondary_motor.setPosition(float('inf'))
        lidar_main_motor.setVelocity(30.0)
        lidar_secondary_motor.setVelocity(60.0)

        right_motor = robot.getDevice('right wheel motor')
        left_motor = robot.getDevice('left wheel motor')
        right_motor.setPosition(float('inf'))
        left_motor.setPosition(float('inf'))
        right_motor.setVelocity(0.0)
        left_motor.setVelocity(0.0)
        max_wheel_speed = min(
            float(left_motor.getMaxVelocity()),
            float(right_motor.getMaxVelocity()),
        )
        drive_speed = min(env_float('WEBOTS_DRIVE_SPEED', max_wheel_speed), max_wheel_speed)
        turn_speed = min(
            env_float('WEBOTS_TURN_SPEED', drive_speed * DEFAULT_TURN_SPEED_RATIO),
            max_wheel_speed,
        )

        keyboard = Keyboard()
        keyboard.enable(timestep)

        lidar_width = lidar.getHorizontalResolution()
        lidar_fov = float(lidar.getFov())
        lidar_max_range = lidar.getMaxRange()

        sender = BridgeSender(BRIDGE_TARGETS, BRIDGE_PORT)
        counter = 0

        print(f'ROS bridge targets {", ".join(f"tcp://{target}:{BRIDGE_PORT}" for target in BRIDGE_TARGETS)}', flush=True)
        print('GPS, IMU, LiDAR, and keyboard initialized', flush=True)
        print('keyboard teleop ready: W forward, S backward, A left, D right', flush=True)
        print(
            f'user_controlled_robot wheel speed cap {max_wheel_speed:.2f} rad/s '
            f'(drive={drive_speed:.2f}, turn={turn_speed:.2f})',
            flush=True,
        )
    except Exception as exc:
        print(f'controller startup failed: {exc}', flush=True)
        raise

    while robot.step(timestep) != -1:
        try:
            pressed_keys = set()
            key = keyboard.getKey()
            while key != -1:
                pressed_keys.add(key)
                key = keyboard.getKey()

            forward = ord('W') in pressed_keys or ord('w') in pressed_keys
            backward = ord('S') in pressed_keys or ord('s') in pressed_keys
            turn_left = ord('A') in pressed_keys or ord('a') in pressed_keys
            turn_right = ord('D') in pressed_keys or ord('d') in pressed_keys

            linear = 0.0
            angular = 0.0

            if forward and not backward:
                linear += drive_speed
            elif backward and not forward:
                linear -= drive_speed

            if turn_left and not turn_right:
                angular += turn_speed
            elif turn_right and not turn_left:
                angular -= turn_speed

            left_speed = clamp(linear - angular, -max_wheel_speed, max_wheel_speed)
            right_speed = clamp(linear + angular, -max_wheel_speed, max_wheel_speed)

            gps_values = gps.getValues()
            robot_x = gps_values[0]
            robot_y = gps_values[1]

            rpy = imu.getRollPitchYaw()
            quaternion = imu.getQuaternion()
            robot_heading = planar_heading_from_imu(imu, rpy)

            if LOG_INTERVAL_STEPS > 0 and counter % LOG_INTERVAL_STEPS == 0:
                print(
                    f'pose -> x: {robot_x:.2f} y: {robot_y:.2f} heading: {robot_heading:.2f} '
                    f'imu_rpy: {rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}',
                    flush=True,
                )

            lidar_values = lidar.getRangeImage()
            ranges = []
            for value in reversed(lidar_values):
                if math.isinf(value) or math.isnan(value):
                    ranges.append(float(lidar_max_range))
                else:
                    ranges.append(float(value))

            if DEBUG_FRAME and LOG_INTERVAL_STEPS > 0 and counter % LOG_INTERVAL_STEPS == 0:
                finite_ranges = [
                    (index, value)
                    for index, value in enumerate(ranges)
                    if math.isfinite(value) and not math.isnan(value)
                ]
                closest_index, closest_range = min(finite_ranges, key=lambda item: item[1], default=(-1, float('nan')))
                closest_angle = (-lidar_fov / 2.0) + (
                    closest_index * (lidar_fov / float(max(lidar_width - 1, 1)))
                )
                headings = candidate_headings(quaternion)
                heading_text = ' '.join(f'{axis}:{value:.2f}' for axis, value in headings.items())
                print(
                    'frame_debug -> '
                    f'keys={sorted(pressed_keys)} source={HEADING_SOURCE} selected_axis={FORWARD_AXIS} '
                    f'sign={HEADING_SIGN:.1f} '
                    f'quat=({quaternion[0]:.3f},{quaternion[1]:.3f},{quaternion[2]:.3f},{quaternion[3]:.3f}) '
                    f'headings={heading_text} closest_scan_index={closest_index} '
                    f'closest_scan_angle={closest_angle:.2f} closest_range={closest_range:.2f}',
                    flush=True,
                )

            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            packet = {
                'pose': {
                    'x': float(robot_x),
                    'y': float(robot_y),
                    'theta': float(robot_heading),
                },
                'scan': {
                    'angle_min': -lidar_fov / 2.0,
                    'angle_max': lidar_fov / 2.0,
                    'angle_increment': lidar_fov / float(max(lidar_width - 1, 1)),
                    'range_min': float(lidar.getMinRange()),
                    'range_max': float(lidar_max_range),
                    'scan_time': timestep / 1000.0,
                    'ranges': ranges,
                },
            }

            payload = json.dumps(packet, separators=(',', ':')).encode('utf-8')
            sent = sender.send(payload)
            if sent and LOG_INTERVAL_STEPS > 0 and counter % LOG_INTERVAL_STEPS == 0:
                print('sent bridge packet', flush=True)
            counter += 1
        except Exception as exc:
            print(f'user_controlled_robot step failed: {exc}', flush=True)
            print(traceback.format_exc(), flush=True)
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)


if __name__ == '__main__':
    main()
