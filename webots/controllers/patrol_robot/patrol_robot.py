import json
import math
import os
import socket
import time

from controller import Robot


TIME_STEP = 64

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

WHEEL_RADIUS = float(os.environ.get('WEBOTS_WHEEL_RADIUS', '0.033'))
WHEEL_BASE = float(os.environ.get('WEBOTS_WHEEL_BASE', '0.16'))
MAX_WHEEL_SPEED = float(os.environ.get('WEBOTS_MAX_WHEEL_SPEED', '6.28'))
CMD_TIMEOUT_SEC = float(os.environ.get('WEBOTS_CMD_TIMEOUT_SEC', '1.0'))
LOG_INTERVAL_STEPS = int(os.environ.get('WEBOTS_LOG_INTERVAL_STEPS', '300'))
CHECKPOINT_TOUCH_ROBOT_RADIUS = float(os.environ.get('WEBOTS_CHECKPOINT_TOUCH_ROBOT_RADIUS', '0.18'))
CHECKPOINT_TOUCH_SLACK = float(os.environ.get('WEBOTS_CHECKPOINT_TOUCH_SLACK', '0.01'))
HEADING_SIGN = float(os.environ.get('WEBOTS_HEADING_SIGN', '1.0'))
HEADING_OFFSET = float(os.environ.get('WEBOTS_HEADING_OFFSET', '0.0'))
FORWARD_AXIS = os.environ.get('WEBOTS_FORWARD_AXIS', 'x').strip().lower()
HEADING_SOURCE = os.environ.get('WEBOTS_HEADING_SOURCE', 'rpy_z').strip().lower()
CHECKPOINT_BLOCK_SIZE = 0.12
CHECKPOINTS = {
    'A': (-1.49882, 1.84407),
    'B': (1.5267, -0.221987),
    'C': (-0.416565, -1.35783),
    'D': (-2.63149, -0.778393),
}


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


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

    return normalize_angle((HEADING_SIGN * float(rpy[2])) + HEADING_OFFSET)


def checkpoint_touch_event(checkpoint_name, robot_x, robot_y):
    marker = CHECKPOINTS.get(checkpoint_name)
    if marker is None:
        return None

    marker_x, marker_y = marker
    half_size = CHECKPOINT_BLOCK_SIZE * 0.5
    dx = abs(robot_x - marker_x)
    dy = abs(robot_y - marker_y)
    outside_x = max(dx - half_size, 0.0)
    outside_y = max(dy - half_size, 0.0)
    edge_gap = math.hypot(outside_x, outside_y)
    touched = edge_gap <= CHECKPOINT_TOUCH_ROBOT_RADIUS + CHECKPOINT_TOUCH_SLACK
    if not touched:
        return None

    center_distance = math.hypot(robot_x - marker_x, robot_y - marker_y)
    return {
        'name': checkpoint_name,
        'distance': float(center_distance),
        'edge_gap': float(edge_gap),
        'robot_radius': float(CHECKPOINT_TOUCH_ROBOT_RADIUS),
    }


def cmd_vel_to_wheel_speeds(linear_x, angular_z):
    left_speed = (linear_x - angular_z * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    right_speed = (linear_x + angular_z * WHEEL_BASE / 2.0) / WHEEL_RADIUS

    left_speed = clamp(left_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    right_speed = clamp(right_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

    return left_speed, right_speed


class BridgeSender:
    """Send sensor packets to ROS and receive cmd_vel packets from ROS."""

    def __init__(self, targets, port):
        self.targets = targets
        self.port = port
        self.sock = None
        self.connected_target = None
        self.failure_count = 0
        self.protocol = BRIDGE_PROTOCOL
        self._udp_socket = None
        self._udp_connected = False
        self._recv_buffer = b''

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
        self._recv_buffer = b''

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
            self._udp_socket.setblocking(False)
        return self._udp_socket

    def _connect(self):
        if self.protocol == 'udp':
            if self._udp_socket is None:
                self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._udp_socket.setblocking(False)

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
                sock.settimeout(0.0)
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
                self._ensure_udp_socket().sendto(
                    payload + b'\n',
                    (self.connected_target, self.port),
                )
                self.failure_count = 0
                return True
            except OSError as exc:
                print(
                    f'ROS bridge send failed ({self.connected_target}:{self.port}): {exc}',
                    flush=True,
                )
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
            print(
                f'ROS bridge send failed ({self.connected_target}:{self.port}): {exc}',
                flush=True,
            )
            self._rotate_failed_target(self.connected_target)
            self.close()
            return False

    def receive_commands(self):
        commands = []

        if self.protocol == 'udp':
            if self._udp_socket is None:
                return commands

            while True:
                try:
                    data, _addr = self._udp_socket.recvfrom(65535)
                except BlockingIOError:
                    break
                except OSError:
                    break

                for line in data.splitlines():
                    command = self._decode_command(line)
                    if command is not None:
                        commands.append(command)

            return commands

        if self.sock is None:
            return commands

        while True:
            try:
                chunk = self.sock.recv(65535)
            except BlockingIOError:
                break
            except OSError:
                self.close()
                break

            if not chunk:
                self.close()
                break

            self._recv_buffer += chunk

            while b'\n' in self._recv_buffer:
                line, self._recv_buffer = self._recv_buffer.split(b'\n', 1)
                command = self._decode_command(line)
                if command is not None:
                    commands.append(command)

        return commands

    def _decode_command(self, line):
        if not line:
            return None

        try:
            packet = json.loads(line.decode('utf-8'))
        except json.JSONDecodeError:
            return None

        active_checkpoint = packet.get('active_checkpoint')
        if isinstance(active_checkpoint, dict):
            return {
                'type': 'active_checkpoint',
                'name': str(active_checkpoint.get('name', '')).strip(),
            }

        checkpoint_event = packet.get('checkpoint_event')
        if isinstance(checkpoint_event, dict):
            message = str(checkpoint_event.get('message', '')).strip()
            if message:
                return {
                    'type': 'checkpoint_event',
                    'message': message,
                }
            return None

        cmd_vel = packet.get('cmd_vel')
        if not isinstance(cmd_vel, dict):
            return None

        return {
            'type': 'cmd_vel',
            'linear_x': float(cmd_vel.get('linear_x', 0.0)),
            'angular_z': float(cmd_vel.get('angular_z', 0.0)),
        }


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep()) or TIME_STEP

    print(f'patrol_robot starting robot_id={ROBOT_ID} map_id={MAP_ID or "unknown"}', flush=True)

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

        lidar_width = lidar.getHorizontalResolution()
        lidar_fov = float(lidar.getFov())
        lidar_max_range = lidar.getMaxRange()

        sender = BridgeSender(BRIDGE_TARGETS, BRIDGE_PORT)
        counter = 0

        current_linear_x = 0.0
        current_angular_z = 0.0
        last_cmd_time = 0.0
        active_checkpoint = ''
        last_reported_touch = ''

        print(
            f'ROS bridge targets {", ".join(f"tcp://{target}:{BRIDGE_PORT}" for target in BRIDGE_TARGETS)}',
            flush=True,
        )
        print('GPS, IMU, LiDAR, and cmd_vel motor control initialized', flush=True)

    except Exception as exc:
        print(f'controller startup failed: {exc}', flush=True)
        raise

    while robot.step(timestep) != -1:
        for command in sender.receive_commands():
            if command.get('type') == 'checkpoint_event':
                print(f'[checkpoint] {command["message"]}', flush=True)
                continue

            if command.get('type') == 'active_checkpoint':
                active_checkpoint = command.get('name', '')
                last_reported_touch = ''
                if active_checkpoint:
                    print(f'[checkpoint] ACTIVE checkpoint {active_checkpoint}', flush=True)
                continue

            current_linear_x = command['linear_x']
            current_angular_z = command['angular_z']
            last_cmd_time = time.time()

        if time.time() - last_cmd_time > CMD_TIMEOUT_SEC:
            current_linear_x = 0.0
            current_angular_z = 0.0

        left_speed, right_speed = cmd_vel_to_wheel_speeds(
            current_linear_x,
            current_angular_z,
        )

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        gps_values = gps.getValues()
        robot_x = gps_values[0]
        robot_y = gps_values[1]

        rpy = imu.getRollPitchYaw()
        robot_heading = planar_heading_from_imu(imu, rpy)

        if LOG_INTERVAL_STEPS > 0 and counter % LOG_INTERVAL_STEPS == 0:
            print(
                f'pose -> x: {robot_x:.2f} y: {robot_y:.2f} heading: {robot_heading:.2f} '
                f'imu_rpy: {rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f} '
                f'cmd_vel -> linear_x: {current_linear_x:.2f} angular_z: {current_angular_z:.2f}',
                flush=True,
            )

        lidar_values = lidar.getRangeImage()
        ranges = []
        for value in reversed(lidar_values):
            if math.isinf(value) or math.isnan(value):
                ranges.append(float(lidar_max_range))
            else:
                ranges.append(float(value))

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

        contact_event = checkpoint_touch_event(active_checkpoint, robot_x, robot_y)
        if contact_event is not None:
            packet['checkpoint_contact'] = contact_event
            if last_reported_touch != active_checkpoint:
                print(
                    f'[checkpoint] TOUCHING checkpoint {active_checkpoint} '
                    f'(center distance {contact_event["distance"]:.2f} m)',
                    flush=True,
                )
                last_reported_touch = active_checkpoint

        payload = json.dumps(packet, separators=(',', ':')).encode('utf-8')
        sent = sender.send(payload)

        if sent and LOG_INTERVAL_STEPS > 0 and counter % LOG_INTERVAL_STEPS == 0:
            print('sent bridge packet', flush=True)

        counter += 1


if __name__ == '__main__':
    main()
