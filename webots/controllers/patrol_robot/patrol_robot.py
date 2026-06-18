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


def clamp(value, low, high):
    return max(low, min(high, value))


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

        cmd_vel = packet.get('cmd_vel')
        if not isinstance(cmd_vel, dict):
            return None

        return {
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
        lidar_max_range = lidar.getMaxRange()

        sender = BridgeSender(BRIDGE_TARGETS, BRIDGE_PORT)
        counter = 0

        current_linear_x = 0.0
        current_angular_z = 0.0
        last_cmd_time = 0.0

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
        robot_y = gps_values[2]

        rpy = imu.getRollPitchYaw()
        robot_heading = rpy[2]

        if counter % 20 == 0:
            print(
                f'pose -> x: {robot_x:.2f} y: {robot_y:.2f} heading: {robot_heading:.2f} '
                f'cmd_vel -> linear_x: {current_linear_x:.2f} angular_z: {current_angular_z:.2f}',
                flush=True,
            )

        lidar_values = lidar.getRangeImage()
        ranges = []
        for value in lidar_values:
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
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': (2.0 * math.pi) / float(lidar_width),
                'range_min': float(lidar.getMinRange()),
                'range_max': float(lidar_max_range),
                'scan_time': timestep / 1000.0,
                'ranges': ranges,
            },
        }

        payload = json.dumps(packet, separators=(',', ':')).encode('utf-8')
        sent = sender.send(payload)

        if sent and counter % 20 == 0:
            print('sent bridge packet', flush=True)

        counter += 1


if __name__ == '__main__':
    main()