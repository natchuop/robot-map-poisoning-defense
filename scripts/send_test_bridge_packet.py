#!/usr/bin/env python3
import argparse
import json
import socket
import time


parser = argparse.ArgumentParser(description='Send fake Webots bridge packets for ROS 2 debugging.')
parser.add_argument('--host', default='127.0.0.1')
parser.add_argument('--port', type=int, default=5005)
parser.add_argument('--count', type=int, default=1)
parser.add_argument('--delay', type=float, default=0.1)
args = parser.parse_args()


with socket.create_connection((args.host, args.port), timeout=2.0) as sock:
    for index in range(args.count):
        packet = {
            'pose': {'x': 1.23 + index * 0.01, 'y': 4.56, 'theta': 0.78},
            'scan': {
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 1.570795,
                'range_min': 0.05,
                'range_max': 3.5,
                'scan_time': 0.064,
                'ranges': [1.0, 1.1, 1.2, 1.3],
            },
        }
        sock.sendall(json.dumps(packet, separators=(',', ':')).encode('utf-8') + b'\n')
        time.sleep(args.delay)

print(f'sent {args.count} TCP bridge test packet(s) to {args.host}:{args.port}')
