import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def covariance_matrix(xy: float, yaw: float):
    covariance = [0.0] * 36
    covariance[0] = xy
    covariance[7] = xy
    covariance[14] = 1e-6
    covariance[21] = 1e-6
    covariance[28] = 1e-6
    covariance[35] = yaw
    return covariance


class InitialPosePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')

        self.declare_parameter('topic', '/initialpose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('use_odom_pose', True)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('covariance_xy', 0.25)
        self.declare_parameter('covariance_yaw', 0.2)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_interval_s', 1.0)

        self.topic = self.get_parameter('topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.use_odom_pose = bool(self.get_parameter('use_odom_pose').value)
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.covariance_xy = float(self.get_parameter('covariance_xy').value)
        self.covariance_yaw = float(self.get_parameter('covariance_yaw').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.sent_count = 0
        self.latest_odom = None
        self.waiting_logged = False

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.timer = self.create_timer(
            float(self.get_parameter('publish_interval_s').value),
            self.publish_pose,
        )

        source = self.odom_topic if self.use_odom_pose else 'configured pose'
        self.get_logger().info(
            f'Initial pose publisher ready: topic={self.topic}, source={source}'
        )

    def odom_callback(self, msg: Odometry) -> None:
        if self.latest_odom is None:
            self.get_logger().info('Received /odom; publishing AMCL initial pose from Webots odom.')
        self.latest_odom = msg

    def initial_pose_values(self):
        if self.use_odom_pose:
            if self.latest_odom is None:
                return None
            pose = self.latest_odom.pose.pose
            return (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_to_yaw(pose.orientation),
            )
        return self.x, self.y, self.yaw

    def publish_pose(self) -> None:
        pose_values = self.initial_pose_values()
        if pose_values is None:
            if not self.waiting_logged:
                self.get_logger().info(f'Waiting for {self.odom_topic} before publishing initial pose.')
                self.waiting_logged = True
            return

        x, y, yaw = pose_values
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)
        msg.pose.covariance = covariance_matrix(self.covariance_xy, self.covariance_yaw)
        self.publisher.publish(msg)
        self.sent_count += 1
        self.get_logger().info(
            f'Published initial pose #{self.sent_count}: ({x:.2f}, {y:.2f}, {yaw:.2f})'
        )

        if self.sent_count >= self.publish_count:
            self.get_logger().info('Initial pose complete.')
            self.timer.cancel()


def main() -> None:
    rclpy.init()
    node = InitialPosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
