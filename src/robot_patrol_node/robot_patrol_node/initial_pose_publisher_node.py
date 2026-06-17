import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


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
    """Publish an initial pose so AMCL starts without manual RViz interaction."""

    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')

        self.declare_parameter('topic', '/initialpose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('covariance_xy', 0.25)
        self.declare_parameter('covariance_yaw', 0.2)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_interval_s', 1.0)

        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.covariance_xy = float(self.get_parameter('covariance_xy').value)
        self.covariance_yaw = float(self.get_parameter('covariance_yaw').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.sent_count = 0

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.topic, 10)
        self.timer = self.create_timer(
            float(self.get_parameter('publish_interval_s').value),
            self.publish_pose,
        )

        self.get_logger().info(
            f'Initial pose publisher ready: topic={self.topic}, pose=({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})'
        )

    def publish_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        # A zero timestamp lets AMCL use the latest odom transform instead of
        # failing on small clock-ordering differences between pose and TF.
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        msg.pose.covariance = covariance_matrix(self.covariance_xy, self.covariance_yaw)
        self.publisher.publish(msg)
        self.sent_count += 1
        self.get_logger().info(f'Published initial pose #{self.sent_count}')

        if self.sent_count >= self.publish_count:
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
