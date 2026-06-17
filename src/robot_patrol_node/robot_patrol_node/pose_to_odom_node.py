import math

import rclpy
from geometry_msgs.msg import Pose2D, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


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


class PoseToOdomNode(Node):
    """Mirror the Webots pose into an odom frame for AMCL smoke tests."""

    def __init__(self) -> None:
        super().__init__('pose_to_odom')

        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('covariance_xy', 0.05)
        self.declare_parameter('covariance_yaw', 0.1)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.covariance_xy = float(self.get_parameter('covariance_xy').value)
        self.covariance_yaw = float(self.get_parameter('covariance_yaw').value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pose_sub = self.create_subscription(Pose2D, self.pose_topic, self.pose_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_pose = None
        self.timer = self.create_timer(0.1, self.publish_latest_pose)

        self.get_logger().info(
            f'Pose-to-odom ready: pose={self.pose_topic}, odom={self.odom_topic}, '
            f'frames={self.odom_frame}->{self.base_frame}'
        )

    def pose_callback(self, msg: Pose2D) -> None:
        x = float(msg.x)
        y = float(msg.y)
        theta = float(msg.theta)
        self.latest_pose = (x, y, theta)

    def publish_latest_pose(self) -> None:
        if self.latest_pose is None:
            return

        x, y, theta = self.latest_pose

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(theta)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        odom.pose.covariance = covariance_matrix(self.covariance_xy, self.covariance_yaw)
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = yaw_to_quaternion(theta)
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = PoseToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
