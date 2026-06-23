import math

from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalizedPose2DNode(Node):
    """Republish AMCL's map-frame pose as Pose2D for map-building overlays."""

    def __init__(self) -> None:
        super().__init__('localized_pose2d')

        self.declare_parameter('input_topic', '/amcl_pose')
        self.declare_parameter('output_topic', '/localized_pose')

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        self.publisher = self.create_publisher(Pose2D, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.input_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f'Localized pose relay ready: {self.input_topic} -> {self.output_topic}'
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        output = Pose2D()
        output.x = float(pose.position.x)
        output.y = float(pose.position.y)
        output.theta = quaternion_to_yaw(pose.orientation)
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = LocalizedPose2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
