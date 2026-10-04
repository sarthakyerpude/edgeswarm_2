"""Publish a known simulator spawn pose to AMCL during startup."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_s', 0.5)
        self._count = 0
        self._localized = False
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        # AMCL's first pose output confirms it received and processed an initial
        # pose. Keep retrying through DDS discovery and lifecycle startup until
        # that confirmation arrives.
        self._pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose, 10)
        self._timer = self.create_timer(
            float(self.get_parameter('publish_period_s').value), self._publish)

    def _on_amcl_pose(self, _message):
        if not self._localized:
            self._localized = True
            self.get_logger().info(
                'AMCL published amcl_pose; initial pose was accepted.')
            self._timer.cancel()

    def _publish(self):
        if self._localized:
            return
        if self._publisher.get_subscription_count() == 0:
            if self._count == 0 or self._count % 10 == 0:
                self.get_logger().info(
                    'Waiting for AMCL to subscribe to initialpose.')
            return
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('frame_id').value
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Conservative start-pose covariance: 15 cm planar and 10 degree yaw.
        message.pose.covariance[0] = 0.0225
        message.pose.covariance[7] = 0.0225
        message.pose.covariance[35] = math.radians(10.0) ** 2
        self._publisher.publish(message)
        self._count += 1
        if self._count % 20 == 0:
            self.get_logger().warning(
                'AMCL has not published amcl_pose after '
                f'{self._count} initial-pose attempts; continuing to retry.')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
