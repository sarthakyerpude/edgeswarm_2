"""Relay Gazebo's global TF streams into the current robot namespace.

The existing single-robot Gazebo bridge publishes /tf and /tf_static globally.
Nav2 bringup scopes TF topics in a robot namespace, so this bridges those
streams into that namespace. Multi-robot frame IDs must be made unique before
using this relay for more than one robot.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__('gazebo_tf_relay')
        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._dynamic_pub = self.create_publisher(TFMessage, 'tf', dynamic_qos)
        self._static_pub = self.create_publisher(TFMessage, 'tf_static', static_qos)
        self.create_subscription(TFMessage, '/tf', self._dynamic_pub.publish, dynamic_qos)
        self.create_subscription(TFMessage, '/tf_static', self._static_pub.publish, static_qos)


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
