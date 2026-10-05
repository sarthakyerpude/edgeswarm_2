"""Fail-safe velocity interceptor driven by amr_description/MotionPermit.

Nav2 publishes `cmd_vel_raw`. This gate is the sole publisher of `cmd_vel`.
Fresh GO/SLOW permits pass a bounded planar Twist; missing/stale permits and
all other actions emit zero velocity continuously. A stale command also stops.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from amr_description.msg import MotionPermit
from amr_fleet.nodes.qos_profiles import PERMIT_QOS


class VelocityGate(Node):
    def __init__(self):
        super().__init__('velocity_gate')
        self.declare_parameter('permit_timeout_s', 0.5)
        self.declare_parameter('command_timeout_s', 0.25)
        self.declare_parameter('max_linear_x', 0.4)
        self.declare_parameter('max_angular_z', 1.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self._permit = None
        self._permit_time = 0.0
        self._command = Twist()
        self._command_time = 0.0
        self.create_subscription(Twist, 'cmd_vel_raw', self._on_velocity, 10)
        self.create_subscription(MotionPermit, 'motion_permit', self._on_permit,
                                 PERMIT_QOS)
        self._output = self.create_publisher(Twist, 'cmd_vel', 10)
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self._timer = self.create_timer(1.0 / rate, self._publish_gated_velocity)
        self.get_logger().info('Fail-safe gate active: waiting for a fresh motion_permit')

    def _on_permit(self, permit):
        self._permit = permit
        self._permit_time = time.monotonic()

    def _on_velocity(self, incoming):
        self._command = incoming
        self._command_time = time.monotonic()

    def _publish_gated_velocity(self):
        now = time.monotonic()
        permit_timeout = max(0.0, float(self.get_parameter('permit_timeout_s').value))
        command_timeout = max(0.0, float(self.get_parameter('command_timeout_s').value))
        permit = self._permit
        command_is_fresh = now - self._command_time <= command_timeout
        permit_is_fresh = permit is not None and now - self._permit_time <= permit_timeout
        outgoing = Twist()
        if command_is_fresh and permit_is_fresh and permit.action in (
            MotionPermit.ACTION_GO, MotionPermit.ACTION_SLOW
        ):
            scale = float(permit.speed_scale)
            if math.isfinite(scale):
                scale = max(0.0, min(1.0, scale))
                max_linear = abs(float(self.get_parameter('max_linear_x').value))
                max_angular = abs(float(self.get_parameter('max_angular_z').value))
                linear_x = self._command.linear.x * scale
                angular_z = self._command.angular.z * scale
                if math.isfinite(linear_x) and math.isfinite(angular_z):
                    outgoing.linear.x = max(-max_linear, min(max_linear, linear_x))
                    outgoing.angular.z = max(-max_angular, min(max_angular, angular_z))
        self._output.publish(outgoing)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
