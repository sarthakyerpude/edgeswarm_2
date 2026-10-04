#!/usr/bin/env python3
"""
mock_robot_node - a robot body with no Gazebo.

WHY THIS MATTERS FOR YOUR TEAM SPLIT
You are writing coordination; a teammate owns the Gazebo world. This node lets
BOTH parts be tested independently and in parallel. It publishes exactly the
topics Gazebo would publish - odom and scan, in the same namespace, with the
same message types - and consumes motion_permit to move.

So `ros2 launch amr_description three_robots_mock.launch.py` gives a complete,
observable three-robot fleet with NO simulator, NO bridge and NO warehouse
model. Your teammate can verify every coordination behaviour in the test plan
before the Gazebo integration exists, and if a bug appears after integration
you immediately know which half it came from.

It is a test fixture, not a simulator. There is no physics: a kinematic
integrator moves the robot along the coordination path, scaled by the permit.
That is precisely enough to exercise conflicts, zones, deadlock and auctions.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from amr_description.msg import MotionPermit as RosPermit
from amr_fleet.nodes.qos_profiles import DIAGNOSTIC_QOS, SENSOR_QOS


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class MockRobotNode(Node):

    def __init__(self):
        super().__init__("mock_robot")

        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("start_x", 1.0)
        self.declare_parameter("start_y", 1.0)
        self.declare_parameter("start_theta", 0.0)
        self.declare_parameter("v_nominal", 0.6)   # = fleet cruise (1.5x)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("scan_samples", 60)
        self.declare_parameter("scan_range_max", 6.0)
        self.declare_parameter("permit_timeout_s", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.robot_id = self.get_parameter("robot_id").value
        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)
        self.theta = float(self.get_parameter("start_theta").value)
        self.v_nom = float(self.get_parameter("v_nominal").value)

        self.speed_scale = 0.0
        self._permit_time = 0.0
        self.path_pts = []
        self.path_i = 0
        self.final_yaw = None       # heading of the path's last pose
        self.w = 0.0

        self.pub_odom = self.create_publisher(Odometry, "odom", SENSOR_QOS)
        self.pub_scan = self.create_publisher(LaserScan, "scan", SENSOR_QOS)

        self.create_subscription(RosPermit, "motion_permit",
                                 self.cb_permit, DIAGNOSTIC_QOS)
        self.create_subscription(Path, "coordination_path",
                                 self.cb_path, DIAGNOSTIC_QOS)

        hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / hz
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"mock_robot {self.robot_id} at ({self.x:.2f},{self.y:.2f}) "
            f"[NO GAZEBO - test fixture]")

    def cb_permit(self, m: RosPermit):
        self.speed_scale = float(m.speed_scale)
        self._permit_time = time.monotonic()

    def cb_path(self, m: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        q = m.poses[-1].pose.orientation if m.poses else None
        yaw = 2.0 * math.atan2(q.z, q.w) if q is not None else None
        if pts != self.path_pts or yaw != self.final_yaw:
            self.path_pts = pts
            self.path_i = 0
            self.final_yaw = yaw

    def step(self):
        # Kinematic follow of the coordination path, scaled by the permit.
        v = 0.0
        permit_timeout = max(0.0, float(
            self.get_parameter("permit_timeout_s").value))
        permit_fresh = time.monotonic() - self._permit_time <= permit_timeout
        speed_scale = self.speed_scale if permit_fresh else 0.0
        w = 0.0
        scale = max(0.0, min(1.0, speed_scale))
        if self.path_pts and self.path_i < len(self.path_pts):
            tx, ty = self.path_pts[self.path_i]
            dx, dy = tx - self.x, ty - self.y
            d = math.hypot(dx, dy)
            # Like Nav2: waypoints are passed loosely, the goal is finished
            # precisely (slow approach), then the robot turns on the spot to
            # the goal heading (RPP rotate-to-heading).
            last = self.path_i == len(self.path_pts) - 1
            if d < (0.02 if last else 0.12):
                self.path_i += 1
            else:
                self.theta = math.atan2(dy, dx)
                v = min(self.v_nom * scale, d / self.dt if last else 1e9)
                if last:
                    v = min(v, max(0.08, d))      # approach slow-down
                self.x += v * math.cos(self.theta) * self.dt
                self.y += v * math.sin(self.theta) * self.dt
        elif self.path_pts and self.final_yaw is not None:
            err = math.atan2(math.sin(self.final_yaw - self.theta),
                             math.cos(self.final_yaw - self.theta))
            if abs(err) > 0.01:
                w = max(-1.0, min(1.0, 2.0 * err)) * scale
                self.theta += w * self.dt
        self.w = w

        now = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = f"{self.robot_id}/{self.get_parameter('odom_frame').value}"
        od.child_frame_id = f"{self.robot_id}/{self.get_parameter('base_frame').value}"
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = yaw_to_quat(self.theta)
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        self.pub_odom.publish(od)

        n = int(self.get_parameter("scan_samples").value)
        rmax = float(self.get_parameter("scan_range_max").value)
        sc = LaserScan()
        sc.header.stamp = now
        sc.header.frame_id = f"{self.robot_id}/laser_link"
        sc.angle_min = -math.pi
        sc.angle_max = math.pi
        sc.angle_increment = (2.0 * math.pi) / n
        sc.range_min = 0.12
        sc.range_max = rmax
        # Empty world: every ray reads max range, so the obstacle detector
        # correctly reports nothing. Inject blockages with `ros2 topic pub`
        # on /fleet/map_update to test rerouting.
        sc.ranges = [rmax - 0.01] * n
        self.pub_scan.publish(sc)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
