"""Relabel the front lidar's scan into ascending-CCW order, with extras.

Webots could not be coaxed into one honest 360-degree scan on this install:
a 360-degree 'fixed' lidar returns fabricated content outside ~100 degrees
(the source of the phantom-wall storms), 'rotating' lidars phase-roll
against Ros2Lidar's static labels, and multi-lidar robots corrupt the side
sensors' buffers. A single 180-degree 'fixed' front lidar IS honest
(95-98% of beams within 0.3 m of ground truth at spawn for all three robots,
lidar_probe.py, 2026-10-04) — so that is the contract: <ns>/scan is a
180-degree forward fan (360 beams, 0.5 deg). AMCL, the costmaps and the
fleet's obstacle detector are all FOV/beam-count agnostic, and the robots
never reverse (Regulated Pure Pursuit). Do not subscribe to the lidar's
point cloud: enabling it crashed Webots on this setup.

This relay converts Ros2Lidar's clockwise labels (angle_min=+fov/2, negative
increment) to the conventional ascending form, applies a 3-beam median
against rack-edge flicker, and projects every 2nd beam through the robot's
AMCL pose as a map-frame Marker on /fleet/scan_points for the fleet RViz
view (which needs no per-robot TF).
"""

import math

import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

COLORS = {
    'robot_1': (0.95, 0.25, 0.2),
    'robot_2': (0.2, 0.8, 0.3),
    'robot_3': (0.25, 0.5, 0.95),
}


class ScanFlip(Node):
    def __init__(self):
        super().__init__('scan_flip')
        self.__pub = self.create_publisher(LaserScan, 'scan',
                                           qos_profile_sensor_data)
        self.__marker_pub = self.create_publisher(Marker, '/fleet/scan_points',
                                                  qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_f_raw', self.__on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self.__on_pose, 10)
        self.__pose = None
        ns = self.get_namespace().strip('/')
        self.__name = ns or 'robot'
        self.__rgb = COLORS.get(self.__name, (0.9, 0.9, 0.2))

    def __on_pose(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.__pose = (p.position.x, p.position.y, yaw)

    def __on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        out = LaserScan()
        out.header = msg.header
        if msg.angle_increment < 0.0:
            # Ros2Lidar labels clockwise from +fov/2; flip to ascending.
            out.angle_min = msg.angle_min + msg.angle_increment * (n - 1)
            out.angle_increment = -msg.angle_increment
            rev = msg.ranges[::-1]
        else:
            out.angle_min = msg.angle_min
            out.angle_increment = msg.angle_increment
            rev = list(msg.ranges)
        out.angle_max = out.angle_min + out.angle_increment * (n - 1)
        out.time_increment = 0.0
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, 0.12)
        out.range_max = min(msg.range_max or 8.0, 8.0)
        # 3-beam median (non-circular: this is a 90-degree fan, the ends are
        # real edges) against single-beam rack-edge flicker.
        med = list(rev)
        for k in range(1, n - 1):
            med[k] = sorted((rev[k - 1], rev[k], rev[k + 1]))[1]
        out.ranges = med
        self.__pub.publish(out)
        self.__publish_marker(out)

    def __publish_marker(self, scan):
        if self.__pose is None:
            return
        x0, y0, yaw = self.__pose
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = scan.header.stamp
        marker.ns = self.__name
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.color.r, marker.color.g, marker.color.b = self.__rgb
        marker.color.a = 0.9
        marker.lifetime.nanosec = 500000000
        for i in range(0, len(scan.ranges), 2):
            r = scan.ranges[i]
            if scan.range_min < r < scan.range_max:
                a = yaw + scan.angle_min + i * scan.angle_increment
                marker.points.append(Point(x=x0 + r * math.cos(a),
                                           y=y0 + r * math.sin(a), z=0.2))
        self.__marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFlip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
