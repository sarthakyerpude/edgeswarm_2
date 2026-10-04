#!/usr/bin/env python3
"""
hitbox_markers_node - draws every robot's hitbox in RViz (R2).

ONE node for the whole fleet (NOT per-robot, NOT fleet_agent_node): it
subscribes to the shared /fleet/robot_state topic and publishes a
visualization_msgs/MarkerArray on /fleet/hitboxes at 5 Hz, frame 'map',
namespace = robot_id, marker lifetime 1.0 s (a robot that stops broadcasting
fades from the view within a second).

Per robot (ids within its namespace):
  0  body outline     LINE_STRIP, robot colour - the TRUE chassis polygon
                      with wheel bumps (what physically occupies the floor)
  1  FRONT EDGE       LINE_LIST, yellow, 0.06 wide - the heading is visible
                      at a glance from the top-down view
  2  heading triangle TRIANGLE_LIST, robot colour
  3  heading arrow    ARROW, centre -> +0.35 m
  4  SAFETY outline   LINE_LIST (dashed), alpha 0.45 - the rect inflated by
                      per_robot_inflation(loc_sigma_lat), drawn SEPARATELY
                      from the true body so uncertainty is never mistaken
                      for the robot itself
  5  label            TEXT_VIEW_FACING '<robot_id> L<priority_score> <status>'

All geometry comes from core/hitbox.marker_geometry, so RViz can never
disagree with what the safety envelope actually uses.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from amr_description.msg import RobotState
from amr_fleet.core import hitbox
from amr_fleet.nodes.qos_profiles import STATE_QOS

# Wire contract: /fleet/hitboxes is reliable, volatile, depth 5. Defined here
# (not in qos_profiles.py) because that file is owned by another session.
HITBOX_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

COLOURS = {
    "robot_1": (0.90, 0.20, 0.20),
    "robot_2": (0.20, 0.80, 0.20),
    "robot_3": (0.20, 0.40, 1.00),
}
DEFAULT_COLOUR = (0.75, 0.75, 0.75)
FRONT_YELLOW = (1.0, 0.85, 0.0)
STATUS_NAMES = {0: "IDLE", 1: "MOVING", 2: "WAITING", 3: "YIELDING",
                4: "CHARGING", 5: "FAULT"}
PUBLISH_HZ = 5.0
LIFETIME_S = 1.0
STALE_DROP_S = 3.0          # stop re-drawing a robot quiet this long
DASH_ON_M = 0.08
DASH_GAP_M = 0.06


class HitboxMarkersNode(Node):

    def __init__(self):
        super().__init__("hitbox_markers")
        self.declare_parameter("frame_id", "map")
        self.latest = {}                   # robot_id -> (msg, rx wall time)
        self.sub = self.create_subscription(
            RobotState, "/fleet/robot_state", self.cb_state, STATE_QOS)
        self.pub = self.create_publisher(MarkerArray, "/fleet/hitboxes",
                                         HITBOX_QOS)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish)
        self.get_logger().info("hitbox_markers: /fleet/robot_state -> "
                               "/fleet/hitboxes at 5 Hz")

    def cb_state(self, msg: RobotState) -> None:
        self.latest[msg.robot_id] = (msg, self.get_clock().now())

    # ------------------------------------------------------------- building
    def _marker(self, ns: str, mid: int, mtype: int) -> Marker:
        m = Marker()
        m.header.frame_id = str(self.get_parameter("frame_id").value)
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=LIFETIME_S).to_msg()
        m.frame_locked = True
        return m

    @staticmethod
    def _rgba(m: Marker, rgb, a: float = 1.0) -> None:
        m.color.r, m.color.g, m.color.b, m.color.a = (
            float(rgb[0]), float(rgb[1]), float(rgb[2]), float(a))

    @staticmethod
    def _pts(seq, z: float = 0.03):
        return [Point(x=float(x), y=float(y), z=z) for x, y in seq]

    @staticmethod
    def _dashes(polyline, on=DASH_ON_M, gap=DASH_GAP_M):
        """Split a closed polyline into dash segments for a LINE_LIST."""
        out = []
        period = on + gap
        for (x0, y0), (x1, y1) in zip(polyline, polyline[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1e-9:
                continue
            ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
            s = 0.0
            while s < seg:
                e = min(seg, s + on)
                out.append(((x0 + ux * s, y0 + uy * s),
                            (x0 + ux * e, y0 + uy * e)))
                s += period
        return out

    def _robot_markers(self, msg: RobotState):
        rid = msg.robot_id
        colour = COLOURS.get(rid, DEFAULT_COLOUR)
        geo = hitbox.marker_geometry(msg.pose.x, msg.pose.y, msg.pose.theta,
                                     msg.loc_sigma_lat)
        out = []

        body = self._marker(rid, 0, Marker.LINE_STRIP)
        body.scale.x = 0.02
        self._rgba(body, colour)
        body.points = self._pts(geo["body"])
        out.append(body)

        front = self._marker(rid, 1, Marker.LINE_LIST)
        front.scale.x = 0.06
        self._rgba(front, FRONT_YELLOW)
        front.points = self._pts(geo["front_edge"], z=0.05)
        out.append(front)

        nose = self._marker(rid, 2, Marker.TRIANGLE_LIST)
        nose.scale.x = nose.scale.y = nose.scale.z = 1.0
        self._rgba(nose, colour, 0.9)
        nose.points = self._pts(geo["nose"], z=0.04)
        out.append(nose)

        arrow = self._marker(rid, 3, Marker.ARROW)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.03, 0.07, 0.05
        self._rgba(arrow, colour)
        arrow.points = self._pts(geo["arrow"], z=0.06)
        out.append(arrow)

        safe = self._marker(rid, 4, Marker.LINE_LIST)
        safe.scale.x = 0.015
        self._rgba(safe, colour, 0.45)
        for a, b in self._dashes(geo["safety"]):
            safe.points += [Point(x=a[0], y=a[1], z=0.02),
                            Point(x=b[0], y=b[1], z=0.02)]
        out.append(safe)

        label = self._marker(rid, 5, Marker.TEXT_VIEW_FACING)
        status = STATUS_NAMES.get(int(msg.status), str(int(msg.status)))
        label.text = f"{rid} L{int(msg.priority_score)} {status}"
        label.pose.position.x = float(msg.pose.x)
        label.pose.position.y = float(msg.pose.y)
        label.pose.position.z = 0.55
        label.scale.z = 0.14
        self._rgba(label, (1.0, 1.0, 1.0))
        out.append(label)
        return out

    def publish(self) -> None:
        now = self.get_clock().now()
        arr = MarkerArray()
        for rid, (msg, rx) in sorted(self.latest.items()):
            if (now - rx).nanoseconds * 1e-9 > STALE_DROP_S:
                continue
            arr.markers += self._robot_markers(msg)
        if arr.markers:
            self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = HitboxMarkersNode()
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
