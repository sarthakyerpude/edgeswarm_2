#!/usr/bin/env python3
"""
rack_labels_node - draws the R8 rack slot names in RViz.

Publishes a latched (transient-local) visualization_msgs/MarkerArray on
/fleet/rack_labels in frame 'map':
  * ns 'ticks'    : a line along every rack face plus a tick at every section
                    boundary (21 per face), pointing into the aisle;
  * ns 'sides'    : the side number ("S1".."S12") written on the half of the
                    rack top that belongs to that face;
  * ns 'sections' : section numbers just outside the face, every
                    `section_label_step` sections plus both ends (1 and 20).
Geometry comes from warehouse_grid.yaml 'racks:' via amr_fleet.core.rack_slots,
so the labels can never disagree with the slot table the fleet uses.
"""
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from amr_fleet.core.rack_slots import SECTIONS_PER_FACE, SIDES, RackSlots
from amr_fleet.nodes.qos_profiles import PATH_QOS  # reliable, latched, depth 1


def _default_grid_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("amr_description"),
                            "config", "warehouse_grid.yaml")
    except Exception:   # noqa: BLE001 - source-tree fallback
        return os.path.join(os.path.dirname(__file__), "..", "..", "config",
                            "warehouse_grid.yaml")


class RackLabelsNode(Node):

    def __init__(self):
        super().__init__("rack_labels")
        self.declare_parameter("grid_yaml", _default_grid_yaml())
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("section_label_step", 1)   # 5 -> 1,5,10,15,20
        self.declare_parameter("rack_height_m", 1.6)
        self.declare_parameter("republish_s", 5.0)
        path = str(self.get_parameter("grid_yaml").value)
        self.slots = RackSlots.from_yaml(path)
        self.pub = self.create_publisher(MarkerArray, "/fleet/rack_labels", PATH_QOS)
        self.msg = self._build()
        self.pub.publish(self.msg)
        period = float(self.get_parameter("republish_s").value)
        if period > 0:
            self.create_timer(period, lambda: self.pub.publish(self.msg))
        self.get_logger().info(
            f"rack_labels: {len(self.msg.markers)} markers, W="
            f"{self.slots.aisle_width_m} m, from {path}")

    def _marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = str(self.get_parameter("frame_id").value)
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.frame_locked = True
        return m

    @staticmethod
    def _rgba(m, r, g, b, a=1.0):
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a

    def _text(self, ns, mid, text, x, y, z, h, rgb):
        t = self._marker(ns, mid, Marker.TEXT_VIEW_FACING)
        t.text = text
        t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, z
        t.scale.z = h
        self._rgba(t, *rgb)
        return t

    def _build(self) -> MarkerArray:
        rs = self.slots
        step = max(1, int(self.get_parameter("section_label_step").value))
        top = float(self.get_parameter("rack_height_m").value) + 0.05
        tick_len = min(0.15, rs.approach_offset_m * 0.5)
        out = MarkerArray()
        clear = self._marker("", 0, Marker.DELETEALL)
        out.markers.append(clear)
        ticks = self._marker("ticks", 0, Marker.LINE_LIST)
        ticks.scale.x = 0.02
        self._rgba(ticks, 1.0, 0.85, 0.1)
        for side in sorted(SIDES):
            rack_id, face = SIDES[side]
            r = rs.racks[rack_id]
            fy = rs.face_y(side)
            sign = 1.0 if face == "N" else -1.0
            L = rs.section_length(rack_id)
            ticks.points += [Point(x=r.x_min, y=fy, z=0.02),
                             Point(x=r.x_max, y=fy, z=0.02)]
            for i in range(SECTIONS_PER_FACE + 1):
                x = r.x_min + i * L
                ticks.points += [Point(x=x, y=fy, z=0.02),
                                 Point(x=x, y=fy + sign * tick_len, z=0.02)]
            # Side number on this face's half of the rack top.
            out.markers.append(self._text(
                "sides", side, f"S{side}", 0.5 * (r.x_min + r.x_max),
                fy - sign * 0.15, top, 0.24, (1.0, 1.0, 1.0)))
            for n in range(1, SECTIONS_PER_FACE + 1):
                if n % step and n not in (1, SECTIONS_PER_FACE):
                    continue
                s = rs.slot(f"{side}R{n}")
                out.markers.append(self._text(
                    "sections", side * 100 + n, str(n), s.centre[0],
                    fy + sign * (tick_len * 0.5 + 0.03), 0.05, 0.11,
                    (0.55, 0.9, 1.0)))
        out.markers.append(ticks)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = RackLabelsNode()
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
