#!/usr/bin/env python3
"""
task_generator_node - an EVENT SOURCE, not a decision maker.

IS THIS A CENTRAL SERVER? No, and the distinction matters.

This node announces WHAT work exists. It never decides WHO does it - that is
settled by the auctioneer-free auction running independently on every robot.
Kill this node mid-run and the fleet keeps executing everything already
announced, keeps coordinating, keeps resolving conflicts. It is equivalent to
a warehouse order feed or a barcode scanner at the loading dock.

For the SIH demo you can also run it from any laptop, or replace it with
`ros2 topic pub` by hand, or let a robot announce a task it discovered. None of
those changes require touching robot code.
"""
import random
import time

import rclpy
from rclpy.node import Node

from amr_description.msg import Task as RosTask
from amr_fleet.nodes.qos_profiles import COORD_QOS


class TaskGeneratorNode(Node):

    def __init__(self):
        super().__init__("task_generator")

        self.declare_parameter("interval_s", 15.0)
        self.declare_parameter("num_tasks", 20)
        self.declare_parameter("seed", 42)
        # Flat [r,c,r,c,...] because ROS parameters do not support nested lists.
        self.declare_parameter("pickup_cells", [89, 26, 89, 93, 74, 26])
        self.declare_parameter("dropoff_cells", [22, 30, 22, 60, 22, 90])
        self.declare_parameter("priority_weights", [0.5, 0.3, 0.15, 0.05])

        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.max_tasks = int(self.get_parameter("num_tasks").value)
        self.count = 0

        self.pickups = self._pairs(self.get_parameter("pickup_cells").value)
        self.dropoffs = self._pairs(self.get_parameter("dropoff_cells").value)
        if not self.pickups or not self.dropoffs:
            self.get_logger().fatal("pickup_cells/dropoff_cells are empty")
            raise SystemExit(2)

        self.pub = self.create_publisher(RosTask, "/fleet/task_announce", COORD_QOS)
        interval = float(self.get_parameter("interval_s").value)
        self.create_timer(interval, self.emit)
        self.get_logger().info(
            f"task_generator: {self.max_tasks} tasks every {interval}s "
            f"seed={self.get_parameter('seed').value} "
            f"(seeded => reproducible experiments)")

    @staticmethod
    def _pairs(flat):
        return [(int(flat[i]), int(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]

    def emit(self):
        if self.count >= self.max_tasks:
            return
        self.count += 1
        pk = self.rng.choice(self.pickups)
        dp = self.rng.choice(self.dropoffs)
        w = list(self.get_parameter("priority_weights").value)
        prio = self.rng.choices([0, 1, 2, 3], weights=w, k=1)[0]

        m = RosTask()
        m.header.stamp = self.get_clock().now().to_msg()
        m.task_id = f"T-{self.count:04d}"
        m.pickup_row, m.pickup_col = pk
        m.dropoff_row, m.dropoff_col = dp
        m.priority = prio
        m.created_at = self.get_clock().now().nanoseconds * 1e-9
        m.deadline = 0.0
        m.announcer_id = "task_generator"
        self.pub.publish(m)
        self.get_logger().info(
            f"announced {m.task_id} pickup={pk} dropoff={dp} prio={prio}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TaskGeneratorNode()
    except SystemExit:
        rclpy.shutdown()
        return
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
