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

from amr_description.msg import (NavigationStatus, RobotState, Task as RosTask,
                                 TaskComplete as RosTaskComplete)
from amr_fleet.nodes.qos_profiles import (COORD_QOS, NAV_STATUS_QOS,
                                          STATE_QOS)

try:    # Generated only after the next colcon build (R6, msg/TaskCancel.msg).
    from amr_description.msg import TaskCancel as RosTaskCancel
except ImportError:                      # pragma: no cover - stale install
    RosTaskCancel = None


class TaskGeneratorNode(Node):

    def __init__(self):
        super().__init__("task_generator")

        self.declare_parameter("interval_s", 15.0)
        self.declare_parameter("num_tasks", 20)
        self.declare_parameter("seed", 42)
        self.declare_parameter("expected_robot_ids", [''])
        self.declare_parameter("readiness_timeout_s", 1.0)
        # Flat [r,c,r,c,...] because ROS parameters do not support nested lists.
        # R7 layout: L1/L2/L3 = approach cells of rack slots 1R10, 3R11, 2R10.
        self.declare_parameter("pickup_cells", [88, 25, 88, 94, 74, 25])
        self.declare_parameter("dropoff_cells", [22, 30, 22, 60, 22, 90])
        self.declare_parameter("priority_weights", [0.5, 0.3, 0.15, 0.05])
        # R5 aging budgets, seconds, indexed by Task.priority 0..3: the task
        # deadline is created_at + deadline_budget_s[priority]. Calibrated
        # against the measured single-robot cycle (64.8 s mean / 74.4 s max
        # on the R7 layout): even the priority-3 budget stays above physics.
        self.declare_parameter("deadline_budget_s", [225.0, 188.0, 150.0, 113.0])
        # R8 rack slots, e.g. ['1R6', '12R5']. When set, pickups are the
        # slots' approach cells (core/rack_slots.py) instead of pickup_cells.
        self.declare_parameter("pickup_slots", [''])
        self.declare_parameter("grid_yaml", '')

        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.max_tasks = int(self.get_parameter("num_tasks").value)
        self.count = 0
        self.expected_robot_ids = {
            str(robot_id) for robot_id in
            self.get_parameter("expected_robot_ids").value
            if str(robot_id).strip()
        }
        self.readiness_timeout_s = max(
            0.0, float(self.get_parameter("readiness_timeout_s").value))
        self._robot_state = {}
        self._navigation_ready = {}
        self._readiness_logged = False

        self.pickups = self._pairs(self.get_parameter("pickup_cells").value)
        slot_labels = [str(s).strip() for s in
                       self.get_parameter("pickup_slots").value if str(s).strip()]
        if slot_labels:
            self.pickups = self._slot_cells(slot_labels)
        self.dropoffs = self._pairs(self.get_parameter("dropoff_cells").value)
        if not self.pickups or not self.dropoffs:
            self.get_logger().fatal("pickup_cells/dropoff_cells are empty")
            raise SystemExit(2)

        # Dedup: never keep two live tasks on one pickup cell — concurrent
        # winners otherwise converge head-on into the single cross-corridor
        # (the reproducer for the fleet livelock). task_id -> (pickup, t).
        self._active = {}
        self.pub = self.create_publisher(RosTask, "/fleet/task_announce", COORD_QOS)
        self.create_subscription(
            RosTaskComplete, "/fleet/task_complete",
            lambda m: self._active.pop(m.task_id, None), COORD_QOS)
        # R6: an aborted task frees its pickup for new work immediately.
        if RosTaskCancel is not None:
            self.create_subscription(RosTaskCancel, "/fleet/task_cancel",
                                     self._on_cancel, COORD_QOS)
        else:
            self.get_logger().warning(
                "TaskCancel msg not installed (stale build): task aborts "
                "will not free their pickup dedup entries")
        if self.expected_robot_ids:
            self.create_subscription(RobotState, "/fleet/robot_state",
                                     self._on_robot_state, STATE_QOS)
            self.create_subscription(
                NavigationStatus, "/fleet/navigation_status",
                self._on_navigation_ready, NAV_STATUS_QOS)
        interval = float(self.get_parameter("interval_s").value)
        self.create_timer(interval, self.emit)
        self.get_logger().info(
            f"task_generator: {self.max_tasks} tasks every {interval}s "
            f"seed={self.get_parameter('seed').value} "
            f"(seeded => reproducible experiments)")

    def _slot_cells(self, labels):
        import os
        from amr_fleet.core.rack_slots import RackSlots
        path = str(self.get_parameter("grid_yaml").value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("amr_description"),
                                "config", "warehouse_grid.yaml")
        slots = RackSlots.from_yaml(path)
        cells = []
        for label in labels:
            try:
                cell = slots.approach_cell(label)
            except ValueError as exc:
                self.get_logger().fatal(f"pickup_slots: {exc}")
                raise SystemExit(2)
            if cell not in cells:
                cells.append(cell)
            self.get_logger().info(f"pickup slot {label} -> approach cell {cell}")
        return cells

    @staticmethod
    def _pairs(flat):
        return [(int(flat[i]), int(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]

    def emit(self):
        if self.count >= self.max_tasks:
            return
        waiting = self._robots_not_ready()
        if waiting:
            if not self._readiness_logged:
                self.get_logger().info(
                    "waiting for fresh healthy state from: "
                    + ", ".join(sorted(waiting)))
                self._readiness_logged = True
            return
        if self._readiness_logged:
            self.get_logger().info("all configured robots are healthy; starting task feed")
            self._readiness_logged = False
        cutoff = time.monotonic() - 180.0
        self._active = {tid: rec for tid, rec in self._active.items()
                        if rec[1] >= cutoff}
        busy = {rec[0] for rec in self._active.values()}
        free = [p for p in self.pickups if p not in busy]
        if not free:
            return          # every pickup has a live task; try next tick
        self.count += 1
        pk = self.rng.choice(free)
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
        # R5: every task carries a time budget; as it is consumed the task's
        # urgency escalates fleet-wide (core/priority.urgency). Higher
        # priority = tighter budget.
        budgets = [float(b) for b in
                   self.get_parameter("deadline_budget_s").value]
        budget = budgets[min(prio, len(budgets) - 1)] if budgets else 0.0
        m.deadline = (m.created_at + budget) if budget > 0.0 else 0.0
        m.announcer_id = "task_generator"
        self.pub.publish(m)
        self._active[m.task_id] = (pk, time.monotonic())
        self.get_logger().info(
            f"announced {m.task_id} pickup={pk} dropoff={dp} prio={prio}")

    def _on_cancel(self, m):
        if self._active.pop(m.task_id, None) is not None:
            self.get_logger().info(
                f"task {m.task_id} cancelled by {m.requester_id or 'unknown'}"
                f" ({m.reason or 'no reason'}): pickup freed")

    def _on_robot_state(self, message):
        self._robot_state[message.robot_id] = (message, time.monotonic())

    def _on_navigation_ready(self, message):
        if message.robot_id in self.expected_robot_ids:
            self._navigation_ready[message.robot_id] = (
                bool(message.ready), time.monotonic())

    def _robots_not_ready(self):
        now = time.monotonic()
        unavailable = set()
        for robot_id in self.expected_robot_ids:
            record = self._robot_state.get(robot_id)
            if (record is None or not record[0].alive or
                    now - record[1] > self.readiness_timeout_s):
                unavailable.add(robot_id)
                continue
            navigation = self._navigation_ready.get(robot_id)
            if (navigation is None or not navigation[0] or
                    now - navigation[1] > self.readiness_timeout_s):
                unavailable.add(robot_id)
        return unavailable


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
