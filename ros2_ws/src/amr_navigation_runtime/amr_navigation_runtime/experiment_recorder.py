"""Record end-to-end task completion time and robot-contact episodes."""

import csv
import os

import rclpy
from amr_description.msg import Task, TaskComplete
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts

CONTACT_SUFFIXES = ('contacts', 'contacts/wheel_left',
                    'contacts/wheel_right', 'contacts/caster')


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__('experiment_recorder')
        self.declare_parameter('pair_id', 'seed_42')
        self.declare_parameter('coordination_mode', 'proposed')
        self.declare_parameter('expected_tasks', 20)
        self.declare_parameter('robot_ids', ['robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('output_file', 'benchmark_runs.csv')
        self.declare_parameter('contact_episode_gap_s', 1.0)
        self.declare_parameter('contact_settle_s', 1.5)
        self._pair_id = str(self.get_parameter('pair_id').value)
        self._mode = str(self.get_parameter('coordination_mode').value)
        self._expected = int(self.get_parameter('expected_tasks').value)
        self._robot_ids = [str(r) for r in self.get_parameter('robot_ids').value]
        self._announced = {}
        self._completed = {}
        self._last_contact = {}
        self._collision_episodes = 0
        self._last_contact_time = None
        self._written = False
        self._first_task_time = None
        self.create_subscription(Task, '/fleet/task_announce',
                                 self._on_task, 10)
        self.create_subscription(TaskComplete, '/fleet/task_complete',
                                 self._on_complete, 10)
        for rid in self._robot_ids:
            for suffix in CONTACT_SUFFIXES:
                self.create_subscription(
                    Contacts, f'/{rid}/{suffix}',
                    lambda message: self._on_contacts(message), 10)
        self.create_timer(0.25, self._finish_if_complete)
        output = os.path.abspath(
            str(self.get_parameter('output_file').value))
        os.makedirs(os.path.dirname(output), exist_ok=True)
        self._stream = open(output, 'a', newline='', encoding='utf-8')
        self._writer = csv.writer(self._stream)
        if os.path.getsize(output) == 0:
            self._writer.writerow([
                'pair_id', 'mode', 'elapsed_s', 'completed_tasks',
                'expected_tasks', 'collisions'])
            self._stream.flush()
        self.get_logger().info(f'writing experiment summary rows to {output}')

    @staticmethod
    def _stamp(message, field):
        value = float(getattr(message, field))
        if value > 0.0:
            return value
        stamp = message.header.stamp
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_task(self, message):
        stamp = self._stamp(message, 'created_at')
        self._announced.setdefault(message.task_id, stamp)
        if self._first_task_time is None or stamp < self._first_task_time:
            self._first_task_time = stamp
        self._finish_if_complete()

    def _on_complete(self, message):
        stamp = self._stamp(message, 'stamp')
        self._completed.setdefault(message.task_id, stamp)
        self._finish_if_complete()

    def _on_contacts(self, message):
        stamp = message.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        if now <= 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
        gap = float(self.get_parameter('contact_episode_gap_s').value)
        for contact in message.contacts:
            first = str(contact.collision1.name)
            second = str(contact.collision2.name)
            robot_a = next((rid for rid in self._robot_ids if rid in first), None)
            robot_b = next((rid for rid in self._robot_ids if rid in second), None)
            if not robot_a or not robot_b or robot_a == robot_b:
                continue
            pair = tuple(sorted((robot_a, robot_b)))
            if self._last_contact_time is None or now > self._last_contact_time:
                self._last_contact_time = now
            previous = self._last_contact.get(pair)
            self._last_contact[pair] = now
            if previous is None or now - previous >= gap:
                self._collision_episodes += 1
                self.get_logger().error(
                    f'inter-robot contact episode {pair[0]} / {pair[1]} '
                    f'at simulation time {now:.3f}s')

    def _finish_if_complete(self):
        if self._written or self._expected <= 0:
            return
        if (len(self._announced) >= self._expected and
                len(self._completed) >= self._expected):
            finish_time = max(self._completed.values())
            latest_event = max(finish_time, self._last_contact_time or finish_time)
            quiet_s = max(0.0, float(
                self.get_parameter('contact_settle_s').value))
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - latest_event >= quiet_s:
                self._write_summary(finish_time)

    def _write_summary(self, finish_time):
        if self._written or self._first_task_time is None:
            return
        self._writer.writerow([
            self._pair_id, self._mode,
            f'{max(0.0, finish_time - self._first_task_time):.6f}',
            len(self._completed), self._expected, self._collision_episodes,
        ])
        self._stream.flush()
        self._written = True
        self.get_logger().info(
            f'completed {len(self._completed)}/{self._expected} tasks in '
            f'{finish_time - self._first_task_time:.2f}s; '
            f'inter-robot contact episodes={self._collision_episodes}')

    def destroy_node(self):
        if not self._written and self._first_task_time is not None:
            complete = (len(self._announced) >= self._expected > 0 and
                        len(self._completed) >= self._expected)
            finish_time = (max(self._completed.values()) if complete else
                           self.get_clock().now().nanoseconds * 1e-9)
            self._write_summary(finish_time)
        if not self._stream.closed:
            self._stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRecorder()
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
