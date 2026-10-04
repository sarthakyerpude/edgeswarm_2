"""Record raw robot-link contact episodes and inter-robot safety episodes."""

import csv
import os

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts

CONTACT_SUFFIXES = ('contacts', 'contacts/wheel_left',
                    'contacts/wheel_right', 'contacts/caster')


def robot_from_collision(name, robot_ids):
    """Match a scoped Gazebo collision name to a whole model ID."""
    for robot_id in sorted(robot_ids, key=len, reverse=True):
        if (name.startswith(f'{robot_id}::') or
                f'::{robot_id}::' in name):
            return robot_id
    return None


class CollisionRecorder(Node):
    def __init__(self):
        super().__init__('collision_recorder')
        self.declare_parameter('robot_ids', ['robot_1', 'robot_2', 'robot_3'])
        self.declare_parameter('output_file', 'inter_robot_contacts.csv')
        self.declare_parameter('events_output_file', 'contact_sensor_events.csv')
        self.declare_parameter('episode_gap_s', 1.0)
        self._robot_ids = [str(r) for r in self.get_parameter('robot_ids').value]
        output = os.path.abspath(str(self.get_parameter('output_file').value))
        events_output = os.path.abspath(
            str(self.get_parameter('events_output_file').value))
        os.makedirs(os.path.dirname(output), exist_ok=True)
        os.makedirs(os.path.dirname(events_output), exist_ok=True)
        self._stream = open(output, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._stream)
        self._writer.writerow([
            'simulation_time_s', 'robot_a', 'robot_b', 'collision_a', 'collision_b'])
        self._stream.flush()
        self._events_stream = open(events_output, 'w', newline='', encoding='utf-8')
        self._events_writer = csv.writer(self._events_stream)
        self._events_writer.writerow([
            'simulation_time_s', 'robot_sensor', 'link_sensor',
            'collision1', 'collision2', 'inter_robot'])
        self._events_stream.flush()
        self._last_seen = {}
        self._last_event_seen = {}
        for robot_id in self._robot_ids:
            for suffix in CONTACT_SUFFIXES:
                self.create_subscription(
                    Contacts, f'/{robot_id}/{suffix}',
                    lambda message, owner=robot_id, link=suffix:
                        self._on_contacts(owner, link, message), 10)
        self.get_logger().info(f'recording inter-robot contacts to {output}')
        self.get_logger().info(f'recording robot-link contact events to {events_output}')

    def _on_contacts(self, owner, link, message):
        now = (message.header.stamp.sec
               + message.header.stamp.nanosec * 1e-9)
        if now <= 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
        gap = float(self.get_parameter('episode_gap_s').value)
        for contact in message.contacts:
            first = str(contact.collision1.name)
            second = str(contact.collision2.name)
            first_robot = robot_from_collision(first, self._robot_ids)
            second_robot = robot_from_collision(second, self._robot_ids)
            pair_key = tuple(sorted((first, second)))
            event_key = (owner, link, pair_key)
            event_previous = self._last_event_seen.get(event_key)
            self._last_event_seen[event_key] = now
            if event_previous is None or now - event_previous >= gap:
                self._events_writer.writerow([
                    f'{now:.6f}', owner, link, first, second,
                    int(bool(first_robot and second_robot and
                             first_robot != second_robot))])
                self._events_stream.flush()
            if not first_robot or not second_robot or first_robot == second_robot:
                continue
            pair = tuple(sorted((first_robot, second_robot)))
            previous = self._last_seen.get(pair)
            self._last_seen[pair] = now
            if previous is not None and now - previous < gap:
                continue
            if first_robot == pair[0]:
                collision_a, collision_b = first, second
            else:
                collision_a, collision_b = second, first
            self._writer.writerow([
                f'{now:.6f}', pair[0], pair[1], collision_a, collision_b])
            self._stream.flush()
            self.get_logger().error(
                f'inter-robot contact #{pair[0]}:{pair[1]} at {now:.3f}s')

    def destroy_node(self):
        if not self._stream.closed:
            self._stream.close()
        if not self._events_stream.closed:
            self._events_stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionRecorder()
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
