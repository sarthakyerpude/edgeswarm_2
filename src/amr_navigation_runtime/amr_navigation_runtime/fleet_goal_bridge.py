"""Translate each robot's decentralized coordination path into a Nav2 goal.

The fleet agent remains responsible for task allocation and route planning.
Nav2 remains responsible for local obstacle avoidance and wheel commands. This
node connects the two: each changed coordination route is sent to the
namespaced ``FollowPath`` controller action so route reservations affect motion.
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import BackUp, FollowPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from amr_description.msg import NavigationStatus, RobotState
from amr_fleet.nodes.qos_profiles import NAV_STATUS_QOS, PATH_QOS, STATE_QOS

from amr_navigation_runtime.path_trim import trim_to_robot


DOCK_MATCH_M = 0.03
POSE_FRESH_S = 1.0     # robot_state older than this: send the route as is
# Wedge recovery: FollowPath is called directly (no BT), so nothing backs a
# robot off a rack corner it swung into - it aborts 'Failed to make
# progress' every ~10 s forever (robot_3, 2026-10-04). After this many
# consecutive no-progress aborts, back up a little before retrying.
WEDGE_ABORTS = 2
WEDGE_MOVED_M = 0.05     # moved less than this during the goal = wedged
BACKUP_M, BACKUP_SPEED, BACKUP_TIME_S = 0.2, 0.1, 6.0
MAX_BACKUPS_PER_ROUTE = 3

# RECTILINEAR TURN MODEL (user directive, 2026-10-04): "move straight until
# the turn point, then spin 90 deg in place, then move straight again,
# instead of taking an arc turn." The bridge SPLITS every coordination route
# into straight segments at cumulative heading changes >= TURN_SPLIT_RAD
# (~60 deg: junction turns are 90; anything smaller - the staircase 45 deg
# lane-merge jogs - stays one smooth segment) and sends them as SEQUENTIAL
# FollowPath goals. Each segment's final pose carries the NEXT segment's
# heading, so RPP (use_rotate_to_heading + the SimpleGoalChecker yaw
# tolerance) finishes the segment by spinning in place at the turn point to
# the new heading; the next goal then starts aligned and drives pure-straight.
# The planner guarantees every such turn point has >= ~0.45 m of static
# clearance (astar turn shaping), the collision monitor's rotation band
# legalizes the spin, and the swept-cells tests verify it offline. After a
# BackUp the current segment is re-sent trimmed to the robot: a straight
# segment + rotate-to-heading IS the rotate-first re-approach, so the old
# "BackUp then re-drive the same arc" loop cannot recur. The final pose of
# the LAST segment keeps the route's own yaw (dock alignment contract).
TURN_SPLIT_RAD = 1.047


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def split_rectilinear(pts, final_yaw=None, split_rad=TURN_SPLIT_RAD):
    """[(x, y)] -> [[(x, y, yaw)], ...]: straight segments split where a
    step heading differs from the SEGMENT's reference (first-step) heading
    by >= split_rad, so a square 90 splits, a 45+45 decomposed corner splits
    at its second 45 (cumulative 90), and an alternating 45 staircase stays
    one segment. Every pose carries its outgoing step heading; the last pose
    of a segment carries the NEXT segment's first heading (the spin target),
    and the route's last pose carries final_yaw when given (dock yaw)."""
    clean = []
    for p in pts or []:
        if not clean or math.hypot(p[0] - clean[-1][0],
                                   p[1] - clean[-1][1]) > 1e-9:
            clean.append((float(p[0]), float(p[1])))
    if not clean:
        return []
    if len(clean) == 1:
        return [[(clean[0][0], clean[0][1],
                  final_yaw if final_yaw is not None else 0.0)]]
    heads = [math.atan2(b[1] - a[1], b[0] - a[0])
             for a, b in zip(clean, clean[1:])]
    bounds = [0]
    ref = heads[0]
    for i, h in enumerate(heads):
        if i > bounds[-1] and abs(_wrap(h - ref)) >= split_rad:
            bounds.append(i)        # the corner pose index
            ref = h
    bounds.append(len(clean) - 1)
    last_yaw = final_yaw if final_yaw is not None else heads[-1]
    segs = []
    for k in range(len(bounds) - 1):
        i0, i1 = bounds[k], bounds[k + 1]
        seg = []
        for g in range(i0, i1 + 1):
            if g == len(clean) - 1:
                yaw = last_yaw                      # the route's own end yaw
            elif g == i1:
                yaw = heads[i1]                     # spin target: next leg
            else:
                yaw = heads[g]
            seg.append((clean[g][0], clean[g][1], yaw))
        segs.append(seg)
    return segs


class FleetGoalBridge(Node):
    def __init__(self):
        super().__init__('fleet_goal_bridge')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        self.declare_parameter('progress_checker_id', 'progress_checker')
        self.declare_parameter('robot_id', '')
        # Dock (= spawn) pose. A path ending within DOCK_MATCH_M of it is a
        # docking approach and gets the precise goal checker.
        self.declare_parameter('dock_x', float('nan'))
        self.declare_parameter('dock_y', float('nan'))
        self.declare_parameter('dock_goal_checker_id', 'precise_goal_checker')
        self._client = ActionClient(self, FollowPath, 'follow_path')
        self._backup_client = ActionClient(self, BackUp, 'backup')
        self._wedged_aborts = 0
        self._backups_for_key = 0
        self._backing_up = False
        self._goal_start_xy = None
        self._sent_final_segment = True
        self._sent_seg_end = None
        self._resume_xy = None          # last completed segment's end
        self._desired = None
        self._desired_key = None
        self._active_key = None
        self._completed_key = None
        self._retry_after = 0.0
        self._goal_handle = None
        self._request_pending = False
        self._cancel_pending = False
        self._ready_pub = self.create_publisher(
            NavigationStatus, '/fleet/navigation_status', NAV_STATUS_QOS)
        self.create_subscription(Path, 'coordination_path', self._on_path,
                                 PATH_QOS)
        # Own map-frame pose from the fleet heartbeat (10 Hz), used to start
        # every goal where the robot is - see path_trim.py.
        self._robot_xy = None
        self._robot_status = ''
        self._robot_xy_t = 0.0
        self.create_subscription(RobotState, '/fleet/robot_state',
                                 self._on_state, STATE_QOS)
        self.create_timer(0.25, self._drive_action_state)
        self.create_timer(0.5, self._publish_readiness)

    def _publish_readiness(self):
        message = NavigationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.robot_id = str(self.get_parameter('robot_id').value).strip('/')
        message.ready = self._client.server_is_ready()
        self._ready_pub.publish(message)

    def _on_state(self, msg):
        if msg.robot_id == str(self.get_parameter('robot_id').value).strip('/'):
            self._robot_xy = (msg.pose.x, msg.pose.y)
            self._robot_status = msg.status
            self._robot_xy_t = time.monotonic()

    def _goal_path(self, route):
        """The route from the pose nearest the robot (or from the robot, if
        it is off the route). Never shortens the final pose away.

        With a STALE robot_state the reference falls back to the end of the
        last completed segment (_resume_xy): the rectilinear sequencer
        relies on this trim to advance to the next leg, and without the
        fallback a heartbeat gap > POSE_FRESH_S re-sent the already-driven
        first leg in a loop until the pose returned."""
        fresh = (self._robot_xy is not None
                 and time.monotonic() - self._robot_xy_t <= POSE_FRESH_S)
        ref = self._robot_xy if fresh else self._resume_xy
        start, prepend = trim_to_robot(
            [(p.pose.position.x, p.pose.position.y) for p in route.poses],
            ref)
        if start == 0 and not prepend:
            return route, 0
        out = Path()
        out.header = route.header
        out.poses = list(route.poses[start:])
        if prepend:
            here = PoseStamped()
            here.header = route.header
            here.pose.position.x, here.pose.position.y = ref
            nxt = out.poses[0].pose.position
            yaw = math.atan2(nxt.y - here.pose.position.y,
                             nxt.x - here.pose.position.x)
            here.pose.orientation.z = math.sin(yaw * 0.5)
            here.pose.orientation.w = math.cos(yaw * 0.5)
            out.poses.insert(0, here)
        return out, start

    def _on_path(self, path):
        if not path.poses:
            self._desired = None
            self._desired_key = None
            return

        route = Path()
        route.header = path.header
        route.poses = list(path.poses)
        # The final heading is part of the goal: a docking path that only
        # changes its final yaw must still be re-sent.
        last = route.poses[-1].pose.orientation
        key = (route.header.frame_id, tuple(
            (round(p.pose.position.x, 2), round(p.pose.position.y, 2))
            for p in route.poses),
            round(2.0 * math.atan2(last.z, last.w), 2))
        if key != self._desired_key:
            self._backups_for_key = 0
            self._resume_xy = None
            self._desired = route
            self._desired_key = key
            self._completed_key = None
            self._retry_after = 0.0

    def _drive_action_state(self):
        if self._request_pending or self._cancel_pending or self._backing_up:
            return
        if (self._wedged_aborts >= WEDGE_ABORTS and self._goal_handle is None
                and self._backups_for_key < MAX_BACKUPS_PER_ROUTE
                and self._backup_client.server_is_ready()):
            self._start_backup()
            return

        if self._goal_handle is not None and self._active_key != self._desired_key:
            self._cancel_pending = True
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_cancelled)
            return

        if (self._desired is None or self._goal_handle is not None or
                self._desired_key == self._completed_key or
                time.monotonic() < self._retry_after):
            return
        if not self._client.server_is_ready():
            return

        self._goal_start_xy = self._robot_xy
        goal = FollowPath.Goal()
        trimmed, skipped = self._goal_path(self._desired)
        if skipped:
            self.get_logger().info(
                f'goal starts at the robot: skipped {skipped} passed poses')
        # Rectilinear turn model: send the FIRST straight segment only. On
        # its SUCCESS the robot stands at the turn point facing the next
        # leg's heading (the segment's final yaw + RPP rotate-to-heading);
        # the next timer tick re-trims the same desired route - which now
        # starts at the turn point - and sends the next leg. No per-segment
        # index is kept: trim-to-robot IS the sequencer.
        goal.path, remaining_turns = self._first_segment(trimmed)
        self._sent_final_segment = remaining_turns == 0
        if remaining_turns:
            self.get_logger().info(
                f'rectilinear: driving the next straight segment '
                f'({len(goal.path.poses)} poses, {remaining_turns} turn(s) '
                f'ahead on this route)')
        goal.controller_id = str(self.get_parameter('controller_id').value)
        goal.goal_checker_id = (self._goal_checker_for(self._desired)
                                if self._sent_final_segment
                                else str(self.get_parameter(
                                    'goal_checker_id').value))
        goal.progress_checker_id = str(
            self.get_parameter('progress_checker_id').value)
        end = goal.path.poses[-1].pose.position
        self._sent_seg_end = (end.x, end.y)
        self._request_pending = True
        request_key = self._desired_key
        self._client.send_goal_async(goal).add_done_callback(
            lambda future: self._on_goal_response(future, request_key))

    def _first_segment(self, path):
        """(the first straight segment of `path` as a Path with per-pose
        yaws, number of turns remaining after it). See split_rectilinear."""
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        lq = path.poses[-1].pose.orientation
        final_yaw = 2.0 * math.atan2(lq.z, lq.w)
        segs = split_rectilinear(pts, final_yaw=final_yaw)
        if not segs:
            return path, 0
        out = Path()
        out.header = path.header
        for (x, y, yaw) in segs[0]:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)
            out.poses.append(ps)
        return out, len(segs) - 1

    def _goal_checker_for(self, path):
        dx = float(self.get_parameter('dock_x').value)
        dy = float(self.get_parameter('dock_y').value)
        end = path.poses[-1].pose.position
        if (math.isfinite(dx) and math.isfinite(dy)
                and math.hypot(end.x - dx, end.y - dy) <= DOCK_MATCH_M):
            return str(self.get_parameter('dock_goal_checker_id').value)
        return str(self.get_parameter('goal_checker_id').value)

    def _on_goal_response(self, future, request_key):
        self._request_pending = False
        try:
            handle = future.result()
        except Exception as exc:  # action transport errors should be visible
            self.get_logger().error(f'Nav2 goal request failed: {exc}')
            self._retry_after = time.monotonic() + 1.0
            return
        if not handle.accepted:
            self.get_logger().warning('Nav2 rejected coordination goal')
            self._retry_after = time.monotonic() + 1.0
            return
        self._goal_handle = handle
        self._active_key = request_key
        self.get_logger().info(
            f'Nav2 accepted coordination path with {len(request_key[1])} '
            f'poses in {request_key[0]}')
        handle.get_result_async().add_done_callback(self._on_result)

    def _start_backup(self):
        self._backing_up = True
        self._backups_for_key += 1
        self.get_logger().warning(
            f'robot wedged ({self._wedged_aborts} no-progress aborts without '
            f'moving {WEDGE_MOVED_M} m): backing up {BACKUP_M} m before '
            f'retrying (backup {self._backups_for_key}/{MAX_BACKUPS_PER_ROUTE})')
        goal = BackUp.Goal()
        goal.target.x = BACKUP_M          # Nav2 backs up by |target.x|
        goal.speed = BACKUP_SPEED
        goal.time_allowance.sec = int(BACKUP_TIME_S)
        self._backup_client.send_goal_async(goal).add_done_callback(
            self._on_backup_accepted)

    def _on_backup_accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'backup request failed: {exc}')
            handle = None
        if handle is None or not handle.accepted:
            self._end_backup('rejected')
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._end_backup(
                'done' if f.exception() is None else 'failed'))

    def _end_backup(self, how):
        self.get_logger().info(f'backup {how}; retrying the route from here')
        self._backing_up = False
        self._wedged_aborts = 0
        self._completed_key = None        # re-send (trimmed to the robot)
        self._retry_after = 0.0

    def _on_cancelled(self, future):
        self._cancel_pending = False
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warning(f'Nav2 goal cancellation failed: {exc}')
        self._goal_handle = None
        self._active_key = None

    def _on_result(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Nav2 goal finished with status {result.status}')
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                if self._sent_final_segment:
                    self._completed_key = self._active_key
                else:
                    # Segment done: the robot stands spun-up at the turn
                    # point. Leave the key uncompleted so the next timer
                    # tick sends the next straight leg at once.
                    self._resume_xy = self._sent_seg_end
                    self._retry_after = 0.0
                self._wedged_aborts = 0
            else:
                self._retry_after = time.monotonic() + 1.0
                moved = (math.hypot(self._robot_xy[0] - self._goal_start_xy[0],
                                    self._robot_xy[1] - self._goal_start_xy[1])
                         if self._robot_xy and self._goal_start_xy else None)
                # A no-progress abort while the FLEET itself holds the
                # robot (velocity gate zeroing cmd_vel under a WAITING /
                # YIELDING permit) is not a wedge - it is Nav2's progress
                # checker timing out a legitimate coordination wait, and
                # backing up out of a queue slot helps nobody (robot_3
                # t=104 / robot_1 t=113, webots_live_final: both 'wedges'
                # at 0.5-1.6 m of face clearance during fleet stops).
                fleet_held = self._robot_status in ('WAITING', 'YIELDING')
                if (result.status == GoalStatus.STATUS_ABORTED
                        and not fleet_held
                        and moved is not None and moved < WEDGE_MOVED_M):
                    self._wedged_aborts += 1
                else:
                    self._wedged_aborts = 0
        except Exception as exc:
            self.get_logger().warning(f'Nav2 result unavailable: {exc}')
            self._retry_after = time.monotonic() + 1.0
        self._goal_handle = None
        self._active_key = None


def main(args=None):
    rclpy.init(args=args)
    node = FleetGoalBridge()
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
