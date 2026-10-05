"""Webots supervisor plugin giving the fleet's tasks a visible item lifecycle.

Listens to the fleet's OWN P2P auction bus (it makes no decisions):
  /fleet/task_announce  -> remember each task's pickup/dropoff cells
  /fleet/task_award     -> remember which robot won the task
  /fleet/task_complete  -> drop the carried item at the robot's position

When the awarded robot arrives at the task's pickup cell, the nearest free
item teleports onto its back (carried above the chassis) and follows it until
completion; a few seconds after delivery the item respawns at its home pickup
marker, so pickup points stay stocked. Positions come from supervisor
ground truth (display logic only — never fed back to the robots).

It also runs the localization divergence monitor: every DIVERGE_LOG_S it logs
each robot's AMCL belief (/robot_N/amcl_pose) against supervisor ground truth
sampled at the belief's own stamp, WARNs 'LOCALIZATION DIVERGED' above
DIVERGE_WARN_M, and publishes the truth on /diagnostics/ground_truth/robot_N
(diagnostics only — no robot node may subscribe to it: no god-mode).
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

from amr_description.msg import Task, TaskAward, TaskComplete, TaskRelease

GRID_ORIGIN = (-6.0, -5.0)
GRID_RES = 0.1
PICKUP_RADIUS = 0.45
CARRY_HEIGHT = 0.26
DROP_HEIGHT = 0.06
RESPAWN_S = 8.0

ITEM_DEFS = ['ITEM_1', 'ITEM_2', 'ITEM_3']
ITEM_HOMES = [(-3.45, 3.85), (3.45, 3.85), (-3.45, 2.45)]   # L1/L2/L3 cell centres (slots 1R10, 3R11, 2R10)
ROBOT_DEFS = {f'robot_{i}': f'ROBOT_{i}' for i in (1, 2, 3)}

DIVERGE_LOG_S = 2.0
DIVERGE_WARN_M = 0.5
CONTACT_DIST_M = 0.42
TRUTH_PUB_S = 0.1
# Truth history kept to match AMCL stamps (AMCL publishes only on filter
# updates, so comparing against *current* truth reads motion as error).
TRUTH_HISTORY_S = 5.0

COORD_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _cell_to_world(row, col):
    return (GRID_ORIGIN[0] + (col + 0.5) * GRID_RES,
            GRID_ORIGIN[1] + (row + 0.5) * GRID_RES)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _truth_pose(node):
    """(x, y, yaw) of a Webots node; world is ENU z-up, so yaw = atan2(R10, R00)."""
    x, y, _ = node.getPosition()
    r = node.getOrientation()
    return x, y, math.atan2(r[3], r[0])


class ItemManager:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__items = [self.__robot.getFromDef(d) for d in ITEM_DEFS]
        self.__robots = {rid: self.__robot.getFromDef(d)
                         for rid, d in ROBOT_DEFS.items()}

        # item state: 'home' | ('carried', robot_id) | ('respawn_at', t)
        self.__state = ['home'] * len(self.__items)
        self.__tasks = {}     # task_id -> (pickup_xy, task row data)
        self.__awards = {}    # task_id -> robot_id
        self.__carrying = {}  # robot_id -> (item_idx, task_id)

        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('item_manager')
        self.__node.create_subscription(
            Task, '/fleet/task_announce', self.__on_task, COORD_QOS)
        self.__node.create_subscription(
            TaskAward, '/fleet/task_award', self.__on_award, COORD_QOS)
        self.__node.create_subscription(
            TaskComplete, '/fleet/task_complete', self.__on_complete, COORD_QOS)
        self.__node.create_subscription(
            TaskRelease, '/fleet/task_release', self.__on_release, COORD_QOS)
        self.__log = self.__node.get_logger()

        # Divergence monitor (diagnostics only).
        self.__truth_hist = {rid: deque() for rid in self.__robots}
        self.__belief = {}    # robot_id -> latest belief-vs-truth record
        self.__truth_pubs = {}
        self.__last_truth_pub = -TRUTH_PUB_S
        self.__last_diverge_log = -DIVERGE_LOG_S
        self.__last_contact_log = {}
        self.__contacts = 0
        for rid in self.__robots:
            self.__node.create_subscription(
                PoseWithCovarianceStamped, f'/{rid}/amcl_pose',
                lambda msg, rid=rid: self.__on_amcl_pose(rid, msg), 10)
            self.__truth_pubs[rid] = self.__node.create_publisher(
                PoseStamped, f'/diagnostics/ground_truth/{rid}', 10)

    def __on_task(self, msg):
        self.__tasks[msg.task_id] = _cell_to_world(msg.pickup_row,
                                                   msg.pickup_col)

    def __on_award(self, msg):
        self.__awards[msg.task_id] = msg.winner_id

    def __on_release(self, msg):
        # The task goes back to the pool: drop any carried item where the
        # releasing robot stands and clear the stale award.
        self.__awards.pop(msg.task_id, None)
        carried = self.__carrying.get(msg.robot_id)
        if carried and carried[1] == msg.task_id:
            self.__carrying.pop(msg.robot_id)
            idx = carried[0]
            node = self.__robots.get(msg.robot_id)
            if node is not None:
                x, y, _ = node.getPosition()
                self.__items[idx].getField('translation').setSFVec3f(
                    [x, y, DROP_HEIGHT])
            self.__state[idx] = ('respawn_at',
                                 self.__robot.getTime() + RESPAWN_S)

    def __on_complete(self, msg):
        carried = self.__carrying.pop(msg.robot_id, None)
        self.__awards.pop(msg.task_id, None)
        self.__tasks.pop(msg.task_id, None)
        if carried is None:
            return
        idx, _ = carried
        node = self.__robots.get(msg.robot_id)
        if node is not None:
            x, y, _ = node.getPosition()
            self.__items[idx].getField('translation').setSFVec3f(
                [x, y, DROP_HEIGHT])
        self.__state[idx] = ('respawn_at',
                             self.__robot.getTime() + RESPAWN_S)
        self.__log.info(f'{msg.robot_id} delivered item_{idx + 1} '
                        f'(task {msg.task_id})')

    # ------------------------------------------------ divergence monitor --
    def __truth_at(self, rid, t):
        """Recorded truth sample closest to (not after) sim time t."""
        hist = self.__truth_hist.get(rid)
        if not hist:
            return None
        for sample in reversed(hist):
            if sample[0] <= t:
                return sample
        return hist[0]

    def __on_amcl_pose(self, rid, msg):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        byaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        truth = self.__truth_at(rid, stamp)
        if truth is None:
            return
        _, tx, ty, tyaw = truth
        cov = msg.pose.covariance
        self.__belief[rid] = dict(
            stamp=stamp, bx=p.position.x, by=p.position.y, byaw=byaw,
            tx=tx, ty=ty, tyaw=tyaw,
            err=math.hypot(p.position.x - tx, p.position.y - ty),
            yaw_err=_wrap(byaw - tyaw),
            sigma_xy=math.sqrt(max(0.0, cov[0] + cov[7])))

    def __monitor_localization(self, now):
        truth_now = {}
        for rid, node in self.__robots.items():
            if node is None:
                continue
            x, y, yaw = _truth_pose(node)
            truth_now[rid] = (x, y, yaw)
            hist = self.__truth_hist[rid]
            hist.append((now, x, y, yaw))
            while hist and now - hist[0][0] > TRUTH_HISTORY_S:
                hist.popleft()

        # Robot-robot contact from ground truth (diagnostics only). Two
        # 0.40 x 0.32 m bodies touch at ~0.36-0.40 m centre distance; checked
        # every step because contacts can be brief, logged at most once per
        # second per pair. Webots has no contact recorder for the CSVs.
        ids = sorted(truth_now)
        for i, a in enumerate(ids):
            for b_id in ids[i + 1:]:
                ax, ay, _ = truth_now[a]
                bx, by, _ = truth_now[b_id]
                d = math.hypot(ax - bx, ay - by)
                key = (a, b_id)
                if (d < CONTACT_DIST_M and
                        now - self.__last_contact_log.get(key, -1e9) >= 1.0):
                    self.__last_contact_log[key] = now
                    self.__contacts += 1
                    self.__log.warning(
                        f'CONTACT {a} {b_id} d={d:.2f}m t={now:.1f} '
                        f'at ({(ax + bx) / 2:.2f},{(ay + by) / 2:.2f}) '
                        f'total={self.__contacts}')

        if now - self.__last_truth_pub >= TRUTH_PUB_S:
            self.__last_truth_pub = now
            for rid, (x, y, yaw) in truth_now.items():
                msg = PoseStamped()
                msg.header.stamp.sec = int(now)
                msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
                msg.header.frame_id = 'map'
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.orientation.w = math.cos(yaw / 2.0)
                self.__truth_pubs[rid].publish(msg)

        if now - self.__last_diverge_log < DIVERGE_LOG_S:
            return
        self.__last_diverge_log = now
        for rid, (x, y, yaw) in truth_now.items():
            b = self.__belief.get(rid)
            if b is None:
                self.__log.info(
                    f'[loc] {rid} t={now:.1f} no amcl_pose yet; '
                    f'true=({x:.2f},{y:.2f},{math.degrees(yaw):.0f}deg)')
                continue
            line = (f'{rid} t={now:.1f} '
                    f'believed=({b["bx"]:.2f},{b["by"]:.2f},'
                    f'{math.degrees(b["byaw"]):.0f}deg) '
                    f'true@stamp=({b["tx"]:.2f},{b["ty"]:.2f},'
                    f'{math.degrees(b["tyaw"]):.0f}deg) '
                    f'err={b["err"]:.2f}m '
                    f'yaw_err={math.degrees(b["yaw_err"]):.0f}deg '
                    f'sigma_xy={b["sigma_xy"]:.2f}m '
                    f'amcl_age={now - b["stamp"]:.1f}s '
                    f'true_now=({x:.2f},{y:.2f})')
            # The '[loc]' sample is emitted UNCONDITIONALLY (round-3 fix):
            # it used to be replaced by the DIVERGED warning above 0.5 m, so
            # any [loc]-based trace extraction lost a robot exactly while it
            # was diverged (loc_validate2: robot_3's [loc] stream ended at
            # t=138 as it entered its divergence - 62 of ~300 samples). The
            # warning is now ADDITIONAL, so all three robots are measured
            # for the full run no matter their error.
            self.__log.info(f'[loc] {line}')
            if b['err'] > DIVERGE_WARN_M:
                self.__log.warning(
                    f'LOCALIZATION DIVERGED {rid} err={b["err"]:.2f}m | {line}')

    def __free_item_near(self, xy):
        best, best_d = None, 0.35
        for idx, state in enumerate(self.__state):
            if state != 'home':
                continue
            pos = self.__items[idx].getField('translation').getSFVec3f()
            d = math.hypot(pos[0] - xy[0], pos[1] - xy[1])
            if d < best_d:
                best, best_d = idx, d
        return best

    def __try_pickups(self):
        for task_id, winner in list(self.__awards.items()):
            if winner in self.__carrying or task_id not in self.__tasks:
                continue
            node = self.__robots.get(winner)
            if node is None:
                continue
            x, y, _ = node.getPosition()
            px, py = self.__tasks[task_id]
            if math.hypot(x - px, y - py) > PICKUP_RADIUS:
                continue
            idx = self.__free_item_near((px, py))
            if idx is None:
                continue
            self.__carrying[winner] = (idx, task_id)
            self.__state[idx] = ('carried', winner)
            self.__log.info(f'{winner} picked up item_{idx + 1} '
                            f'(task {task_id})')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        now = self.__robot.getTime()
        self.__monitor_localization(now)
        if now - getattr(self, '_dbg_t', -10.0) >= 5.0:
            self._dbg_t = now
            dists = []
            for tid, winner in list(self.__awards.items())[:3]:
                node = self.__robots.get(winner)
                if node is not None and tid in self.__tasks:
                    x, y, _ = node.getPosition()
                    px, py = self.__tasks[tid]
                    dists.append(f'{tid}:{winner}@({x:.1f},{y:.1f})->'
                                 f'({px:.1f},{py:.1f})='
                                 f'{math.hypot(x - px, y - py):.2f}m')
            self.__log.info(
                f'tasks={len(self.__tasks)} awards={len(self.__awards)} '
                f'carrying={list(self.__carrying)} {" | ".join(dists)}')
        self.__try_pickups()
        for idx, state in enumerate(self.__state):
            if isinstance(state, tuple) and state[0] == 'carried':
                node = self.__robots.get(state[1])
                if node is None:
                    continue
                x, y, _ = node.getPosition()
                self.__items[idx].getField('translation').setSFVec3f(
                    [x, y, CARRY_HEIGHT])
            elif isinstance(state, tuple) and state[0] == 'respawn_at':
                if now >= state[1]:
                    hx, hy = ITEM_HOMES[idx]
                    self.__items[idx].getField('translation').setSFVec3f(
                        [hx, hy, DROP_HEIGHT])
                    self.__state[idx] = 'home'
