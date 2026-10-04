"""Live fleet scorecard for a Webots run (diagnostics only — never fed back).

    python3 fleet_eval.py [duration_s]

Uses the supervisor's ground truth on /diagnostics/ground_truth/robot_N and
each robot's belief on /robot_N/amcl_pose. Reports per robot:
  * localization error (believed vs true) — mean / max / worst moment
  * lane centering: lateral offset from the aisle/corridor centre line while
    the TRUE position is inside aisle A, aisle B or the centre corridor
  * arrival accuracy: true distance to the dropoff marker when the robot
    announces task completion
and fleet totals (tasks announced / awarded / completed).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

from amr_description.msg import RobotState, Task, TaskAward, TaskComplete

ROBOTS = ['robot_1', 'robot_2', 'robot_3']
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
COORD_QOS = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def cell_to_world(row, col):
    return -6.0 + (col + 0.5) * 0.1, -5.0 + (row + 0.5) * 0.1


def lane_offset(x, y):
    """Lateral offset from the lane centre line, or None outside lanes."""
    # R7 layout: every aisle and the spine is 1.6 m wide (warehouse_grid.yaml
    # 'racks:'); the 0.2 m margin keeps spine-mouth turns out of the aisles.
    if 3.4 <= y <= 5.0 and abs(x) > 1.0:
        return ('aisle_N', abs(y - 4.2))
    if 1.2 <= y <= 2.8 and abs(x) > 1.0:
        return ('aisle_A', abs(y - 2.0))
    if -1.0 <= y <= 0.6 and abs(x) > 1.0:
        return ('aisle_B', abs(y + 0.2))
    if abs(x) <= 0.8 and -1.6 <= y <= 3.4:
        return ('corridor', abs(x))
    return None


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


class Eval(Node):
    def __init__(self):
        super().__init__('fleet_eval')
        self.truth, self.belief = {}, {}
        self.err = {r: [] for r in ROBOTS}
        self.worst = {r: (0.0, None) for r in ROBOTS}
        self.lane = {r: {} for r in ROBOTS}
        self.arrivals = []
        self.tasks, self.awards, self.completed = {}, {}, []
        self.min_sep = {}          # (a, b) -> (distance, where/when)
        self.contact_samples = 0
        # R5/R4(4): per-task cycle time (announce -> complete) and duplicate
        # ownership from each robot's broadcast task_id (ground truth).
        self.announce_t = {}       # task_id -> wall time first announced
        self.cycles = []           # (task_id, robot, cycle_s) in finish order
        self.double_awards = []    # (task_id, first_winner, second_winner)
        self.claim = {}            # robot -> (task_id, wall time of message)
        self.dup_samples = {}      # task_id -> duplicate-ownership samples
        self.completed_once = {}   # task_id -> first completing robot
        for r in ROBOTS:
            self.create_subscription(
                PoseStamped, f'/diagnostics/ground_truth/{r}',
                lambda m, r=r: self.truth.__setitem__(r, m), 10)
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{r}/amcl_pose',
                lambda m, r=r: self.belief.__setitem__(r, m), 10)
        self.create_subscription(Task, '/fleet/task_announce',
                                 self.on_task, COORD_QOS)
        self.create_subscription(TaskAward, '/fleet/task_award',
                                 self.on_award, COORD_QOS)
        self.create_subscription(TaskComplete, '/fleet/task_complete',
                                 self.on_complete, COORD_QOS)
        # Best-effort matches both reliable and best-effort publishers; the
        # fleet switched robot_state to a sensor-style QoS.
        self.create_subscription(
            RobotState, '/fleet/robot_state', self.on_state,
            QoSProfile(depth=50,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.5, self.sample)

    def on_task(self, m):
        self.tasks[m.task_id] = cell_to_world(m.dropoff_row, m.dropoff_col)
        self.announce_t.setdefault(m.task_id, time.time())

    def on_award(self, m):
        prev = self.awards.get(m.task_id)
        if (prev is not None and prev != m.winner_id
                and m.task_id not in self.completed_once):
            # A re-award after release/dead-assignee is legal; two LIVE
            # winners is the R4(4) bug. The dup_samples counter below is the
            # authoritative ground-truth check; this flags the award trail.
            self.double_awards.append((m.task_id, prev, m.winner_id))
        self.awards[m.task_id] = m.winner_id

    def on_state(self, m):
        # task_id ground truth at state rate: two robots broadcasting the
        # same live task_id simultaneously = duplicate ownership.
        self.claim[m.robot_id] = (m.task_id, time.time())

    def on_complete(self, m):
        self.completed.append((m.task_id, m.robot_id))
        self.completed_once.setdefault(m.task_id, m.robot_id)
        t0 = self.announce_t.get(m.task_id)
        if t0 is not None:
            self.cycles.append((m.task_id, m.robot_id,
                                time.time() - t0))
        t = self.truth.get(m.robot_id)
        drop = self.tasks.get(m.task_id)
        if t is not None and drop is not None:
            d = math.hypot(t.pose.position.x - drop[0],
                           t.pose.position.y - drop[1])
            self.arrivals.append((m.robot_id, m.task_id, d))

    def sample(self):
        now = time.time()
        # duplicate ownership: two robots broadcasting one live task_id
        fresh = {r: tid for r, (tid, ts) in self.claim.items()
                 if tid and now - ts < 1.0 and tid not in self.completed_once}
        seen = {}
        for r, tid in fresh.items():
            seen.setdefault(tid, []).append(r)
        for tid, rs in seen.items():
            if len(rs) >= 2:
                self.dup_samples[tid] = self.dup_samples.get(tid, 0) + 1
        ids = [r for r in ROBOTS if r in self.truth]
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                pa, pb = self.truth[a].pose.position, self.truth[b].pose.position
                d = math.hypot(pa.x - pb.x, pa.y - pb.y)
                if d < 0.42:
                    self.contact_samples += 1
                if d < self.min_sep.get((a, b), (1e9, ''))[0]:
                    self.min_sep[(a, b)] = (
                        d, f'at ({(pa.x + pb.x) / 2:.2f},{(pa.y + pb.y) / 2:.2f}) '
                           f't+{now - START:.0f}s')
        for r in ROBOTS:
            t = self.truth.get(r)
            if t is None:
                continue
            tx, ty = t.pose.position.x, t.pose.position.y
            lo = lane_offset(tx, ty)
            if lo is not None:
                self.lane[r].setdefault(lo[0], []).append(lo[1])
            b = self.belief.get(r)
            if b is None:
                continue
            bx, by = b.pose.pose.position.x, b.pose.pose.position.y
            e = math.hypot(bx - tx, by - ty)
            self.err[r].append(e)
            if e > self.worst[r][0]:
                self.worst[r] = (e, f'true=({tx:.2f},{ty:.2f}) '
                                    f'believed=({bx:.2f},{by:.2f}) '
                                    f't+{now - START:.0f}s')


def main():
    global START
    rclpy.init()
    node = Eval()
    START = time.time()
    while time.time() - START < DURATION:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f'=== FLEET SCORECARD ({DURATION:.0f} s) ===')
    print(f'tasks announced={len(node.tasks)} awarded={len(node.awards)} '
          f'completed={len(node.completed)}')
    for r in ROBOTS:
        e = node.err[r]
        if not node.truth.get(r):
            print(f'{r}: NO GROUND TRUTH received')
            continue
        if e:
            print(f'{r}: loc error mean={sum(e) / len(e):.2f} m '
                  f'max={max(e):.2f} m  worst: {node.worst[r][1]}')
        else:
            print(f'{r}: no amcl_pose received')
        for lane, offs in sorted(node.lane[r].items()):
            print(f'    {lane:9s} centring: mean={sum(offs) / len(offs):.2f} m '
                  f'max={max(offs):.2f} m (n={len(offs)})')
    for r, tid, d in node.arrivals:
        print(f'arrival {tid} by {r}: {d:.2f} m from dropoff marker (truth)')
    if node.cycles:
        cs = [c for _, _, c in node.cycles]
        h = (len(cs) + 1) // 2
        second = cs[h:]
        print(f'task cycle (announce->complete): mean={sum(cs) / len(cs):.1f} s '
              f'max={max(cs):.1f} s n={len(cs)}'
              + (f'  1st half={sum(cs[:h]) / h:.1f} s '
                 f'2nd half={sum(second) / len(second):.1f} s' if second else ''))
        for tid, r, c in node.cycles:
            print(f'    {tid} by {r}: {c:.1f} s')
    if node.double_awards:
        print(f'DOUBLE AWARDS (R4(4) violation candidates): {node.double_awards}')
    if node.dup_samples:
        print('DUPLICATE OWNERSHIP (robot_state task_id ground truth, 2 Hz):')
        for tid, n in sorted(node.dup_samples.items()):
            print(f'    {tid}: {n * 0.5:.1f} s')
    else:
        print('duplicate task ownership: none observed')
    print(f'robot-robot contact samples (<0.42 m, 2 Hz): {node.contact_samples}')
    for (a, b), (d, where) in sorted(node.min_sep.items()):
        print(f'min separation {a}-{b}: {d:.2f} m {where}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
