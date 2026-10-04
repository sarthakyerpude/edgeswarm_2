"""
The ONLY file that knows both ROS messages and core dataclasses.

Every other module is on exactly one side of the boundary. Keeping the
conversion in one place is what allows core/ to be unit-tested without ROS and
(later) run on hardware with a different transport.
"""
import math
from typing import List, Optional

from amr_description.msg import (Intent as RosIntent, MapUpdate as RosMapUpdate,
                                 MotionPermit as RosPermit,
                                 RobotState as RosRobotState, Task as RosTask,
                                 Conflict as RosConflict)

try:    # Generated only after the next colcon build (R6, msg/TaskCancel.msg).
    from amr_description.msg import TaskCancel as RosTaskCancel
except ImportError:                      # pragma: no cover - stale install
    RosTaskCancel = None
from amr_fleet.core.models import (Conflict, Intent, Permit, Pose2D, RobotState,
                                   STATUS_ORDER, Task)

ACTION_ORDER = ["GO", "SLOW", "STOP", "YIELD", "REROUTE"]
CONFLICT_KINDS = ["CELL", "SWAP", "ZONE", "TTC"]


def _stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# ------------------------------------------------------------ RobotState ----
def state_to_ros(s: RobotState, clock) -> RosRobotState:
    m = RosRobotState()
    m.header.stamp = clock.now().to_msg()
    m.header.frame_id = "map"
    m.robot_id = s.robot_id
    m.seq = int(s.seq) & 0xFFFFFFFF
    m.boot_id = int(s.boot_id) & 0xFFFFFFFFFFFFFFFF
    m.pose.x, m.pose.y, m.pose.theta = s.pose.x, s.pose.y, s.pose.theta
    m.velocity.linear.x = float(s.v)
    m.velocity.angular.z = float(s.w)
    m.status = STATUS_ORDER.index(s.status) if s.status in STATUS_ORDER else 0
    m.task_id = s.task_id or ""
    m.task_priority = int(s.task_priority)
    m.battery_pct = float(s.battery_pct)
    m.waiting_time = float(s.waiting_time)
    m.waiting_for = s.waiting_for or ""
    m.priority_score = float(s.priority_score)
    m.alive = bool(s.alive)
    m.intent_seq = int(s.intent_seq) & 0xFFFFFFFF
    m.loc_health = max(0, min(255, int(s.loc_health)))
    m.loc_sigma_lat = float(s.loc_sigma_lat)
    return m


def state_from_ros(m: RosRobotState) -> RobotState:
    return RobotState(
        robot_id=m.robot_id,
        seq=int(m.seq),
        boot_id=int(m.boot_id),
        stamp=_stamp_to_float(m.header.stamp),
        pose=Pose2D(m.pose.x, m.pose.y, m.pose.theta),
        v=m.velocity.linear.x,
        w=m.velocity.angular.z,
        status=STATUS_ORDER[m.status] if m.status < len(STATUS_ORDER) else "IDLE",
        task_id=m.task_id,
        task_priority=int(m.task_priority),
        battery_pct=float(m.battery_pct),
        waiting_time=float(m.waiting_time),
        waiting_for=m.waiting_for,
        priority_score=float(m.priority_score),
        alive=bool(m.alive),
        intent_seq=int(m.intent_seq),
        loc_health=int(m.loc_health),
        loc_sigma_lat=float(m.loc_sigma_lat),
    )


# ---------------------------------------------------------------- Intent ----
def intent_to_ros(robot_id: str, seq: int, it: Intent, clock,
                  task_id: str = "", task_priority: int = 0,
                  priority_score: float = 0.0,
                  stamp: Optional[float] = None) -> RosIntent:
    """stamp: the clock reading the windows were timed from. Receivers use
    t - header.stamp, so it must be the same 'now' the intent was built at."""
    m = RosIntent()
    if stamp is None:
        m.header.stamp = clock.now().to_msg()
    else:
        sec = math.floor(stamp)
        m.header.stamp.sec = int(sec)
        m.header.stamp.nanosec = min(999999999, int((stamp - sec) * 1e9))
    m.header.frame_id = "map"
    m.robot_id = robot_id
    m.seq = int(seq) & 0xFFFFFFFF
    m.path_rows = [int(c[0]) for c in it.cells]
    m.path_cols = [int(c[1]) for c in it.cells]
    m.t_enter = [float(t) for t in it.t_enter]
    m.t_exit = [float(t) for t in it.t_exit]
    m.zones_needed = list(it.zones)
    m.active_zone_index = int(it.active_zone_index)
    m.goal.x, m.goal.y, m.goal.theta = it.goal.x, it.goal.y, it.goal.theta
    m.task_id = task_id
    m.task_priority = int(task_priority)
    m.priority_score = float(priority_score)
    return m


def intent_from_ros(m: RosIntent, rx_time: Optional[float] = None) -> Intent:
    """With rx_time (receiver clock), windows are converted to the receiver's
    clock: t_local = rx_time + (t - header.stamp), so clock skew between
    robots cancels. Without it, the sender's times are kept as sent."""
    # Defensive: never trust remote array lengths to agree.
    n = min(len(m.path_rows), len(m.path_cols), len(m.t_enter), len(m.t_exit))
    it = Intent(
        cells=[(int(m.path_rows[i]), int(m.path_cols[i])) for i in range(n)],
        t_enter=[float(m.t_enter[i]) for i in range(n)],
        t_exit=[float(m.t_exit[i]) for i in range(n)],
        zones=list(m.zones_needed),
        goal=Pose2D(m.goal.x, m.goal.y, m.goal.theta),
        active_zone_index=int(m.active_zone_index),
    )
    if rx_time is None:
        return it
    return it.to_receiver_clock(_stamp_to_float(m.header.stamp), rx_time)


# ------------------------------------------------------------ MapUpdate ----
def map_update_to_ros(reporter: str, blocked, cleared, confidence: float,
                      expiry: float, clock) -> RosMapUpdate:
    m = RosMapUpdate()
    m.header.stamp = clock.now().to_msg()
    m.header.frame_id = "map"
    m.reporter_id = reporter
    m.blocked_rows = [int(c[0]) for c in blocked]
    m.blocked_cols = [int(c[1]) for c in blocked]
    m.cleared_rows = [int(c[0]) for c in cleared]
    m.cleared_cols = [int(c[1]) for c in cleared]
    m.confidence = float(confidence)
    m.expiry = float(expiry)
    return m


def map_update_from_ros(m: RosMapUpdate):
    nb = min(len(m.blocked_rows), len(m.blocked_cols))
    nc = min(len(m.cleared_rows), len(m.cleared_cols))
    blocked = [(int(m.blocked_rows[i]), int(m.blocked_cols[i])) for i in range(nb)]
    cleared = [(int(m.cleared_rows[i]), int(m.cleared_cols[i])) for i in range(nc)]
    return m.reporter_id, blocked, cleared, float(m.confidence), float(m.expiry)


# ----------------------------------------------------------------- Task -----
def task_from_ros(m: RosTask) -> Task:
    return Task(task_id=m.task_id,
                pickup=(int(m.pickup_row), int(m.pickup_col)),
                dropoff=(int(m.dropoff_row), int(m.dropoff_col)),
                priority=int(m.priority),
                created_at=float(m.created_at),
                deadline=float(m.deadline))


def task_to_ros(d: dict, clock, announcer: str) -> RosTask:
    m = RosTask()
    m.header.stamp = clock.now().to_msg()
    m.task_id = d["task_id"]
    m.pickup_row = int(d["pickup_row"])
    m.pickup_col = int(d["pickup_col"])
    m.dropoff_row = int(d["dropoff_row"])
    m.dropoff_col = int(d["dropoff_col"])
    m.priority = int(d.get("priority", 0))
    m.created_at = float(d.get("created_at", 0.0))
    m.deadline = float(d.get("deadline", 0.0))
    m.announcer_id = announcer
    return m


# ----------------------------------------------------------- TaskCancel -----
def task_cancel_to_ros(task_id: str, requester_id: str, reason: str,
                       clock=None, msg=None):
    """Build a TaskCancel message (R6). `msg` lets tests pass a stub when the
    generated class is not installed yet; nodes omit it."""
    if msg is None:
        if RosTaskCancel is None:
            raise RuntimeError(
                "amr_description.msg.TaskCancel is not installed; "
                "rebuild amr_description (msg/TaskCancel.msg)")
        msg = RosTaskCancel()
    if clock is not None:
        msg.header.stamp = clock.now().to_msg()
    msg.task_id = str(task_id)
    msg.requester_id = str(requester_id)
    msg.reason = str(reason)
    return msg


def task_cancel_from_ros(m) -> dict:
    """Duck-typed on purpose: works on the generated class and on stubs."""
    return dict(task_id=str(m.task_id), requester_id=str(m.requester_id),
                reason=str(m.reason))


# --------------------------------------------------------------- Permit -----
def permit_to_ros(robot_id: str, p: Permit, clock) -> RosPermit:
    m = RosPermit()
    m.header.stamp = clock.now().to_msg()
    m.robot_id = robot_id
    m.action = ACTION_ORDER.index(p.action) if p.action in ACTION_ORDER else 0
    m.speed_scale = float(max(0.0, min(1.0, p.speed_scale)))
    m.reason = p.reason
    m.blocking_robot = p.blocking_robot
    m.zone_id = p.zone_id
    m.deadlock_detected = bool(p.deadlock_detected)
    m.deadlock_cycle = list(p.deadlock_cycle)
    return m


# ------------------------------------------------------------- Conflict -----
def conflict_to_ros(robot_id: str, c: Conflict, clock) -> RosConflict:
    m = RosConflict()
    m.header.stamp = clock.now().to_msg()
    m.robot_id = robot_id
    m.peer_id = c.peer_id
    m.kind = CONFLICT_KINDS.index(c.kind) if c.kind in CONFLICT_KINDS else 0
    m.cell_row = int(c.cell[0]) if c.cell else -1
    m.cell_col = int(c.cell[1]) if c.cell else -1
    m.zone_id = c.zone_id
    m.t_conflict = float(c.t_conflict)
    m.severity = float(c.severity)
    m.my_priority = float(c.my_priority)
    m.peer_priority = float(c.peer_priority)
    m.i_have_priority = bool(c.i_have_priority)
    return m
