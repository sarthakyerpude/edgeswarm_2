"""
Plain dataclasses shared by every core algorithm.

These mirror the ROS messages but are deliberately independent of them. The
conversion lives in amr_fleet/nodes/conversions.py and nowhere else. That one
adapter file is the entire price of keeping the algorithms portable.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Cell = Tuple[int, int]          # (row, col) in the occupancy grid

# Status strings. Kept as strings in core (readable in logs and tests) and
# mapped to the uint8 constants in the ROS layer.
IDLE, MOVING, WAITING, YIELDING, CHARGING, FAULT = (
    "IDLE", "MOVING", "WAITING", "YIELDING", "CHARGING", "FAULT")

STATUS_ORDER = [IDLE, MOVING, WAITING, YIELDING, CHARGING, FAULT]

# RobotState.loc_health values (same numbering as the uint8 on the wire).
LOC_OK, LOC_DEGRADED, LOC_LOST = 0, 1, 2


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Intent:
    """A time-parameterised claim on space.

    cells[i] is occupied from t_enter[i] to t_exit[i] (absolute epoch seconds,
    already widened by the safety margin at generation time). A held peer
    intent is always on the RECEIVER's clock (see to_receiver_clock).
    """
    cells: List[Cell] = field(default_factory=list)
    t_enter: List[float] = field(default_factory=list)
    t_exit: List[float] = field(default_factory=list)
    zones: List[str] = field(default_factory=list)
    goal: Pose2D = field(default_factory=Pose2D)
    active_zone_index: int = -1

    def triples(self):
        """Yield (cell, t_enter, t_exit) safely even if arrays got truncated
        in transit - never trust remote array lengths to match."""
        n = min(len(self.cells), len(self.t_enter), len(self.t_exit))
        for i in range(n):
            yield self.cells[i], self.t_enter[i], self.t_exit[i]

    def window_for(self, cell: Cell) -> Optional[Tuple[float, float]]:
        for c, a, b in self.triples():
            if c == cell:
                return (a, b)
        return None

    def to_receiver_clock(self, sender_stamp: float,
                          rx_time: float) -> "Intent":
        """Copy with every window shifted onto the receiver's clock:
        t_local = rx_time + (t - sender_stamp). Only differences of the
        sender's own timestamps are used, so inter-robot clock skew cancels;
        network latency is absorbed into the (advisory) windows."""
        offset = rx_time - sender_stamp
        return Intent(cells=list(self.cells),
                      t_enter=[t + offset for t in self.t_enter],
                      t_exit=[t + offset for t in self.t_exit],
                      zones=list(self.zones),
                      goal=Pose2D(self.goal.x, self.goal.y, self.goal.theta),
                      active_zone_index=self.active_zone_index)


@dataclass
class RobotState:
    """Everything one robot broadcasts about itself."""
    robot_id: str
    seq: int = 0
    boot_id: int = 0
    stamp: float = 0.0              # capture time (epoch seconds)
    pose: Pose2D = field(default_factory=Pose2D)
    v: float = 0.0                  # linear  m/s
    w: float = 0.0                  # angular rad/s
    status: str = IDLE
    task_id: str = ""
    task_priority: int = 0
    battery_pct: float = 100.0
    waiting_time: float = 0.0
    waiting_for: str = ""
    # R1: the broadcast right-of-way level L = 1000*class + 100*urgency +
    # wait bonus (core/priority.level). Integer-valued 0..4909, exact in
    # float32 on the wire. Consumers that assumed [0, 1] must rescale.
    priority_score: float = 0.0
    alive: bool = True
    intent: Intent = field(default_factory=Intent)
    intent_seq: int = 0             # seq of the sender's latest Intent
    loc_health: int = LOC_OK        # LOC_OK | LOC_DEGRADED | LOC_LOST
    loc_sigma_lat: float = 0.0      # lateral 1-sigma pose error, metres

    # Populated locally by the peer registry on receipt; never transmitted.
    rx_time: float = 0.0

    @property
    def vx(self) -> float:
        import math
        return self.v * math.cos(self.pose.theta)

    @property
    def vy(self) -> float:
        import math
        return self.v * math.sin(self.pose.theta)


@dataclass
class Task:
    """One pickup->dropoff job.

    deadline (R5): absolute epoch seconds by which the task should be
    delivered. task_generator fills it as created_at + BUDGET_S[priority]
    (priority.BUDGET_S, higher priority = tighter budget). 0.0 means "not
    set": consumers then apply priority.eff_deadline(task, now), which falls
    back to the same budget from created_at. As (now - created_at) consumes
    the budget the task's URGENCY escalates (priority.urgency via
    tasks.task_urgency), so old tasks win batch assignment order, pull order
    and right-of-way, and never starve. deadline is a soft aging anchor, not
    a hard abort.
    """
    task_id: str
    pickup: Cell
    dropoff: Cell
    priority: int = 0
    created_at: float = 0.0
    deadline: float = 0.0


@dataclass
class Conflict:
    kind: str                       # "CELL" | "SWAP" | "ZONE" | "TTC"
    peer_id: str
    t_conflict: float
    cell: Optional[Cell] = None
    zone_id: str = ""
    severity: float = 0.0
    my_priority: float = 0.0
    peer_priority: float = 0.0
    i_have_priority: bool = False


@dataclass
class Permit:
    """The decision this package produces. Consumed by the navigation layer."""
    action: str = "GO"              # GO | SLOW | STOP | YIELD | REROUTE
    speed_scale: float = 1.0
    reason: str = ""
    blocking_robot: str = ""
    zone_id: str = ""
    deadlock_detected: bool = False
    deadlock_cycle: List[str] = field(default_factory=list)
