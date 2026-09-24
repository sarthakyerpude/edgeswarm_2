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


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Intent:
    """A time-parameterised claim on space.

    cells[i] is occupied from t_enter[i] to t_exit[i] (absolute epoch seconds,
    already widened by the safety margin at generation time).
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


@dataclass
class RobotState:
    """Everything one robot broadcasts about itself."""
    robot_id: str
    seq: int = 0
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
    priority_score: float = 0.0
    alive: bool = True
    intent: Intent = field(default_factory=Intent)

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
