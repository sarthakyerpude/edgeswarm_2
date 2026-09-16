"""
A* on the warehouse occupancy grid, plus space-time intent generation.

SCOPE - read this before assuming this file plans the robot's motion.

This planner exists so the coordination layer can produce a SPACE-TIME INTENT
to broadcast, and so the deadlock handler can evaluate "is there an alternative
route?". It does NOT command the robot. Motion control belongs to
amr_navigation / Nav2, which owns cmd_vel. We publish a MotionPermit and, if
the navigation team wants it, a suggested path on <ns>/coordination_path.

Keeping these separate means this package never fights Nav2 for the actuator.

WHY PLAIN A* AND NOT D* LITE
On a grid of this size (roughly 1100 free cells) A* expands 150-400 nodes,
which is about 2-6 ms on a laptop and 4-8 ms on a Jetson Nano. Replans happen a
few times per minute. D* Lite's incremental advantage only pays off on much
larger maps replanned at high rate, and it costs ~250 lines of subtle code.
Measure before optimising.
"""
import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from .models import Cell, Intent, Pose2D

# 8-connected with true diagonal cost.
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, 1.4142136), (-1, 1, 1.4142136),
              (1, -1, 1.4142136), (1, 1, 1.4142136)]


def octile(a: Cell, b: Cell) -> float:
    """Admissible heuristic for 8-connected grids with sqrt(2) diagonals."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (1.4142136 - 2.0) * min(dx, dy)


def astar(gridmap, start: Cell, goal: Cell,
          blocked: Optional[Set[Cell]] = None,
          extra_cost: Optional[Dict[Cell, float]] = None,
          max_expansions: int = 20000) -> List[Cell]:
    """Shortest path start -> goal, or [] if none exists.

    blocked    : hard dynamic obstacles (from MapUpdate merging)
    extra_cost : SOFT penalties per cell. This is where congestion awareness
                 enters: adding ~0.4 per cell a peer plans to occupy soon makes
                 robots spread across aisles BEFORE any explicit negotiation is
                 needed. A robot that never enters the busy aisle never has to
                 argue about it, and that is a large part of the throughput
                 gain over stop-and-wait.
    max_expansions : safety valve so a pathological map cannot hang the tick.
    """
    blocked = blocked or set()
    extra_cost = extra_cost or {}

    def free(c: Cell) -> bool:
        return gridmap.is_static_free(c) and c not in blocked

    if not free(start) or not free(goal):
        return []
    if start == goal:
        return [start]

    open_heap: List[Tuple[float, float, Cell]] = [(octile(start, goal), 0.0, start)]
    came: Dict[Cell, Cell] = {}
    g: Dict[Cell, float] = {start: 0.0}
    closed: Set[Cell] = set()
    expansions = 0

    while open_heap:
        _, gc, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expansions += 1
        if expansions > max_expansions:
            return []

        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]

        for dr, dc, step in NEIGHBOURS:
            nxt = (cur[0] + dr, cur[1] + dc)
            if nxt in closed or not free(nxt):
                continue
            # Forbid corner cutting. Without this the path slips diagonally
            # between two rack corners and the real robot clips a rack leg.
            if dr and dc:
                if not free((cur[0] + dr, cur[1])) or not free((cur[0], cur[1] + dc)):
                    continue
            ng = gc + step + extra_cost.get(nxt, 0.0)
            if ng < g.get(nxt, float("inf")):
                g[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_heap, (ng + octile(nxt, goal), ng, nxt))
    return []


def path_length_m(gridmap, path: List[Cell]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += gridmap.cell_distance_m(a, b)
    return total


def congestion_costs(peer_intents, now: float, horizon: float = 10.0,
                     penalty: float = 0.4) -> Dict[Cell, float]:
    """Soft cost from peers' broadcast plans. Feeds astar(extra_cost=...)."""
    out: Dict[Cell, float] = {}
    for intent in peer_intents:
        for cell, t_in, _t_out in intent.triples():
            if t_in <= now + horizon:
                out[cell] = out.get(cell, 0.0) + penalty
    return out


def build_intent(gridmap, path: List[Cell], v_nominal: float, now: float,
                 turn_penalty_s: float = 0.5,
                 safety_margin_s: float = 0.5,
                 goal: Optional[Pose2D] = None) -> Intent:
    """Convert a geometric path into a time-stamped occupancy claim.

    safety_margin_s widens every cell window at BOTH ends. It absorbs clock
    skew between robots, control lag, and prediction error. Start at 0.5 s.
    If you observe near-misses, raise it; if throughput suffers badly, lower it
    and measure. Tune it empirically and REPORT the tuning - that is exactly
    the kind of detail that distinguishes a serious project.
    """
    intent = Intent(goal=goal or Pose2D())
    if not path:
        return intent

    t = now
    cell_time = gridmap.resolution / max(0.05, v_nominal)

    for i, cell in enumerate(path):
        travel = cell_time
        # Turning costs real time and is the main source of ETA error.
        if 0 < i < len(path) - 1:
            a, b, c = path[i - 1], path[i], path[i + 1]
            if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                travel += turn_penalty_s
        intent.cells.append(cell)
        intent.t_enter.append(t - safety_margin_s)
        intent.t_exit.append(t + travel + safety_margin_s)
        t += travel

    intent.zones = gridmap.zones_on_path(path)
    intent.active_zone_index = 0 if intent.zones else -1
    return intent
