"""
Deterministic priority scoring and total ordering.

THE REQUIREMENT: if robot_1 and robot_2 observe the same conflict, they must
independently compute the same winner. No randomness, no wall-clock tie-breaks,
no dependence on message arrival order.

HOW DETERMINISM IS GUARANTEED
1. Every input comes from a broadcast field. Nothing local is used.
2. Scores are rounded to a fixed number of decimals before any comparison.
   Float arithmetic can differ in the last bits between an x86 laptop and an
   ARM Jetson; rounding removes that as a source of disagreement.
3. Ties break on (score DESC, lamport ASC, robot_id ASC) - a TOTAL order, so
   two robots can never both believe they won.
"""
from typing import Dict, Iterable, Optional, Tuple

from .models import RobotState

# ---------------------------------------------------------------- constants
# Normalisation constants. Exposed so they can be tuned from YAML, but the
# SAME values must be used on every robot or determinism is lost.
DEFAULT_WEIGHTS = {
    "task_urgency": 0.30,
    "waiting_time": 0.30,   # tied for largest ON PURPOSE - see below
    "battery": 0.15,
    "distance": 0.15,
    "commitment": 0.10,
}
WAIT_SATURATION_S = 15.0    # waiting_time at which the aging term maxes out
DIST_SATURATION_M = 30.0    # distance-to-goal at which that term bottoms out
MAX_TASK_PRIORITY = 3
SCORE_DECIMALS = 6          # rounding precision - MUST match on all robots

# Escape hatches for pathological cases.
HARD_WAIT_CEILING_S = 20.0  # beyond this, score is forced to 1.0
CONSECUTIVE_WIN_CAP = 3     # after N wins on one zone, take a one-round penalty
CONSECUTIVE_WIN_PENALTY = 0.25


def priority_score(state: RobotState,
                   dist_to_goal_m: float,
                   in_contested_zone: bool,
                   weights: Optional[Dict[str, float]] = None) -> float:
    """Compute a robot's priority in [0, 1]. Higher wins.

    Each term, and why it is there:

    task_urgency  - an urgent task should not queue behind a routine one.

    waiting_time  - grows without bound while a robot waits, and resets to 0
                    the moment it moves. THIS IS THE ANTI-STARVATION MECHANISM
                    and it is why the weight is joint-largest. See the proof in
                    docs/ALGORITHMS.md.

    battery       - LOW battery means HIGH priority. A robot that strands
                    itself mid-aisle becomes a permanent obstacle for everyone,
                    which costs far more than letting it through first.

    distance      - a robot nearly at its goal is favoured. It finishes sooner
                    and frees the contested space sooner, which measurably
                    lowers mean task completion time.

    commitment    - a robot already inside the contested zone gets a small
                    bonus. Reversing a committed robot is slow and risky, and
                    without this term the system thrashes between two robots
                    that keep swapping the lead.
    """
    w = weights or DEFAULT_WEIGHTS

    # Hard ceiling: something is wrong with tuning, but nobody starves.
    if state.waiting_time >= HARD_WAIT_CEILING_S:
        return 1.0

    u = min(state.task_priority, MAX_TASK_PRIORITY) / float(MAX_TASK_PRIORITY)
    t = min(state.waiting_time / WAIT_SATURATION_S, 1.0)
    b = 1.0 - max(0.0, min(state.battery_pct, 100.0)) / 100.0
    d = 1.0 - min(max(dist_to_goal_m, 0.0) / DIST_SATURATION_M, 1.0)
    c = 1.0 if in_contested_zone else 0.0

    raw = (w["task_urgency"] * u + w["waiting_time"] * t + w["battery"] * b +
           w["distance"] * d + w["commitment"] * c)

    # Rounding is load-bearing, not cosmetic. Do not remove.
    return round(max(0.0, min(raw, 1.0)), SCORE_DECIMALS)


def order_key(score: float, lamport: int, robot_id: str) -> Tuple:
    """Sort key producing a TOTAL order. Sorting ascending puts the WINNER first.

    - -score   : higher score first
    -  lamport : earlier logical request first (causal fairness)
    -  robot_id: lexicographic, the final deterministic tie-break
    """
    return (-round(score, SCORE_DECIMALS), lamport, robot_id)


def wins(a: Tuple[float, int, str], b: Tuple[float, int, str]) -> bool:
    """True if candidate a beats candidate b. Each tuple is (score, lamport, id).

    Strict: wins(a, b) and wins(b, a) can never both be true, because robot_id
    is unique. That property is what makes the zone protocol safe.
    """
    return order_key(*a) < order_key(*b)


def rank(candidates: Dict[str, Tuple[float, int]]) -> list:
    """candidates: {robot_id: (score, lamport)} -> [robot_id] best first."""
    return sorted(candidates.keys(),
                  key=lambda rid: order_key(candidates[rid][0],
                                            candidates[rid][1], rid))


def select_winner(candidates: Dict[str, Tuple[float, int]]) -> Optional[str]:
    ordered = rank(candidates)
    return ordered[0] if ordered else None


def select_deadlock_victim(cycle: Iterable[str],
                           scores: Dict[str, float]) -> Optional[str]:
    """Pick the robot that yields: the LOWEST priority in the cycle.

    Uses the same total order reversed, so every robot in the cycle picks the
    same victim with no negotiation round at all.
    """
    members = list(cycle)
    if not members:
        return None
    return max(members,
               key=lambda rid: order_key(scores.get(rid, 0.0), 0, rid))
