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


# ============================================================================
# QUANTIZED PRIORITY LEVEL L (increments 3+4, R1/R5).
#
# The continuous score above produced the measured 0.271-vs-0.269 mutual-HOLD:
# two robots each computed a score a hair above the other's BROADCAST score
# (waiting-time aging advanced between broadcast and comparison), so each
# believed it outranked the other and both held. L is integer-valued
# (1000*C + 100*U + W, exact in float32), so two robots comparing the same
# two broadcast values always agree, and ties break on robot_id.
#
#   C  class 0..4   - what the robot is doing (from the NODE-set task phase,
#                     never inferred from goal_cell: that inference carried a
#                     measured 195 s mutual stall)
#   U  urgency 0..9 - task priority plus deadline aging (R5)
#   W  wait 0..9    - floor(waiting_time / 10 s), the anti-starvation term
#
# Total order: L desc, robot_id asc. legacy priority_score() above is kept
# for the increment-1 tests and the baseline mode.
# ============================================================================
CLASS_IDLE = 0
CLASS_REPOSITION = 1
CLASS_TO_PICKUP = 2
CLASS_CARRYING = 3
CLASS_STARVED = 4
STARVE_S = 45.0            # continuous wait that promotes to CLASS_STARVED
STARVE_CLEAR_M = 0.3       # movement that clears the promotion
WAIT_STEP_S = 10.0
AGE_STEPS = (0.5, 0.75, 1.0, 1.25, 1.5)   # age_frac thresholds, one U each
BUDGET_S = {0: 225.0, 1: 188.0, 2: 150.0, 3: 113.0}


def urgency(task_priority: int, age_frac: float) -> int:
    """0..9: the task's own priority plus one step per AGE_STEPS threshold
    the task's age fraction (elapsed / budget) has passed."""
    steps = sum(1 for t in AGE_STEPS if age_frac >= t)
    return max(0, min(9, int(task_priority) + steps))


def level(cls: int, urg: int, wait_s: float) -> float:
    """L = 1000*C + 100*U + W. Integer-valued, exact in float32."""
    w = min(9, int(max(0.0, wait_s) // WAIT_STEP_S))
    return float(1000 * int(cls) + 100 * int(urg) + w)


def class_of_score(score: float) -> int:
    """Recover C from a broadcast L (clamped; a legacy [0,1] score reads 0)."""
    return max(0, min(4, int(score // 1000)))


def outranks(score_a: float, id_a: str, score_b: float, id_b: str) -> bool:
    """True if (a) outranks (b): L desc, robot_id asc - a TOTAL order."""
    return (-score_a, id_a) < (-score_b, id_b)


def pick_victim(members: Dict[str, float]) -> str:
    """Deadlock victim: order by (score desc, id asc); the LAST yields
    (lowest L; on ties the higher robot_id)."""
    ordered = sorted(members, key=lambda rid: (-members[rid], rid))
    return ordered[-1]


def eff_deadline(task, now: float) -> float:
    """The deadline used for aging: task.deadline when set, else
    created_at + BUDGET_S[priority]; +inf when the task has no created_at."""
    created = getattr(task, "created_at", 0.0) or 0.0
    if created <= 0.0:
        return float("inf")
    dl = getattr(task, "deadline", 0.0) or 0.0
    if dl > created:
        return dl
    prio = max(0, min(3, int(getattr(task, "priority", 0))))
    return created + BUDGET_S[prio]
