"""Quantized priority level L (increment 3, R1/R5).

Pins the frozen contract: L = 1000*C + 100*U + W, exact in float32; a TOTAL
order (L desc, robot_id asc); C from the NODE-set task phase, never from
goal_cell; STARVED promotion; deadline aging.
"""
import math
import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import priority as prio                     # noqa: E402
from amr_fleet.core.coordinator import FleetCoordinator          # noqa: E402
from amr_fleet.core.gridmap import GridMap                       # noqa: E402
from amr_fleet.core.models import Pose2D, Task                   # noqa: E402

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
NOW = 100.0


def _coord(rid="robot_1", cell=(10, 30)):
    c = FleetCoordinator(rid, GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world(cell), 0.0)
    return c


# ----------------------------------------------------------------- the key
def test_level_formula_and_class_recovery():
    assert prio.level(0, 0, 0.0) == 0.0
    assert prio.level(3, 2, 25.0) == 3202.0
    assert prio.level(4, 9, 999.0) == 4909.0          # W saturates at 9
    for cls in range(5):
        for urg in (0, 5, 9):
            for w in (0.0, 15.0, 95.0):
                L = prio.level(cls, urg, w)
                assert prio.class_of_score(L) == cls


def test_level_is_exact_in_float32():
    """RobotState.priority_score is a float32 on the wire: every reachable L
    must survive the round trip bit-exactly."""
    for cls in range(5):
        for urg in range(10):
            for w9 in range(10):
                L = prio.level(cls, urg, w9 * prio.WAIT_STEP_S)
                (rt,) = struct.unpack("<f", struct.pack("<f", L))
                assert rt == L


def test_measured_mutual_hold_case_agrees_under_quantization():
    """The measured bug: continuous scores 0.271 vs 0.269 - each robot's own
    score advanced (waiting-time aging) between broadcast and comparison, so
    each compared its FRESH score against the other's STALE broadcast and
    both concluded they won. With L both robots compare the same two
    broadcast integers, so exactly one wins and both agree."""
    # The legacy continuous pair disagrees when each side uses a slightly
    # fresher own-score (what the field logs showed):
    a_stale, b_stale = 0.269, 0.271
    a_fresh, b_fresh = 0.272, 0.274       # both aged a hair before comparing
    a_thinks_a = (-a_fresh, "robot_1") < (-b_stale, "robot_2")
    b_thinks_b = (-b_fresh, "robot_2") < (-a_stale, "robot_1")
    assert a_thinks_a and b_thinks_b, "the continuous bug reproduces"
    # Quantized: the same situation lands on the same L (same class, same
    # urgency, wait inside one WAIT_STEP_S bucket) on both robots' copies.
    La = prio.level(2, 0, 12.0)
    Lb = prio.level(2, 0, 14.0)           # 2 s more waiting, same bucket
    assert La == Lb
    a_wins_on_a = prio.outranks(La, "robot_1", Lb, "robot_2")
    a_wins_on_b = prio.outranks(La, "robot_1", Lb, "robot_2")
    assert a_wins_on_a == a_wins_on_b     # both evaluate the same expression
    assert a_wins_on_a and not prio.outranks(Lb, "robot_2", La, "robot_1")


def test_total_order_antisymmetric_and_ties_by_id():
    cases = [(3202.0, "robot_1", 3202.0, "robot_2"),
             (2000.0, "robot_3", 2100.0, "robot_1"),
             (0.0, "robot_2", 0.0, "robot_2")]
    for sa, ia, sb, ib in cases:
        both = (prio.outranks(sa, ia, sb, ib)
                and prio.outranks(sb, ib, sa, ia))
        assert not both
    assert prio.outranks(1000.0, "robot_1", 1000.0, "robot_2")


def test_pick_victim_lowest_level_ties_higher_id_yields():
    assert prio.pick_victim({"robot_1": 3000.0, "robot_2": 1000.0,
                             "robot_3": 2000.0}) == "robot_2"
    assert prio.pick_victim({"robot_1": 2000.0, "robot_2": 2000.0,
                             "robot_3": 2000.0}) == "robot_3"


# ------------------------------------------------------------ urgency (R5)
def test_urgency_ages_through_the_steps_and_saturates():
    assert prio.urgency(0, 0.0) == 0
    assert prio.urgency(0, 0.5) == 1
    assert prio.urgency(0, 1.5) == 5
    assert prio.urgency(3, 0.76) == 5          # 3 + two steps passed
    assert prio.urgency(9, 2.0) == 9           # saturates
    assert prio.urgency(0, 0.49) == 0


def test_eff_deadline_budget_fallback():
    t = Task("T1", (88, 25), (22, 30), priority=2, created_at=50.0,
             deadline=0.0)
    assert prio.eff_deadline(t, NOW) == 50.0 + prio.BUDGET_S[2]
    t.deadline = 300.0
    assert prio.eff_deadline(t, NOW) == 300.0
    t.created_at = 0.0
    assert math.isinf(prio.eff_deadline(t, NOW))


# -------------------------------------------- coordinator: class from phase
def test_class_comes_from_task_phase_never_goal_cell():
    """The measured 195 s mutual stall came from inferring CARRYING from
    goal_cell. A goal alone is REPOSITION (1); only the node-set phase gives
    TO_PICKUP (2) or CARRYING (3)."""
    c = _coord()
    c.tick(NOW)
    assert prio.class_of_score(c.state.priority_score) == prio.CLASS_IDLE
    assert c.set_goal((22, 60), now=NOW)
    c.tick(NOW + 0.1)
    assert prio.class_of_score(c.state.priority_score) \
        == prio.CLASS_REPOSITION
    c.task_phase = "PICKUP"
    c.tick(NOW + 0.2)
    assert prio.class_of_score(c.state.priority_score) \
        == prio.CLASS_TO_PICKUP
    c.task_phase = "DROPOFF"
    c.tick(NOW + 0.3)
    assert prio.class_of_score(c.state.priority_score) \
        == prio.CLASS_CARRYING
    # Phase outweighs goal: DROPOFF with no goal is still CARRYING.
    c.clear_goal()
    c.tick(NOW + 0.4)
    assert prio.class_of_score(c.state.priority_score) \
        == prio.CLASS_CARRYING


def test_starvation_promotes_after_45s_and_clears_on_movement():
    c = _coord()
    c.task_phase = "PICKUP"
    c.state.waiting_time = prio.STARVE_S + 1.0     # continuous wait
    c.tick(NOW)
    assert prio.class_of_score(c.state.priority_score) \
        == prio.CLASS_STARVED
    # Still starved while it has not moved STARVE_CLEAR_M...
    c.state.waiting_time = 0.0
    c.state.pose = Pose2D(c.state.pose.x + 0.1, c.state.pose.y, 0.0)
    c.tick(NOW + 1.0)
    assert prio.class_of_score(c.state.priority_score) \
        == prio.CLASS_STARVED
    # ...cleared once it has.
    c.state.pose = Pose2D(c.state.pose.x + prio.STARVE_CLEAR_M,
                          c.state.pose.y, 0.0)
    c.tick(NOW + 2.0)
    assert prio.class_of_score(c.state.priority_score) \
        == prio.CLASS_TO_PICKUP


def test_task_aging_raises_urgency_component():
    c = _coord()
    c.task_phase = "PICKUP"
    c.current_task = Task("T1", (88, 25), (22, 30), priority=1,
                          created_at=NOW, deadline=NOW + 100.0)
    c.tick(NOW + 1.0)
    early = c.state.priority_score
    c.tick(NOW + 90.0)                       # age_frac 0.9 -> +2 steps
    late = c.state.priority_score
    assert prio.class_of_score(early) == prio.class_of_score(late) == 2
    assert late > early


def test_legacy_priority_score_still_available():
    from amr_fleet.core.priority import priority_score, order_key, wins
    from amr_fleet.core.models import RobotState
    s = priority_score(RobotState(robot_id="r"), 10.0, False)
    assert 0.0 <= s <= 1.0
    assert wins((0.9, 1, "a"), (0.1, 1, "b"))
    assert order_key(0.5, 1, "a") < order_key(0.5, 1, "b")
