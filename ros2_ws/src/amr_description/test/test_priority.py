"""Determinism of the priority system - the property the whole design rests on."""
import itertools
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.models import RobotState
from amr_fleet.core.priority import (order_key, priority_score, rank,
                                     select_deadlock_victim, select_winner, wins)


def mk(rid, prio=0, wait=0.0, batt=100.0):
    return RobotState(robot_id=rid, task_priority=prio,
                      waiting_time=wait, battery_pct=batt)


def test_score_is_bounded():
    for prio in range(4):
        for wait in (0.0, 7.5, 15.0):
            for batt in (0.0, 50.0, 100.0):
                s = priority_score(mk("r", prio, wait, batt), 10.0, False)
                assert 0.0 <= s <= 1.0


def test_score_is_repeatable():
    s1 = priority_score(mk("r", 2, 5.0, 40.0), 12.0, True)
    s2 = priority_score(mk("r", 2, 5.0, 40.0), 12.0, True)
    assert s1 == s2, "identical inputs must give bit-identical output"


def test_waiting_raises_priority_monotonically():
    """The anti-starvation property, verified rather than asserted."""
    prev = -1.0
    for wait in [0.0, 2.0, 5.0, 10.0, 15.0]:
        s = priority_score(mk("r", 1, wait, 80.0), 10.0, False)
        assert s >= prev, "waiting must never LOWER priority"
        prev = s


def test_low_battery_raises_priority():
    hi = priority_score(mk("r", 1, 0.0, 95.0), 10.0, False)
    lo = priority_score(mk("r", 1, 0.0, 10.0), 10.0, False)
    assert lo > hi, "a nearly-flat robot must outrank a full one"


def test_hard_ceiling_prevents_starvation():
    assert priority_score(mk("r", 0, 25.0, 100.0), 30.0, False) == 1.0


def test_total_order_is_strict():
    """wins(a,b) and wins(b,a) must NEVER both be true.

    If they could, two robots could each believe they won the same zone, and
    that is a collision.
    """
    cands = [(0.5, 1, "robot_1"), (0.5, 1, "robot_2"),
             (0.5, 2, "robot_1"), (0.7, 9, "robot_3")]
    for a, b in itertools.permutations(cands, 2):
        assert not (wins(a, b) and wins(b, a))


def test_ranking_independent_of_insertion_order():
    """Every robot receives peer messages in a DIFFERENT order. The ranking
    must not depend on that or robots will disagree."""
    c = {"robot_1": (0.61, 5), "robot_2": (0.61, 3), "robot_3": (0.44, 9)}
    expected = rank(c)
    for perm in itertools.permutations(c.items()):
        assert rank(dict(perm)) == expected


def test_id_breaks_exact_ties():
    c = {"robot_3": (0.5, 1), "robot_1": (0.5, 1), "robot_2": (0.5, 1)}
    assert select_winner(c) == "robot_1"


def test_deadlock_victim_is_unanimous():
    """Every robot in the cycle must pick the SAME victim independently."""
    cycle = ["robot_1", "robot_2", "robot_3"]
    scores = {"robot_1": 0.7, "robot_2": 0.5, "robot_3": 0.9}
    picks = {select_deadlock_victim(cycle[i:] + cycle[:i], scores)
             for i in range(len(cycle))}
    assert len(picks) == 1, f"robots disagreed on the victim: {picks}"
    assert picks.pop() == "robot_2"           # lowest score yields
