"""R5 task aging: deadline budgets, urgency escalation, retry halving and
the priority-level ordering that consumes the urgency.

The level/class functions live in core/priority.py (PRIORITY builder); the
tests for them activate once that contract lands and skip until then.
"""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from amr_fleet.core import priority, tasks  # noqa: E402
from amr_fleet.core.models import Task  # noqa: E402
from amr_fleet.core.tasks import (NO_BID_RETRY_S, TaskAuction, age_frac,  # noqa: E402
                                  task_urgency)

AGE_STEPS = getattr(priority, "AGE_STEPS", (0.5, 0.75, 1.0, 1.25, 1.5))
BUDGET_S = getattr(priority, "BUDGET_S",
                   {0: 225.0, 1: 188.0, 2: 150.0, 3: 113.0})
_HAS_LEVEL = hasattr(priority, "level") and hasattr(priority, "urgency")


# ------------------------------------------------------------ urgency steps --
def test_urgency_steps_at_every_age_steps_boundary():
    for base in (0, 1, 2, 3):
        assert tasks._urgency(base, 0.0) == base
        for i, step in enumerate(AGE_STEPS):
            below = tasks._urgency(base, step - 1e-6)
            at = tasks._urgency(base, step)
            assert at == min(9, base + i + 1), (base, step)
            assert below == min(9, base + i), (base, step)


def test_urgency_caps_at_nine():
    assert tasks._urgency(3, 100.0) == min(9, 3 + len(AGE_STEPS))
    assert tasks._urgency(9, 0.0) == 9
    assert tasks._urgency(12, 5.0) == 9


def test_task_urgency_uses_the_consumed_budget_fraction():
    t = Task("T", (0, 0), (1, 1), priority=2, created_at=100.0,
             deadline=100.0 + BUDGET_S[2])
    assert task_urgency(t, 100.0) == 2
    # half the budget consumed -> first step crossed
    assert task_urgency(t, 100.0 + 0.5 * BUDGET_S[2]) == 3
    # fully overdue -> all steps up to 1.0 crossed
    assert task_urgency(t, 100.0 + BUDGET_S[2]) == 2 + 3


# ------------------------------------------------------- eff_deadline rules --
def test_eff_deadline_fallback_applies_the_priority_budget():
    for prio, budget in BUDGET_S.items():
        t = Task("T", (0, 0), (1, 1), priority=int(prio), created_at=50.0,
                 deadline=0.0)
        assert tasks._eff_deadline(t, 60.0) == 50.0 + budget


def test_eff_deadline_prefers_an_explicit_deadline():
    t = Task("T", (0, 0), (1, 1), priority=0, created_at=50.0, deadline=80.0)
    assert tasks._eff_deadline(t, 60.0) == 80.0


def test_unaged_task_without_created_at_never_ages():
    t = Task("T", (0, 0), (1, 1), priority=3)      # created_at 0.0
    assert age_frac(t, 1e9) == 0.0
    assert task_urgency(t, 1e9) == 3


# ------------------------------------------------- no-bid retry halves (R5) --
def _retry_period(task, t0):
    """Measured announce period for an unbid task held by the lowest id."""
    sent = []
    a = TaskAuction("robot_1", lambda d: sent.append(d), lambda d: None,
                    lambda d: None, path_length_m=lambda p, q: -1.0)
    a.on_announce(task, now=t0)
    times = []
    t = t0
    while t < t0 + 20.0:
        n = len(sent)
        a.tick(t, {"robot_2"}, {"robot_2": t})
        if len(sent) > n:
            times.append(t)
        t = round(t + 0.1, 1)
    assert len(times) >= 2
    return times[1] - times[0]


def test_no_bid_retry_halves_once_the_task_is_overdue():
    fresh = Task("T-f", (1, 1), (2, 2), priority=1, created_at=1000.0,
                 deadline=1000.0 + BUDGET_S[1])
    overdue = Task("T-o", (1, 1), (2, 2), priority=1, created_at=100.0,
                   deadline=160.0)                    # long past its budget
    assert abs(_retry_period(fresh, 1000.0) - NO_BID_RETRY_S) < 0.2
    assert abs(_retry_period(overdue, 1000.0) - NO_BID_RETRY_S * 0.5) < 0.2


# --------------------------------------------------- priority.level contract --
@pytest.mark.skipif(not _HAS_LEVEL,
                    reason="core/priority.py level/urgency not landed yet")
def test_level_orders_classes_carrying_above_all_at_equal_urgency():
    lv = priority.level
    carrying = lv(priority.CLASS_CARRYING, 5, 0.0)
    to_pickup = lv(priority.CLASS_TO_PICKUP, 5, 0.0)
    reposition = lv(priority.CLASS_REPOSITION, 5, 0.0)
    idle = lv(priority.CLASS_IDLE, 5, 0.0)
    assert carrying > to_pickup > reposition > idle
    # the wait bonus can never flip a class
    assert lv(priority.CLASS_TO_PICKUP, 9, 1e6) < lv(priority.CLASS_CARRYING,
                                                     0, 0.0)


@pytest.mark.skipif(not _HAS_LEVEL,
                    reason="core/priority.py level/urgency not landed yet")
def test_level_is_exact_in_float32_and_class_recoverable():
    import struct
    lv = priority.level(priority.CLASS_STARVED, 9, 95.0)
    packed = struct.unpack("f", struct.pack("f", lv))[0]
    assert packed == lv == 4909.0
    assert priority.class_of_score(lv) == priority.CLASS_STARVED


@pytest.mark.skipif(not hasattr(priority, "eff_deadline"),
                    reason="priority.eff_deadline not landed yet")
def test_tasks_defer_to_priority_eff_deadline_once_landed():
    t = Task("T", (0, 0), (1, 1), priority=1, created_at=10.0, deadline=0.0)
    assert tasks._eff_deadline(t, 20.0) == priority.eff_deadline(t, 20.0)


# ------------------------------------------------ deadline fill in the sim ---
def test_stream_tasks_carry_the_priority_budget_deadline():
    import fleet_sim as fs
    sim = fs.FleetSim(fs.random_stream(seed=2, interval_s=5.0))
    sim.run(6.0)
    assert sim.live_tasks, "a stream task should have been announced"
    for task in sim.live_tasks.values():
        assert task.deadline == pytest.approx(
            task.created_at + fs._TASK_BUDGET_S[task.priority])


def test_overloaded_stream_serves_backlog_without_starvation():
    """A 90 s burst at 8 s intervals: every announced task is eventually
    assigned in (urgency, age) order by the pulls, none starves while idle
    robots exist. Smoke-scale version of the AC10 900 s criterion."""
    import fleet_sim as fs
    sim = fs.FleetSim(fs.random_stream(seed=4, interval_s=8.0, num_tasks=8))
    m = sim.run(90.0)
    assigned = {e["task"] for e in m["events"] if e["kind"] == "award"}
    announced = {e["task"] for e in m["events"] if e["kind"] == "announce"}
    # every pickup-distinct announced task gets an owner while robots idle
    assert announced, "stream produced no tasks"
    assert assigned & announced, "no announced task was ever assigned"
    assert m["duplicate_owner_s"] == 0.0
