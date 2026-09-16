"""Wait-for graph cycle detection and unanimous victim selection."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.deadlock import DeadlockManager
from amr_fleet.core.models import RobotState, WAITING, MOVING
from amr_fleet.core.priority import select_deadlock_victim


def test_two_cycle():
    g = {"robot_1": "robot_2", "robot_2": "robot_1"}
    assert DeadlockManager.find_cycle(g, "robot_1") == ["robot_1", "robot_2"]


def test_three_cycle_from_every_member():
    """R1 -> R2 -> R3 -> R1. Every robot must find the same cycle members."""
    g = {"robot_1": "robot_2", "robot_2": "robot_3", "robot_3": "robot_1"}
    for start in g:
        cyc = DeadlockManager.find_cycle(g, start)
        assert cyc is not None and set(cyc) == {"robot_1", "robot_2", "robot_3"}


def test_chain_is_not_a_cycle():
    """R1 -> R2 -> R3 (R3 waits for nobody) is congestion, not deadlock."""
    g = {"robot_1": "robot_2", "robot_2": "robot_3"}
    assert DeadlockManager.find_cycle(g, "robot_1") is None


def test_cycle_not_containing_me_is_ignored():
    """R1 -> R2 -> R3 -> R2. R1 is not IN the cycle; it is someone else's
    problem, and R1 must not appoint itself a victim."""
    g = {"robot_1": "robot_2", "robot_2": "robot_3", "robot_3": "robot_2"}
    assert DeadlockManager.find_cycle(g, "robot_1") is None


def test_all_members_pick_same_victim():
    cycle = ["robot_1", "robot_2", "robot_3"]
    scores = {"robot_1": 0.8, "robot_2": 0.3, "robot_3": 0.6}
    picks = {select_deadlock_victim(cycle, scores) for _ in range(5)}
    assert picks == {"robot_2"}


def test_victim_tie_broken_deterministically():
    cycle = ["robot_3", "robot_1", "robot_2"]
    scores = {"robot_1": 0.5, "robot_2": 0.5, "robot_3": 0.5}
    assert select_deadlock_victim(cycle, scores) == "robot_3"


def test_transient_cycle_ignored():
    """Two robots that both just sent a ZoneRequest briefly point at each
    other. That resolves itself in one round trip and must NOT trigger a
    reroute."""
    dm = DeadlockManager("robot_1")
    me = RobotState(robot_id="robot_1", status=WAITING, waiting_for="robot_2",
                    priority_score=0.3)
    peer = RobotState(robot_id="robot_2", status=WAITING, waiting_for="robot_1",
                      priority_score=0.7)
    assert dm.tick(me, {"robot_2": peer}, now=100.0) is None      # t = 0
    assert dm.tick(me, {"robot_2": peer}, now=100.3) is None      # t = 0.3 s
    out = dm.tick(me, {"robot_2": peer}, now=101.5)               # t = 1.5 s
    assert out is not None and out["role"] == "VICTIM"            # lower score


def test_timeout_fires_without_structural_cycle():
    """The safety net: a peer crashed holding a zone, so no cycle ever forms,
    but we must still recover."""
    dm = DeadlockManager("robot_1")
    me = RobotState(robot_id="robot_1", status=WAITING, waiting_for="ghost",
                    priority_score=0.5)
    assert dm.tick(me, {}, now=0.0) is None
    out = dm.tick(me, {}, now=9.0)
    assert out is not None and out["role"] == "VICTIM"


def test_metrics_record_recovery_time():
    dm = DeadlockManager("robot_1")
    me = RobotState(robot_id="robot_1", status=WAITING, waiting_for="robot_2",
                    priority_score=0.1)
    peer = RobotState(robot_id="robot_2", status=WAITING, waiting_for="robot_1",
                      priority_score=0.9)
    dm.tick(me, {"robot_2": peer}, now=0.0)
    dm.tick(me, {"robot_2": peer}, now=2.0)
    dm.mark_resolved(now=3.5)
    m = dm.metrics()
    assert m["deadlocks_detected"] == 1 and m["deadlocks_resolved"] == 1
    assert m["mean_recovery_s"] > 0
