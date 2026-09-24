"""Conflict detection, including the swap case that cell-overlap alone misses."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.conflict import ConflictDetector
from amr_fleet.core.geometry import safety_distance, time_to_collision
from amr_fleet.core.models import Intent, Pose2D, RobotState


def test_head_on_collision_predicted():
    """Closing at 1 m/s from 4 m apart.

    Contact distance D = 2*0.16 + 0.15 = 0.47 m, so first contact is at
    (4.0 - 0.47) / 1.0 = 3.53 s. Checked against hand arithmetic, not against
    whatever the code happens to return.
    """
    t, d = time_to_collision(0, 0, 0.5, 0, 4.0, 0, -0.5, 0)
    assert t is not None
    assert abs(t - 3.53) < 0.05, f"expected ~3.53 s, got {t}"


def test_separating_robots_never_conflict():
    t, _ = time_to_collision(0, 0, -0.5, 0, 4.0, 0, 0.5, 0)
    assert t is None


def test_parallel_robots_no_division_by_zero():
    """Identical velocities -> zero relative velocity. Must not crash."""
    t, d = time_to_collision(0, 0, 0.5, 0, 0, 3.0, 0.5, 0)
    assert t is None and abs(d - 3.0) < 1e-6


def test_crossing_paths_detected():
    t, _ = time_to_collision(0, -3, 0, 0.5, 0, 3, 0, -0.5)
    assert t is not None


def test_already_touching():
    t, _ = time_to_collision(0, 0, 0, 0, 0.2, 0, 0, 0)
    assert t == 0.0


def test_safety_distance_grows_with_speed():
    assert safety_distance(1.5) > safety_distance(0.5) > safety_distance(0.0)


def _state(rid, x, y, cells, t0=0.0, step=1.0):
    s = RobotState(robot_id=rid, pose=Pose2D(x, y, 0.0))
    s.intent = Intent(cells=list(cells),
                      t_enter=[t0 + i * step for i in range(len(cells))],
                      t_exit=[t0 + (i + 1) * step for i in range(len(cells))])
    return s


def test_cell_conflict_detected():
    det = ConflictDetector()
    me = _state("robot_1", 0, 0, [(5, 5), (5, 6), (5, 7)])
    peer = _state("robot_2", 3, 0, [(5, 7), (5, 6), (5, 5)], t0=0.5)
    out = det.detect(me, me.intent, {"robot_2": peer}, now=0.0)
    assert any(c.kind == "CELL" for c in out)


def test_swap_conflict_detected():
    """THE CASE CELL-OVERLAP CAN MISS.

    Robot 1 goes A->B while robot 2 goes B->A. If their timings interleave,
    no single cell has overlapping windows, yet they collide head-on.
    """
    det = ConflictDetector()
    me = _state("robot_1", 0, 0, [(5, 5), (5, 6)], t0=0.0)
    peer = _state("robot_2", 1, 0, [(5, 6), (5, 5)], t0=10.0)   # no time overlap
    out = det.detect(me, me.intent, {"robot_2": peer}, now=0.0)
    kinds = {c.kind for c in out}
    assert "SWAP" in kinds, f"swap not detected, only got {kinds}"


def test_zone_conflict_detected():
    det = ConflictDetector()
    me = _state("robot_1", 0, 0, [(1, 1)])
    me.intent.zones = ["inter_X1"]
    peer = _state("robot_2", 5, 5, [(9, 9)])
    peer.intent.zones = ["inter_X1"]
    out = det.detect(me, me.intent, {"robot_2": peer}, now=0.0)
    assert any(c.kind == "ZONE" and c.zone_id == "inter_X1" for c in out)


def test_dead_peer_ignored():
    det = ConflictDetector()
    me = _state("robot_1", 0, 0, [(5, 5)])
    peer = _state("robot_2", 0.3, 0, [(5, 5)])
    peer.alive = False
    assert det.detect(me, me.intent, {"robot_2": peer}, now=0.0) == []


def test_conflict_priority_flag_is_deterministic():
    det = ConflictDetector()
    me = _state("robot_1", 0, 0, [(5, 5)], t0=0.0)
    peer = _state("robot_2", 0.2, 0, [(5, 5)], t0=0.0)
    me.priority_score = 0.8
    peer.priority_score = 0.2
    out = det.detect(me, me.intent, {"robot_2": peer}, now=0.0)
    assert any(c.i_have_priority for c in out if c.peer_id == "robot_2")

