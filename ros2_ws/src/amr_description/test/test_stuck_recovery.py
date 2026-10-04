"""Stuck-with-GO self-recovery + actionable deadlock victims
(webots_strictlanes1: robot_1 held GO nose-to-wall forever while the
deadlock detector elected it victim 1868 times with no effect).
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.coordinator import (STUCK_SELF_RECOVER_S,
                                        FleetCoordinator)
from amr_fleet.core.deadlock import DeadlockManager
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import MOVING, WAITING, Pose2D, RobotState, Task

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fleet_sim as fs                                        # noqa: E402

GRID_YAML = str(pathlib.Path(__file__).resolve().parents[1]
                / "config" / "warehouse_grid.yaml")
NOW = 500.0


# ------------------------------------------- actionable victims (deadlock)
def test_non_waiting_blocker_is_never_elected_victim():
    """F3: robot_3 waited on robot_1, which held GO (status MOVING, v=0).
    Electing robot_1 is a no-op forever - its own detector only acts while
    WAITING - so the group must collapse to [me] and *I* get the victim
    role (an actionable one)."""
    dm = DeadlockManager("robot_3")
    me = RobotState(robot_id="robot_3", status=WAITING,
                    waiting_for="robot_1", priority_score=2000.0)
    blocker = RobotState(robot_id="robot_1", status=MOVING,
                         priority_score=1000.0)      # would win the old vote
    assert dm.tick(me, {"robot_1": blocker}, now=0.0) is None
    out = dm.tick(me, {"robot_1": blocker}, now=9.0)
    assert out is not None and out["role"] == "VICTIM"
    assert out["victim"] == "robot_3"
    assert out["cycle"] == ["robot_3"]


def test_moving_blocker_keeps_the_pair_election():
    """A DRIVING non-waiting blocker clears my wait by itself: the old pair
    semantics stay (self-victimising against it measurably cost deliveries)."""
    dm = DeadlockManager("robot_3")
    me = RobotState(robot_id="robot_3", status=WAITING,
                    waiting_for="robot_1", priority_score=2000.0)
    blocker = RobotState(robot_id="robot_1", status=MOVING,
                         priority_score=1000.0)
    blocker.v = 0.4
    assert dm.tick(me, {"robot_1": blocker}, now=0.0) is None
    out = dm.tick(me, {"robot_1": blocker}, now=9.0)
    assert out is not None and out["role"] == "HOLD"
    assert out["victim"] == "robot_1"


def test_waiting_blocker_keeps_the_pair_election():
    """Control: a blocker that is itself WAITING is actionable and the pair
    is still resolved by priority (the CARRYING robot holds course)."""
    dm = DeadlockManager("robot_3")
    me = RobotState(robot_id="robot_3", status=WAITING,
                    waiting_for="robot_1", priority_score=2000.0)
    blocker = RobotState(robot_id="robot_1", status=WAITING,
                         priority_score=1000.0)
    assert dm.tick(me, {"robot_1": blocker}, now=0.0) is None
    out = dm.tick(me, {"robot_1": blocker}, now=9.0)
    assert out is not None and out["role"] == "HOLD"
    assert out["victim"] == "robot_1"


def test_victim_election_log_is_rate_limited():
    """1868 identical 'DEADLOCK timeout victim=' lines at 10 Hz: an
    unchanged election now logs once, then at most every LOG_EVERY_S."""
    lines = []
    dm = DeadlockManager("robot_3", logger=lines.append)
    me = RobotState(robot_id="robot_3", status=WAITING,
                    waiting_for="robot_1", priority_score=2000.0)
    blocker = RobotState(robot_id="robot_1", status=WAITING,
                         priority_score=1000.0)
    assert dm.tick(me, {"robot_1": blocker}, now=0.0) is None
    t = 8.1
    while t < 16.0:                         # ~79 ticks past the timeout
        dm.tick(me, {"robot_1": blocker}, now=t)
        t = round(t + 0.1, 1)
    timeouts = [l for l in lines if "DEADLOCK timeout" in l]
    assert 1 <= len(timeouts) <= 3, timeouts


# --------------------------------------------- stuck-with-GO self-recovery
def _wall_stuck_coord():
    grid = GridMap.from_yaml(GRID_YAML)
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    # The F1 pose class: nose at the north wall after the J_N left turn.
    c.state.pose = Pose2D(0.45, 4.35, math.pi / 2)
    c.state.loc_sigma_lat = 0.05
    c.state.v = c.state.w = 0.0
    return c


def test_wall_stuck_go_robot_replans_then_becomes_waiting():
    """GO + frozen pose + a wall inside the nose cone + NO peer: within
    STUCK_SELF_RECOVER_S the robot replans away from the obstruction (new
    route), and if still frozen it degrades to WAITING so the deadlock
    machinery finally owns an ACTIONABLE victim (itself)."""
    logs = []
    c = _wall_stuck_coord()
    c.log = logs.append
    assert c.set_goal(c.grid.world_to_cell(-1.55, 4.45), now=NOW)
    first_path = list(c.path)
    t, saw_go, actions = NOW, False, []
    while t < NOW + 30.0:
        p = c.tick(t)
        actions.append(p.action)
        saw_go = saw_go or p.action in ("GO", "SLOW")
        t = round(t + 0.5, 1)
    assert saw_go, "precondition: the robot held a GO permit while frozen"
    assert any("stuck with GO" in m for m in logs), logs[-6:]
    assert list(c.path) != first_path or "STOP" in actions
    # Once degraded to WAITING, the deadlock timeout elects *me* and the
    # yield/retreat machinery runs - the robot is no longer invisible.
    assert "STOP" in actions
    assert (c.deadlock.events
            or any("retreating to refuge" in m or "yield" in m.lower()
                   for m in logs)), logs[-8:]


def test_open_ground_frozen_go_robot_does_not_self_recover():
    """Guard: with nothing static in the nose cone a GO stall is a follower
    problem (Nav2's own progress/BackUp machinery), not a planner one - no
    spurious replans, no phantom WAITING."""
    grid = GridMap.from_yaml(GRID_YAML)
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(0.45, -2.0, math.pi / 2)    # open spine mouth
    c.state.loc_sigma_lat = 0.05
    logs = []
    c.log = logs.append
    assert c.set_goal((88, 94), now=NOW)
    t = NOW
    while t < NOW + 12.0:
        p = c.tick(t)
        t = round(t + 0.5, 1)
    assert p.action in ("GO", "SLOW")
    assert not any("stuck with GO" in m for m in logs)


def test_fleet_sim_wall_stuck_robot_becomes_actionable_within_15s():
    """The regression the task demands: a GO-but-stalled robot (motion
    zeroed) nose-to-rack recovers - i.e. its own machinery ACTS: the
    self-stuck log fires and its broadcast status leaves plain MOVING-with-
    GO within 15 s, so peers' deadlock graphs can finally see it."""
    robots = [fs.RobotSpec("robot_1", -1.0, 0.5, math.pi / 2, dock=False,
                           tasks=[(0.2, Task("ST-1", pickup=fs.L3,
                                             dropoff=fs.D1))])]
    sc = fs.Scenario(name="wall_stuck", robots=robots, seed=0, keep_log=True)
    sim = fs.FleetSim(sc)
    sim.by_id["robot_1"]._control = lambda: (0.0, 0.0)
    while sim.t < 15.0:
        sim.step()
    assert any("stuck with GO" in msg for _, _, msg in sim.logs), \
        [l for l in sim.logs][-8:]
    a = sim.by_id["robot_1"]
    assert a.coord.state.status == WAITING or a.permit.action == "STOP"
