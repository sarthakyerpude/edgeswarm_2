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


def test_open_ground_frozen_go_robot_waits_for_the_phantom_patience():
    """With nothing static in the believed nose cone, a GO stall is first
    the follower's problem: NO replan before STUCK_PHANTOM_S (segment spins
    and SLOW-scaled turns are zero-translation GO spells of up to ~8 s).
    Past it, the hold is a PHANTOM (belief clear, physics stopped - the
    live rack-3/rack-6 wedge class, where belief-truth drift pressed the
    true body to a face the believed map cannot see) and the blocked-ahead
    replan must run anyway."""
    grid = GridMap.from_yaml(GRID_YAML)
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(0.45, -2.0, math.pi / 2)    # open spine mouth
    c.state.loc_sigma_lat = 0.05
    logs = []
    c.log = logs.append
    assert c.set_goal((88, 94), now=NOW)
    t = NOW
    while t < NOW + 11.5:
        p = c.tick(t)
        t = round(t + 0.5, 1)
    assert p.action in ("GO", "SLOW")
    assert not any("stuck with GO" in m for m in logs), \
        "no spurious replan inside the phantom patience"
    while t < NOW + 20.0:
        c.tick(t)
        t = round(t + 0.5, 1)
    assert any("stuck with GO" in m and "phantom hold" in m for m in logs), \
        logs[-6:]


def _phantom_cluster_case(pose, goal, name):
    """Live wedge-cluster regression core: frozen GO robot at the measured
    believed pose, believed nose ray CLEAR (that is what made the old
    machinery blind), must phantom-replan by STUCK_PHANTOM_S + a tick and
    end up with a different path or a WAITING downgrade."""
    grid = GridMap.from_yaml(GRID_YAML)
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.state.pose = pose
    c.state.loc_sigma_lat = 0.26
    logs = []
    c.log = logs.append
    assert c.set_goal(goal, now=NOW), name
    assert c._static_ahead_m() >= 0.65, (
        name, "precondition: the believed nose ray is clear - the hold is "
              "invisible to the seen-obstruction trigger")
    first = list(c.path)
    t, actions = NOW, []
    while t < NOW + 16.0:
        actions.append(c.tick(t).action)
        t = round(t + 0.5, 1)
    assert any("phantom hold" in m for m in logs), (name, logs[-6:])
    assert list(c.path) != first or "STOP" in actions, name


def test_live_cluster_rack3_eb_wedge_phantom_recovers():
    """webots_live_final cluster B: robot_2 held at believed (-2.93, 1.45,
    -7 deg) beside rack 3's north face (true clearance 0.21) for 5 wedges /
    8 StopZone stops across three L3->east trips. Believed ray at -7 deg
    exits the 0.65 m window above the face, so only the phantom trigger can
    see it."""
    _phantom_cluster_case(Pose2D(-2.93, 1.45, math.radians(-7)), (22, 60),
                          "rack3_eb")


def test_live_cluster_rack6_nb_entry_phantom_recovers():
    """webots_live_final cluster A: robot_2/robot_1 held at believed
    ~(0.46, -1.8, 80 deg) entering the rack-5/6 gap northbound (true pose
    0.13-0.17 east of belief, 0.18-0.25 from rack 6's west face). The
    believed ray runs up the free gap, so again only the phantom trigger
    fires."""
    _phantom_cluster_case(Pose2D(0.46, -1.80, math.radians(80)), (88, 25),
                          "rack6_nb")


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
