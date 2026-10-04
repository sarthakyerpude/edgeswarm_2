"""Street-style OVERTAKE of a stationary blocker (user report: "due to 1
robot other robots are stuck").

Gate semantics (traffic.overtake_gate) on a synthetic two-lane corridor,
then end-to-end in fleet_sim: a robot frozen mid-lane WITH a GO permit (the
F1 wedge class - broadcast status MOVING, v = 0) is passed by the robot
directly behind it through the opposing lane, gated, audit-clean and with
zero contacts; a robot parked ON my goal is a station queue and is never
overtaken.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import traffic
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Intent, Pose2D, RobotState, Task

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fleet_sim as fs                                        # noqa: E402

NOW = 500.0
H, W = 30, 24


def make_grid():
    occ = []
    for r in range(H):
        if r in (0, H - 1):
            occ.append("#" * W)
        else:
            occ.append("#" + "." * (W - 2) + "#")
    g = GridMap(occ, 0.1, (0.0, 0.0), {})

    def lane_band_at(row, col):
        if not (1 <= row <= H - 2 and 1 <= col <= W - 2):
            return None
        return ("corr", "NB" if col >= 12 else "SB")

    g.lane_band_at = lane_band_at
    g.in_crossing_zone = lambda row, col: False
    return g


GRID = make_grid()
MY_BAND = ("corr", "NB")
MY_POSE = (*GRID.cell_to_world((10, 16)), math.pi / 2, 0.05)
BORROW = [(r, c) for r in range(8, 19) for c in range(6, 12)]


def _peer(rid, cell, theta=-math.pi / 2, v=0.0, sigma=0.05):
    x, y = GRID.cell_to_world(cell)
    st = RobotState(robot_id=rid, seq=1, stamp=NOW, pose=Pose2D(x, y, theta))
    st.v = v
    st.loc_sigma_lat = sigma
    st.intent = Intent()
    return st


def gate(peers=(), borrow=BORROW, blocker_id=""):
    peers = {p.robot_id: p for p in peers}
    return traffic.overtake_gate(NOW, MY_POSE, MY_BAND, borrow, peers, GRID,
                                 blocker_id=blocker_id)


def test_clear_corridor_allows_the_borrow():
    ok, reason = gate()
    assert ok and reason == "clear"


def test_oncoming_mover_in_the_opposing_half_denies():
    p = _peer("robot_9", (26, 8), v=0.6)     # SB, 0.8 m upstream of the window
    ok, reason = gate([p])
    assert not ok and "oncoming" in reason and "robot_9" in reason


def test_far_stationary_opposing_peer_is_clear():
    p = _peer("robot_9", (26, 8), v=0.0)     # parked 0.8 m upstream: inert
    ok, reason = gate([p])
    assert ok, reason


def test_occupant_of_the_borrow_cells_denies_unless_it_is_the_blocker():
    p = _peer("robot_9", (12, 8), v=0.0)
    ok, reason = gate([p])
    assert not ok and "occupied" in reason
    ok, reason = gate([p], blocker_id="robot_9")
    assert ok, reason


def test_undrivable_borrow_ground_denies():
    ok, reason = gate(borrow=BORROW + [(12, 1)])     # 0.15 m off the wall
    assert not ok and "static fit" in reason


# ----------------------------------------------------------- end to end ----
def _freeze(sim, rid):
    sim.by_id[rid]._control = lambda: (0.0, 0.0)


def test_follower_overtakes_a_frozen_leader_and_delivers():
    """robot_2 freezes mid NB lane with a GO permit (status MOVING, v=0, the
    F1 wedge class); robot_1's task runs straight through it. Expect: a
    gated overtake within ~15 s of queueing, delivery, zero contacts, zero
    wrong-lane audit ticks (overtake_active-exempt)."""
    robots = [
        fs.RobotSpec("robot_1", -3.0, -4.0, math.pi / 2, dock=False,
                     tasks=[(0.5, Task("OV-1", pickup=fs.L1, dropoff=fs.D1))]),
        fs.RobotSpec("robot_2", 0.45, 0.85, math.pi / 2, dock=False,
                     tasks=[(0.2, Task("OV-2", pickup=fs.L2, dropoff=fs.D2))]),
    ]
    sc = fs.Scenario(name="overtake_frozen_leader", robots=robots, seed=0,
                     keep_log=True)
    sim = fs.FleetSim(sc)
    _freeze(sim, "robot_2")
    while sim.t < 240.0 and sim.by_id["robot_1"].deliveries < 1:
        sim.step()
    m = sim.metrics(sim.t)
    assert sim.by_id["robot_1"].deliveries == 1, (
        m["permit_actions"], [l for l in sim.logs if l[1] == "robot_1"][-8:])
    assert m["contacts"] == 0
    assert m["overtake_count"] >= 1
    assert m["wrong_lane_ticks"] == 0, m["wrong_lane_samples"][:5]
    assert any("OVERTAKE" in msg for _, rid, msg in sim.logs
               if rid == "robot_1")


def test_blocker_on_my_goal_is_queued_not_overtaken():
    """A robot parked ON my pickup is a station queue (goal-queue permit):
    no overtake may fire, whatever the wait."""
    robots = [
        fs.RobotSpec("robot_1", -3.0, -4.0, math.pi / 2, dock=False,
                     tasks=[(0.5, Task("GQ-1", pickup=fs.L1, dropoff=fs.D1))]),
        fs.RobotSpec("robot_2", *fs._cell_xy(fs.L1), theta=math.pi / 2,
                     dock=False,
                     tasks=[(0.2, Task("GQ-2", pickup=fs.L2, dropoff=fs.D2))]),
    ]
    sc = fs.Scenario(name="goal_queue_no_overtake", robots=robots, seed=0,
                     keep_log=True)
    sim = fs.FleetSim(sc)
    _freeze(sim, "robot_2")
    while sim.t < 90.0:
        sim.step()
    m = sim.metrics(sim.t)
    assert m["contacts"] == 0
    assert m["overtake_count"] == 0, [
        (t, r, msg) for t, r, msg in sim.logs if "OVERTAKE" in msg]
    assert sim.by_id["robot_1"].pickups == 0       # still queued, no barge
