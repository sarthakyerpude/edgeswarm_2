"""Increment 1 integration: data layer (peers, intents) x safety envelope."""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import astar, safety
from amr_fleet.core.coordinator import FleetCoordinator
from amr_fleet.core.gridmap import GridMap, Zone
from amr_fleet.core.models import Intent, Pose2D, RobotState
from amr_fleet.core.safety import PeerDisc, safety_permit

# Open 6 x 6 m floor, 0.1 m cells, world (0, 0) at cell (30, 30).
GRID = GridMap(["." * 60] * 60, 0.1, (-3.0, -3.0), {})
NOW = 100.0


def _me(x=0.0, y=0.0, theta=0.0, v=0.0):
    return RobotState(robot_id="robot_1", pose=Pose2D(x, y, theta), v=v)


def _line(x0, x1, y=0.0):
    r = GRID.world_to_cell(0.0, y)[0]
    c0, c1 = GRID.world_to_cell(x0, y)[1], GRID.world_to_cell(x1, y)[1]
    step = 1 if c1 >= c0 else -1
    return [(r, c) for c in range(c0, c1 + step, step)]


def test_parked_peer_beside_path_does_not_hold_me_forever():
    """Inside D_stop (0.61 m < 0.67 m) but beside me, a few cm ahead of my
    shoulder: driving on only passes it, so the envelope must not stop me
    (the old 'peer ahead' dot-product test stopped me for good)."""
    path = _line(0.0, 2.0)
    beside = PeerDisc("robot_2", x=0.15, y=0.6, rx_time=NOW)
    assert safety_permit(_me(0.05, 0.05), path, [beside], GRID, NOW) is None
    assert safety_permit(_me(0.05, 0.05, v=0.3), path, [beside],
                         GRID, NOW) is None
    # Same offset but well ahead: passing would close the gap, so STOP.
    ahead = PeerDisc("robot_2", x=0.4, y=0.55, rx_time=NOW)
    p = safety_permit(_me(0.05, 0.05), path, [ahead], GRID, NOW)
    assert p is not None and p.action == "STOP"


def test_nan_sigma_never_disables_the_envelope():
    path = _line(0.0, 2.0)
    peer = PeerDisc("robot_2", x=1.1, y=0.05, rx_time=NOW,
                    loc_sigma_lat=float("nan"))
    p = safety_permit(_me(0.05, 0.05), path, [peer], GRID, NOW)
    assert p is not None and p.action == "STOP"
    assert safety.d_stop(float("nan"), 0.0) > safety.D_STOP_BASE_M


def test_deadlock_victim_reroute_still_obeys_envelope():
    """The victim's reroute GO replaces the enveloped permit; a peer closing
    head-on must still stop it."""
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(0.05, 0.05, 0.0)
    assert c.set_goal(GRID.world_to_cell(2.0, 0.0), now=NOW)
    c.on_peer_state(RobotState(robot_id="robot_2", seq=1,
                               pose=Pose2D(0.95, 0.05, math.pi), v=0.4),
                    NOW)
    p = c._yield_permit({"cycle": ["robot_1", "robot_2"]}, NOW)
    assert p.action == "STOP" and p.blocking_robot == "robot_2"
    assert p.deadlock_detected and p.deadlock_cycle == ["robot_1", "robot_2"]


def test_conflicts_use_my_retimed_suffix():
    """Peer windows are on my clock at receipt; my own side of the CELL check
    must be re-timed too, or nothing overlaps once my replan is old."""
    occ = ["." * 130 for _ in range(20)]
    grid = GridMap(occ, 0.1, (0.0, 0.0), {})
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.path = [(10, col) for col in range(111)]
    c.intent = astar.build_intent(grid, c.path, c.v_nominal, 0.0)
    c.state.pose = Pose2D(*grid.cell_to_world((10, 50)), 0.0)
    now = 100.0                          # replan windows ended long ago
    peer = RobotState(robot_id="robot_2", seq=1,
                      pose=Pose2D(*grid.cell_to_world((18, 120)), 0.0))
    c.on_peer_state(peer, now)
    c.registry.update_intent("robot_2", Intent(
        cells=[(10, 53), (10, 54)], t_enter=[now, now + 1.0],
        t_exit=[now + 5.0, now + 6.0]), seq=1, now=now)
    c.tick(now)
    assert any(k.kind == "CELL" and k.peer_id == "robot_2"
               for k in c.last_conflicts)


def test_zones_within_matches_brute_force():
    occ = ["." * 60 for _ in range(40)]
    rect = {(r, c) for r in range(10, 20) for c in range(5, 25)}
    ell = {(r, 30) for r in range(0, 30)} | {(29, c) for c in range(30, 55)}
    grid = GridMap(occ, 0.1, (-1.0, -2.0),
                   {"RECT": Zone("RECT", "corridor", rect),
                    "ELL": Zone("ELL", "corridor", ell)})

    def brute(x, y, radius):
        reach2 = (radius + 0.05) ** 2
        return {zid for zid, z in grid.zones.items()
                if any((grid.cell_to_world(cl)[0] - x) ** 2
                       + (grid.cell_to_world(cl)[1] - y) ** 2 <= reach2
                       for cl in z.cells)}

    for x in (-2.0, -0.5, 0.3, 1.0, 2.5, 4.9, 6.0):
        for y in (-3.0, -1.5, 0.0, 0.7, 1.3, 2.5):
            for radius in (0.2, 1.0, 3.0):
                assert grid.zones_within(x, y, radius) == brute(x, y, radius)
