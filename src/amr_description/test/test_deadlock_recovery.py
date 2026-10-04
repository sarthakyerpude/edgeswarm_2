"""Deadlock-victim recovery: the victim must physically clear the way.

Regression for the Webots run where a deadlock victim 'rerouted' onto the
same route (live peers are not planner obstacles), emitted REROUTE (zero speed
at velocity_gate), and froze for minutes while being re-elected every tick.
"""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import astar, traffic
from amr_fleet.core.coordinator import (FleetCoordinator,
                                        RETREAT_INTENT_CLEAR_M,
                                        RETREAT_PEER_CLEAR_M)
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Pose2D, RobotState

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
NOW = 100.0
L1 = (88, 25)          # north-west pickup (rack slot 1R10)
D2 = (22, 60)          # centre dropoff
# Blocker stopped in the west half of the 1.6 m aisle-A crossing (R7): the
# east half stays passable, 0.86 m from its centre.
BLOCKER_X, BLOCKER_Y = -0.3, 2.0


def _victim(x, y, theta, goal):
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(x, y, theta)
    assert c.set_goal(goal, now=NOW)
    return c


def _peer_with_intent(rid, x, y, theta, goal, now=NOW):
    start = GRID.world_to_cell(x, y)
    path = astar.astar(GRID, start, goal)
    assert path
    intent = astar.build_intent(GRID, path, 0.4, now)
    return RobotState(robot_id=rid, seq=1, stamp=now, pose=Pose2D(x, y, theta),
                      intent=intent)


def _head_on_in_spine():
    """robot_1 northbound to L1 waits in the aisle-A crossing; robot_2
    southbound to D2 sits in the 1.6 m spine gap through rack row 1, where
    nobody can pass it (its body + the rack-face bands close the whole gap)."""
    victim = _victim(0.0, 2.0, math.pi / 2, L1)
    blocker = _peer_with_intent("robot_2", 0.0, 3.1, -math.pi / 2, D2)
    victim.on_peer_state(blocker, NOW)
    return victim, blocker


def _hold_route_zones(c):
    """Traffic rule 3: a robot inside SPINE got there holding every ranked
    zone of its route (acquired in rank order at its acquisition point)."""
    for z in traffic.required_zones(GRID, c.path):
        c.arbiter.state[z] = "HELD"
    return c


def test_blocker_in_next_segment_means_retreat_not_a_squeeze_past():
    """R10 semantics: the blocker stands in the aisle-A crossing, which is
    now the capacity-1 segment SPINE_N. The victim (in SPINE_M, below the
    junction) may NOT slip past inside the blocker's segment - occupancy
    blocks it - so the deadlock resolution is a retreat, whose route never
    hugs a rack face, never enters a ranked zone the victim neither holds
    nor stands in, and keeps clear of the blocker's body."""
    victim = _hold_route_zones(_victim(0.0, 0.9, math.pi / 2, L1))
    victim.on_peer_state(_peer_with_intent("robot_2", BLOCKER_X, BLOCKER_Y,
                                           -math.pi / 2, D2), NOW)
    p = victim._yield_permit({"cycle": ["robot_1", "robot_2"]}, NOW)
    assert p.action in ("GO", "SLOW")
    assert victim._retreat is not None, (
        "a capacity-1 segment with a body inside must not be 'rerouted' "
        "through")
    usable = (set(victim.arbiter.held_zones())
              | set(victim._inside_zones()))
    clearance = GRID.clearance_m()
    for cell in victim.path[1:]:
        cx, cy = GRID.cell_to_world(cell)
        assert math.hypot(cx - BLOCKER_X, cy - BLOCKER_Y) >= 0.7
        assert clearance[cell[0]][cell[1]] >= 0.22
        assert set(traffic.zones_at(GRID, cell)) <= usable, cell


def test_victim_inside_a_zone_it_does_not_hold_retreats_instead():
    """Same geometry, but the victim holds nothing (drifted in, or its
    holds were dropped): a 'reroute' would stop it at once to acquire SPINE
    from inside, so it does not count - the victim retreats, and the retreat
    route enters no ranked zone except the one it already stands in."""
    victim = _victim(0.0, 0.9, math.pi / 2, L1)
    victim.on_peer_state(_peer_with_intent("robot_2", BLOCKER_X, BLOCKER_Y,
                                           -math.pi / 2, D2), NOW)
    p = victim._yield_permit({"cycle": ["robot_1", "robot_2"]}, NOW)
    assert victim._retreat is not None and p.action in ("GO", "SLOW")
    for cell in victim.path:
        assert set(traffic.zones_at(GRID, cell)) <= {"SPINE_M"}, cell


def test_victim_retreats_off_the_blockers_path_and_moves():
    victim, blocker = _head_on_in_spine()
    p = victim._yield_permit({"cycle": ["robot_1", "robot_2"]}, NOW)

    # It moves (velocity_gate passes GO/SLOW only) - never REROUTE/zero.
    assert p.action in ("GO", "SLOW"), p
    assert p.speed_scale > 0.0
    assert victim._retreat is not None, "head-on in the spine needs a retreat"
    assert victim.state.status == "YIELDING" and victim.state.waiting_for == ""

    # The refuge is off the blocker's body and off its whole broadcast path.
    rx, ry = GRID.cell_to_world(victim._retreat["cell"])
    assert math.hypot(rx - 0.0, ry - 3.1) >= RETREAT_PEER_CLEAR_M
    for cell in blocker.intent.cells:
        cx, cy = GRID.cell_to_world(cell)
        assert math.hypot(rx - cx, ry - cy) >= RETREAT_INTENT_CLEAR_M - 1e-6
    # ...and the retreat route never drives through the blocker's body or
    # along a rack face.
    clearance = GRID.clearance_m()
    for cell in victim.path[1:]:
        cx, cy = GRID.cell_to_world(cell)
        assert math.hypot(cx - 0.0, cy - 3.1) > 0.5
        assert clearance[cell[0]][cell[1]] >= 0.22


def test_victim_holds_then_resumes_task_once_blocker_has_passed():
    victim, _ = _head_on_in_spine()
    victim._yield_permit({"cycle": ["robot_1", "robot_2"]}, NOW)
    refuge = victim._retreat["cell"]
    rx, ry = GRID.cell_to_world(refuge)

    # Arrive at the refuge; blocker still close by -> keep holding.
    victim.state.pose = Pose2D(rx, ry, 0.0)
    t = NOW + 5.0
    victim.on_peer_state(_peer_with_intent("robot_2", 0.0, 2.2, -math.pi / 2,
                                           D2, now=t), t)
    p = victim.tick(t)
    assert p.action == "STOP" and "yielding at refuge" in p.reason
    assert victim.state.waiting_for == "", "no wait-for edge while yielding"

    # Blocker has driven well past (south corridor) -> resume the task.
    for t in (NOW + 8.0, NOW + 9.0, NOW + 10.0):
        victim.on_peer_state(_peer_with_intent(
            "robot_2", 0.0, -3.0, -math.pi / 2, D2, now=t), t)
        p = victim.tick(t)
    assert victim._retreat is None
    assert victim.goal_cell == L1 and victim.path, "task route restored"
    assert victim.state.status != "YIELDING"


def test_clear_goal_mid_retreat_parks_at_refuge_without_a_task():
    victim, _ = _head_on_in_spine()
    victim._yield_permit({"cycle": ["robot_1", "robot_2"]}, NOW)
    victim.clear_goal()                     # e.g. task released mid-yield
    assert victim._retreat is not None and victim.path, "still clears the way"
    assert victim._retreat["saved_goal"] is None


def test_reroute_counts_only_if_it_avoids_the_blocker():
    """Open floor: a genuine detour exists, so the victim reroutes (GO) and
    the new path keeps clear of the blocker's body."""
    open_grid = GridMap(["." * 60] * 60, 0.1, (-3.0, -3.0), {})
    c = FleetCoordinator("robot_1", open_grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(-1.5, 0.0, 0.0)
    assert c.set_goal(open_grid.world_to_cell(1.5, 0.0), now=NOW)
    c.on_peer_state(RobotState(robot_id="robot_2", seq=1, stamp=NOW,
                               pose=Pose2D(0.0, 0.0, math.pi)), NOW)
    p = c._yield_permit({"cycle": ["robot_1", "robot_2"]}, NOW)
    assert p.action == "GO" and c._retreat is None
    for cell in c.path:
        cx, cy = open_grid.cell_to_world(cell)
        assert math.hypot(cx, cy) > 0.7


# ------------------------------------------- designated refuges (increment 2)
def _designated():
    tier1, tier2 = traffic.designated_refuge_cells(GRID, GRID.clearance_m(),
                                                   0.30)
    return set(tier1), set(tier2)


def test_refuge_comes_from_the_designated_set_and_no_ranked_zone():
    """The Webots run parked victims in the spine's rack gaps ((86,55),
    (67,62)): now the refuge is a retreat cell / wait bay (tier 1) or a
    refuge centre-line cell (tier 2), never a ranked-zone cell."""
    tier1, tier2 = _designated()
    pockets = set(traffic.pockets(GRID))
    assert pockets == {(66, 80), (74, 80), (44, 80), (52, 80),
                       (44, 40), (52, 40)}, "R3 pull-over pockets"
    assert tier1 == ({(47, 47), (47, 72), (70, 73), (30, 35), (30, 85)}
                     | pockets)
    assert tier2 and not any(traffic.zones_at(GRID, c) for c in tier2)
    for start in ((0.0, 2.0), (0.0, 0.9), (0.0, -1.75)):
        victim, _ = _head_on_in_spine()
        victim.state.pose = Pose2D(start[0], start[1], math.pi / 2)
        refuge = victim._find_refuge(NOW)
        assert refuge in tier1 | tier2, (start, refuge)
        assert traffic.zones_at(GRID, refuge) == [], refuge
        assert refuge not in {(86, 55), (67, 62)}


def test_tier1_is_preferred_and_peers_intents_are_respected():
    victim, blocker = _head_on_in_spine()
    tier1, _ = _designated()
    refuge = victim._find_refuge(NOW)
    assert refuge in tier1
    # Put a second peer's route through that cell: a different one is chosen,
    # still from the designated set.
    rx, ry = GRID.cell_to_world(refuge)
    victim.on_peer_state(_peer_with_intent("robot_3", rx, ry, 0.0, D2), NOW)
    other = victim._find_refuge(NOW)
    assert other is not None and other != refuge
    assert other in tier1 | _designated()[1]


def test_fallback_search_never_parks_in_a_ranked_zone(monkeypatch):
    """No designated cell available: the old local BFS runs, but not into
    the spine's rack gaps or any other ranked zone."""
    monkeypatch.setattr(traffic, "designated_refuge_cells",
                        lambda *a, **k: ([], []))
    victim, _ = _head_on_in_spine()
    refuge = victim._find_refuge(NOW)
    assert refuge is not None
    assert traffic.zones_at(GRID, refuge) == [], refuge


def test_escalate_turns_into_a_retreat_not_another_reroute():
    """deadlock.py 'escalate' (yielding failed for T_YIELD_S): on open floor
    a reroute would exist, but the victim retreats instead."""
    open_grid = GridMap(["." * 60] * 60, 0.1, (-3.0, -3.0), {})
    c = FleetCoordinator("robot_1", open_grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(-1.5, 0.0, 0.0)
    assert c.set_goal(open_grid.world_to_cell(1.5, 0.0), now=NOW)
    c.on_peer_state(RobotState(robot_id="robot_2", seq=1, stamp=NOW,
                               pose=Pose2D(0.0, 0.0, math.pi)), NOW)
    p = c._yield_permit({"cycle": ["robot_1", "robot_2"], "escalate": True},
                        NOW)
    assert c._retreat is not None and p.action in ("GO", "SLOW")


def test_retreat_keeps_lower_zones_while_committed_then_releases_them():
    """A victim inside SPINE_N holding SP_NW for L1 retreats: SP_NW is not
    on the refuge route, but giving it back while still committed in
    SPINE_N would force a rank-0 request from inside rank 3 on resume. It
    is kept until the robot has left SPINE_N, then released with it."""
    victim, _ = _head_on_in_spine()
    _hold_route_zones(victim)
    assert {"SP_NW", "SPINE_N"} <= set(victim.arbiter.held_zones())
    victim._yield_permit({"cycle": ["robot_1", "robot_2"], "escalate": True},
                         NOW)
    assert victim._retreat is not None
    victim.tick(NOW + 0.1)                    # still inside SPINE_N
    assert victim.arbiter.state["SP_NW"] == "HELD"
    assert victim.arbiter.state["SPINE_N"] == "HELD"
    rx, ry = GRID.cell_to_world(victim._retreat["cell"])
    victim.state.pose = Pose2D(rx, ry, 0.0)   # out of the spine, at refuge
    victim.tick(NOW + 0.2)
    assert victim.arbiter.state.get("SP_NW", "FREE") == "FREE"
    assert victim.arbiter.state.get("SPINE_N", "FREE") == "FREE"
