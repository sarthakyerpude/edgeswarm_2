"""R10 - 'like a traffic signal where paths intersect': the SPINE corridor
is re-cut into 3 junction-bounded ranked segments (SPINE_S < SPINE_M <
SPINE_N < J_N < J_AW), acquisition is WINDOWED (only the segments the next
stretch of path needs), and a held segment is released the moment the
robot's HITBOX clears it - gap-triggered, never by pose centre, path index
alone, or a timer. The ENTIRE corridor is never one lock.
"""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import astar, traffic
from amr_fleet.core.coordinator import FleetCoordinator
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Pose2D

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
NOW = 100.0
L1 = (88, 25)
SEGS = ("SPINE_S", "SPINE_M", "SPINE_N")


# ------------------------------------------------------- acquisition window
def test_northbound_window_is_one_segment_at_a_time():
    """Ascending-rank travel pipelines: the first window is the first
    segment (plus the rank-0 spur that could never be requested later);
    every other segment and J_N stay deferred - so a second robot can hold
    a DIFFERENT segment at the same time."""
    path = astar.astar(GRID, (22, 60), L1)
    assert path
    w0 = traffic.zone_window(GRID, path)
    assert "SPINE_S" in w0 and "SP_NW" in w0
    assert not {"SPINE_M", "SPINE_N", "J_N"} & set(w0)
    # Second round: only the next segment.
    w1 = traffic.zone_window(GRID, path, skip=set(w0))
    assert w1 == ["SPINE_M"]
    w2 = traffic.zone_window(GRID, path, skip=set(w0) | set(w1))
    assert w2 == ["SPINE_N"] or w2[0] == "SPINE_N"


def test_southbound_window_closes_over_the_descending_run():
    """Descending-rank travel must pre-acquire the whole run (Havender):
    deferring a lower-ranked segment would mean requesting it from inside a
    higher one."""
    path = astar.astar(GRID, (91, 60), (22, 60))
    assert path
    w0 = traffic.zone_window(GRID, path)
    assert {"SPINE_S", "SPINE_M", "SPINE_N", "J_N"} <= set(w0)
    keys = [traffic.rank_key(GRID, z) for z in w0]
    assert keys == sorted(keys), "window comes out in acquisition order"


def test_crossing_robot_needs_only_the_segment_it_crosses():
    """An east-west crossing of the corridor at aisle B touches SPINE_M
    only: the junction is arbitrated without locking the corridor ends."""
    path = astar.astar(GRID, (47, 30), (47, 90))
    assert path
    req = traffic.required_zones(GRID, path)
    assert "SPINE_M" in req
    assert not {"SPINE_S", "SPINE_N", "J_N"} & set(req)
    assert traffic.zone_window(GRID, path) == ["SPINE_M"]


# --------------------------------------------------- gap-triggered release
def _northbound_holder(row):
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((25, 60)), math.pi / 2)
    assert c.set_goal(L1, now=NOW)
    c.arbiter.state["SPINE_S"] = "HELD"
    c.state.pose = Pose2D(*GRID.cell_to_world((row, 60)), math.pi / 2)
    assert c._path_index_near_current() > max(
        i for i, cell in enumerate(c.path)
        if cell in GRID.zones["SPINE_S"].cells), "pose must be past SPINE_S"
    return c


def test_segment_released_only_when_the_hitbox_clears_it():
    """Pose centre past the boundary but hitbox still overlapping: HELD.
    Hitbox fully out: released. Same call, no time argument at all - the
    release is gap-triggered, not timer- or index-triggered."""
    c = _northbound_holder(41)      # centre 0.15 m past the boundary...
    for _ in range(50):             # ...and no amount of ticking releases it
        c._maybe_release_passed_zones()
    assert c.arbiter.state["SPINE_S"] == "HELD", (
        "hitbox still overlaps SPINE_S - releasing now would double-occupy")
    assert c._occupies("SPINE_S")
    c.state.pose = Pose2D(*GRID.cell_to_world((45, 60)), math.pi / 2)
    assert not c._occupies("SPINE_S")
    c._maybe_release_passed_zones()
    assert c.arbiter.state.get("SPINE_S", "FREE") == "FREE"


def test_release_gate_grows_with_localization_sigma():
    """The release inflation is K_SIG*sigma, the same rule peers apply to my
    occupancy: a badly localized robot keeps its segment longer."""
    c = _northbound_holder(43)      # 0.35 m past the boundary
    c.state.loc_sigma_lat = 0.2     # forced-fallback sigma (AC12)
    c._maybe_release_passed_zones()
    assert c.arbiter.state["SPINE_S"] == "HELD"
    c.state.loc_sigma_lat = 0.0
    c._maybe_release_passed_zones()
    assert c.arbiter.state.get("SPINE_S", "FREE") == "FREE"


def test_waiting_robot_sees_the_segment_free_when_the_hitbox_clears():
    """What gates a waiter's entry (zone occupancy) flips exactly with the
    crossing robot's hitbox - pose centre already out of SPINE_M does NOT
    free it while the body still straddles the junction row."""
    x, y = GRID.cell_to_world((62, 60))        # centre in SPINE_N already
    assert traffic.zone_occupants(
        GRID, "SPINE_M", {"a": (x, y, math.pi / 2, 0.0)}) == {"a"}
    x2, y2 = GRID.cell_to_world((66, 60))      # body fully across
    assert traffic.zone_occupants(
        GRID, "SPINE_M", {"a": (x2, y2, math.pi / 2, 0.0)}) == set()


# ------------------------------------------------------------ rank safety
def test_segment_ranks_keep_the_havender_order():
    order = [traffic.rank_key(GRID, z) for z in
             ("SP_AW", "SPINE_S", "SPINE_M", "SPINE_N", "J_N", "J_AW")]
    assert order == sorted(order)
    assert GRID.zones["J_N"].rank > GRID.zones["SPINE_N"].rank
    assert GRID.zones["J_AW"].rank > GRID.zones["SPINE_N"].rank


def test_every_station_route_window_is_rank_safe():
    """For every station-to-station route, each successive acquisition
    window ranks strictly above everything a robot could still hold from
    earlier windows - no window sequence can deadlock."""
    pts = dict(GRID.stations)
    import itertools
    for a, b in itertools.permutations(sorted(pts), 2):
        if pts[a] == pts[b]:
            continue
        path = astar.astar(GRID, pts[a], pts[b])
        assert path, (a, b)
        done: set = set()
        prev_max = None
        for _ in range(8):
            w = traffic.zone_window(GRID, path, skip=done)
            if not w:
                break
            lo = min(traffic.rank_key(GRID, z) for z in w)
            if prev_max is not None:
                assert lo > prev_max, (a, b, w, done)
            prev_max = max(traffic.rank_key(GRID, z) for z in w)
            done |= set(w)
        assert set(traffic.required_zones(GRID, path)) == done, (a, b)
