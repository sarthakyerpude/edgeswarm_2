"""
Clearance-aware A*: Nav2 follows the fleet's A* path verbatim, so these tests
pin down that the path runs along aisle centre lines instead of hugging racks
and walls (wall scraping spins the wheels and corrupts odometry).

Geometry (warehouse_grid.yaml, 0.1 m cells, origin (-6,-5), row->Y, col->X):
  R7 equal aisles: every aisle and the spine is 1.6 m wide.
  aisle A  rows 62-77  y in [1.2, 2.8]   centre y = 2.0 (rows 69/70)
  aisle B  rows 40-55  y in [-1.0, 0.6]  centre y = -0.2 (rows 47/48)
  centre spine cols 52-67, x in [-0.8, 0.8]
  side strips: col 1 and col 118 (0.2 m wall-to-rack gap, narrower than the
  robot, closed by wall_endcap_* on the rack rows; cols 0/119 are the
  wall-face ring)
"""
import pathlib

import pytest

from amr_fleet.core.astar import astar
from amr_fleet.core.gridmap import CLEARANCE_LETHAL_M, GridMap

GRID = pathlib.Path(__file__).resolve().parents[1] / "config" / "warehouse_grid.yaml"
# Pre-increment-2 zones (inter_X1/aisle_A1/aisle_A2), frozen for the
# zone-name-dependent test below. See wiki/Traffic-Lanes.md.
LEGACY_GRID = (pathlib.Path(__file__).resolve().parent / "fixtures"
               / "warehouse_grid_legacy.yaml")
SIDE_STRIP_COLS = {1, 118}


@pytest.fixture(scope="module")
def gm():
    return GridMap.from_yaml(str(GRID))


@pytest.fixture(scope="module")
def gm_centre():
    """The centring-contract grid: reservations flipped ON so no lane band
    applies and the pure clearance-centring layer is what routes.

    STRICT DIRECTIONAL LANES (user order, 2026-10-04): on the shipped ROAD
    MODEL yaml every mode - the degraded-sigma SPINE fallback included -
    now plans keep-right lanes (test_road_lanes.py pins that), so aisle
    CENTRING is no longer reachable there. It remains the base layer for
    legacy reservation grids and for everything outside the lane bands,
    and these tests pin that machinery on the legacy configuration."""
    g = GridMap.from_yaml(str(GRID))
    g.traffic.reservations = True
    return g


@pytest.fixture(scope="module")
def legacy_gm():
    return GridMap.from_yaml(str(LEGACY_GRID))


SPINE = "SPINE"


def _interior(path, n=5):
    """Drop the first/last n cells (0.5 m) where the path may leave the line."""
    return path[n:-n]


def test_aisle_a_path_holds_centre_line(gm_centre):
    gm = gm_centre      # centring contract: legacy config (no lane bands)
    start = gm.world_to_cell(-5.0, 2.0)
    goal = gm.world_to_cell(-1.5, 2.0)
    path = astar(gm, start, goal, mode=SPINE)
    assert path and path[-1] == goal
    for cell in _interior(path):
        _, y = gm.cell_to_world(cell)
        assert abs(y - 2.0) <= 0.1 + 1e-6, f"{cell} y={y:.2f} off aisle A centre"


def test_aisle_a_entered_off_centre_converges_to_centre(gm_centre):
    gm = gm_centre      # centring contract: legacy config (no lane bands)
    # Start hugging the north rack face, end hugging the south one: the
    # middle of the run must still sit on the centre line.
    start = gm.world_to_cell(-5.0, 2.6)
    goal = gm.world_to_cell(-1.5, 1.45)
    path = astar(gm, start, goal, mode=SPINE)
    assert path and path[-1] == goal
    mid = [c for c in path if -4.0 <= gm.cell_to_world(c)[0] <= -2.5]
    assert mid
    for cell in mid:
        assert abs(gm.cell_to_world(cell)[1] - 2.0) <= 0.1 + 1e-6, cell


def test_centre_corridor_path_holds_x_zero(gm_centre):
    gm = gm_centre      # centring contract: legacy config (no lane bands)
    # Spawn row (y=-4) to the north strip, straight up the centre corridor.
    start = gm.world_to_cell(0.0, -4.0)
    goal = gm.world_to_cell(0.0, 4.35)
    path = astar(gm, start, goal, mode=SPINE)
    assert path and path[-1] == goal
    for cell in _interior(path):
        x, _ = gm.cell_to_world(cell)
        assert abs(x) <= 0.15 + 1e-6, f"{cell} x={x:.2f} off corridor centre"


def test_corridor_entered_from_the_side_stays_centred_between_racks(gm_centre):
    gm = gm_centre      # centring contract: legacy config (no lane bands)
    # Pickup L1 (north strip, west) to drop-off P1 (south corridor): every
    # cell between the rack rows (y in [-1.6, 3.4]) is in the centre spine.
    path = astar(gm, gm.stations["L1"], gm.stations["P1"], mode=SPINE)
    assert path
    between_racks = [c for c in path if -1.6 <= gm.cell_to_world(c)[1] <= 3.4]
    assert between_racks
    for cell in between_racks:
        assert abs(gm.cell_to_world(cell)[0]) <= 0.15 + 1e-6, cell


def test_aisle_b_is_still_traversable_and_centred(gm_centre):
    gm = gm_centre      # centring contract: legacy config (no lane bands)
    start = gm.world_to_cell(-5.0, -0.2)
    goal = gm.world_to_cell(-1.5, -0.2)
    path = astar(gm, start, goal, mode=SPINE)
    assert path and path[-1] == goal
    clr = gm.clearance_m()
    for cell in _interior(path):
        _, y = gm.cell_to_world(cell)
        assert abs(y + 0.2) <= 0.1 + 1e-6, cell
        # Must physically fit: centre at least the robot half-width from a rack.
        assert clr[cell[0]][cell[1]] >= CLEARANCE_LETHAL_M, cell


def test_no_path_uses_the_side_strips(gm):
    # West end of aisle A -> west end of aisle B. The 0.2 m strip along the
    # west wall is the geometric shortcut (~2 m); the robot does not fit, so
    # the path must go round through the centre corridor instead.
    start = gm.world_to_cell(-5.0, 2.0)
    goal = gm.world_to_cell(-5.0, -0.2)
    path = astar(gm, start, goal)
    assert path and path[-1] == goal
    assert not any(c in SIDE_STRIP_COLS for _, c in path)
    assert any(abs(gm.cell_to_world(c)[0]) <= 1.0 for c in path)


def test_station_paths_avoid_strips_and_reach_goal(gm):
    names = sorted(gm.stations)
    for a in names:
        for b in names:
            if a == b:
                continue
            path = astar(gm, gm.stations[a], gm.stations[b])
            assert path and path[-1] == gm.stations[b], f"{a}->{b}"
            assert not any(c in SIDE_STRIP_COLS for _, c in path), f"{a}->{b}"


def test_goal_beside_rack_is_reached_with_direct_approach(gm):
    # L1 (rack slot 1R10 approach cell) sits 0.45 m from the rack face.
    # The taper must still end the path exactly there, approached from the
    # free north side rather than by sliding along the rack.
    goal = gm.stations["L1"]
    path = astar(gm, gm.world_to_cell(0.0, -4.0), goal)
    assert path and path[-1] == goal
    clr = gm.clearance_m()
    assert all(clr[r][c] >= CLEARANCE_LETHAL_M for r, c in path[:-1])


def test_extra_cost_still_composes_with_clearance(gm):
    # A heavy soft penalty on the aisle A centre line must still divert the
    # path (congestion / zone penalties keep working on top of centring).
    # Aisle A centre y = 2.0 lies on a cell boundary: rows 69 and 70 tie, so
    # penalise both.
    start = gm.world_to_cell(-5.0, 2.0)
    goal = gm.world_to_cell(-1.5, 2.0)
    row = start[0]
    extra = {(r, c): 10.0 for r in (row - 1, row, row + 1)
             for c in range(start[1] + 6, goal[1] - 5)}
    path = astar(gm, start, goal, extra_cost=extra)
    assert path and path[-1] == goal
    assert not any(c in extra for c in path)


def test_stacked_zone_penalties_never_open_the_strips(legacy_gm):
    # replan(avoid_zones=held) adds 50/cell per held zone (+8 timeout
    # penalty), aisle_A1 overlaps inter_X1 so those stack, and srv_replan
    # takes a caller-chosen penalty. Heavy soft costs must never make a
    # physically impassable strip the cheaper route. (Legacy map, pre-R7
    # geometry with its 0.3 m strips, cols 1-2 / 117-118.)
    gm = legacy_gm
    extra = {}
    for zid in ("aisle_A1", "inter_X1", "aisle_A2"):
        for c in gm.zones[zid].cells:
            extra[c] = extra.get(c, 0.0) + 100.0
    clr = gm.clearance_m()
    for a, b in (("L1", "P1"), ("L2", "P3"), ("P2", "L1"), ("L3", "P3")):
        path = astar(gm, gm.stations[a], gm.stations[b], extra_cost=extra)
        assert path and path[-1] == gm.stations[b], f"{a}->{b}"
        assert all(clr[r][c] >= CLEARANCE_LETHAL_M for r, c in path[1:-1]), f"{a}->{b}"


def test_clearance_field_is_cached(gm):
    assert gm.clearance_cost() is gm.clearance_cost()
    assert gm.clearance_m() is gm.clearance_m()
