"""Lane-band A* costs (core/gridmap.LaneBand, astar mode='LANES') and the
hard clearance gate: every station route and all 240 rack-slot approach
cells stay reachable at CLEARANCE_LETHAL_M=0.225 with astar.free()
rejecting sub-chassis clearance (AC11)."""
import math
import os
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

from amr_fleet.core import astar as astar_mod
from amr_fleet.core.astar import FREE_MIN_CLEARANCE_M, astar
from amr_fleet.core.gridmap import (CLEARANCE_LETHAL_M, LANE_LATERAL,
                                    LANE_OFF_CENTRE, LANE_WRONG_SIDE,
                                    LANE_WRONG_SIDE_HARD,
                                    GridMap, LaneBand)
from amr_fleet.core.rack_slots import RackSlots, all_labels

HERE = pathlib.Path(__file__).resolve().parent
YAML = HERE.parent / "config" / "warehouse_grid.yaml"

# Shared-contract lane band for the R7 spine (the PRIORITY builder writes the
# same numbers into the yaml; this test must not depend on that landing).
BAND = LaneBand(rect=(25, 52, 83, 67), axis="v",
                centre_pos=63.5, centre_neg=55.5)


def _grid():
    g = GridMap.from_yaml(str(YAML))
    return g


# -------------------------------------------------------------- LaneBand ----
def test_lane_band_membership_and_divider():
    assert BAND.divider == 59.5
    assert BAND.in_band(25, 52) and BAND.in_band(83, 67)
    assert not BAND.in_band(24, 60) and not BAND.in_band(60, 68)


def test_step_cost_wrong_side_off_centre_and_lateral():
    # northbound (increasing row) on the east side near centre: cheap
    assert BAND.step_cost(50, 63, 51, 63) == pytest.approx(
        LANE_OFF_CENTRE * 0.5 / 5.0)
    assert BAND.step_cost(50, 64, 51, 64) == pytest.approx(
        LANE_OFF_CENTRE * 0.5 / 5.0)
    # northbound on the WEST side: wrong way = PLANNER-FORBIDDEN (strict
    # directional lanes) outside a crossing zone...
    cost = BAND.step_cost(50, 56, 51, 56)
    assert cost >= LANE_WRONG_SIDE_HARD
    # ...and soft (escapable, still charged) inside one (crossing=True).
    assert LANE_WRONG_SIDE <= BAND.step_cost(
        50, 56, 51, 56, crossing=True) < LANE_WRONG_SIDE_HARD
    # the wrong-way term is NEVER tapered by along_scale (no wrong-way roll
    # at a route start); only the off-centre pull fades.
    assert BAND.step_cost(50, 56, 51, 56,
                          along_scale=0.01) >= LANE_WRONG_SIDE_HARD
    # southbound mirrored
    assert BAND.step_cost(51, 56, 50, 56) == pytest.approx(
        LANE_OFF_CENTRE * 0.5 / 5.0)
    assert BAND.step_cost(51, 63, 50, 63) >= LANE_WRONG_SIDE_HARD
    # pure lateral step: soft whatever the side - lateral is how a robot
    # merges onto its own lane, so a band always stays escapable sideways
    assert BAND.step_cost(50, 60, 50, 61) == pytest.approx(LANE_LATERAL)
    # diagonal pays both
    diag = BAND.step_cost(50, 62, 51, 63)
    assert diag == pytest.approx(LANE_LATERAL + LANE_OFF_CENTRE * 0.5 / 5.0)
    # off-centre saturates at LANE_OFF_SAT_CELLS (wrong side: hard term)
    assert BAND.step_cost(50, 53, 51, 53) == pytest.approx(
        LANE_WRONG_SIDE_HARD + LANE_OFF_CENTRE)
    # outside the band: free
    assert BAND.step_cost(20, 60, 21, 60) == 0.0


def test_yaml_lanes_and_pockets_parse(tmp_path):
    """GridMap.from_yaml reads traffic.lanes/pockets per the contract schema
    (written on a scratch yaml so this holds whether or not the live yaml
    has gained the section yet)."""
    y = tmp_path / "g.yaml"
    y.write_text(
        "meta: {resolution: 0.1, origin: [0.0, 0.0], width_cells: 10,"
        " height_cells: 10}\n"
        "occupancy:\n" + "".join(f"  - '{'.' * 10}'\n" for _ in range(10)) +
        "traffic:\n"
        "  mode: LANES\n"
        "  lanes:\n"
        "    spine: {rect: [2, 3, 8, 7], axis: v, centre_pos: 6.5,"
        " centre_neg: 4.5}\n"
        "  pockets: [[4, 2], [6, 2]]\n")
    g = GridMap.from_yaml(str(y))
    assert g.lanes is not None
    assert g.lanes.rect == (2, 3, 8, 7)
    assert g.lanes.axis == "v"
    assert g.lanes.centre_pos == 6.5 and g.lanes.centre_neg == 4.5
    assert g.pockets == [(4, 2), (6, 2)]
    # absent section -> None/[] (hand-built grids and legacy yamls)
    g2 = GridMap(["." * 10] * 10, 0.1, (0.0, 0.0), {})
    assert g2.lanes is None and g2.pockets == []


def test_lanes_mode_splits_north_and_south_traffic_in_the_spine():
    """AC11: a northbound and a southbound route through the spine keep to
    disjoint sides of the divider (NB east ~cols 62-65, SB west ~cols 54-57)
    when astar runs with mode='LANES'; without the mode both ride the centre."""
    g = _grid()
    g.lanes = BAND
    nb = astar(g, (26, 60), (82, 60), max_expansions=200000, mode="LANES")
    sb = astar(g, (82, 60), (26, 60), max_expansions=200000, mode="LANES")
    assert nb and sb
    # interior of the band (clear of the forced start/goal funnels)
    nb_cols = {c for r, c in nb if BAND.in_band(r, c) and 35 <= r <= 75}
    sb_cols = {c for r, c in sb if BAND.in_band(r, c) and 35 <= r <= 75}
    assert nb_cols and sb_cols
    assert all(c > BAND.divider for c in nb_cols), nb_cols
    assert all(c < BAND.divider for c in sb_cols), sb_cols
    assert not (nb_cols & sb_cols)
    assert all(abs(c - BAND.centre_pos) <= 2.0 for c in nb_cols), nb_cols
    assert all(abs(c - BAND.centre_neg) <= 2.0 for c in sb_cols), sb_cols
    # SPINE (the degraded-sigma fallback) on the ROAD MODEL grid: STRICT
    # directional lanes - the fallback stays in lane too (it used to drop
    # to the shared centre line = both lanes; user order 2026-10-04).
    plain_nb = astar(g, (26, 60), (82, 60), max_expansions=200000,
                     mode="SPINE")
    plain_sb = astar(g, (82, 60), (26, 60), max_expansions=200000,
                     mode="SPINE")
    pn = {c for r, c in plain_nb if BAND.in_band(r, c) and 35 <= r <= 75}
    ps = {c for r, c in plain_sb if BAND.in_band(r, c) and 35 <= r <= 75}
    assert pn and ps and not (pn & ps)
    # Only on a LEGACY grid (reservations on) does SPINE mean the shared
    # centre line (no lane costs at all).
    g2 = _grid()
    g2.traffic.reservations = True
    c_nb = astar(g2, (26, 60), (82, 60), max_expansions=200000, mode="SPINE")
    c_sb = astar(g2, (82, 60), (26, 60), max_expansions=200000, mode="SPINE")
    cn = {c for r, c in c_nb if 35 <= r <= 75}
    cs = {c for r, c in c_sb if 35 <= r <= 75}
    assert cn & cs


# -------------------------------------------------------- clearance gate ----
def test_constants_match_the_contract():
    assert CLEARANCE_LETHAL_M == 0.225
    assert FREE_MIN_CLEARANCE_M == 0.16


def test_gate_blocks_through_routes_but_exempts_start_and_goal():
    """Two rooms joined only by a 2-cell-wide corridor (every corridor cell
    0.05 m from a wall): the corridor is a WALL for routing. But a goal or
    start right beside a wall (also < 0.11 m clear) stays plannable - the
    exemption covers exactly the endpoint cells."""
    room = "#" + "." * 8 + "###" + "." * 8 + "#"
    corr = "#" + "." * 8 + "..." + "." * 8 + "#"
    occ = (["#" * 21] + [room] * 4
           + [corr, corr]                   # rows 5-6: the 2-wide corridor
           + [room] * 4 + ["#" * 21])
    # make the corridor only exist in the wall: rows 1-4 and 7-10 keep the
    # dividing wall at cols 9-11
    g = GridMap(occ, 0.1, (0.0, 0.0), {})
    clr = g.clearance_m()
    assert clr[5][10] < 0.11 and clr[6][10] < 0.11   # corridor is tight
    # through the corridor: refused
    assert astar(g, (5, 4), (5, 16)) == []
    # goal one cell off a wall (clearance 0.05): reachable via exemption
    tight_goal = (1, 4)
    assert clr[1][4] < 0.11
    assert astar(g, (3, 4), tight_goal) != []
    # and a robot sitting there can plan out again
    assert astar(g, tight_goal, (3, 4)) != []


def test_all_stations_and_all_240_approach_cells_stay_reachable():
    """AC11 feasibility at CLEARANCE_LETHAL_M=0.225 + the free() gate: BFS
    over gate-free cells from L1 reaches every station and every rack-slot
    approach cell (a goal counts as reachable if it or a neighbour is
    reached, matching the A* start/goal exemption)."""
    g = _grid()
    clr = g.clearance_m()
    gate = FREE_MIN_CLEARANCE_M - 0.5 * g.resolution
    from collections import deque
    start = g.stations["L1"]
    seen = {start}
    q = deque([start])
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (r + dr, c + dc)
            if (n not in seen and g.is_static_free(n)
                    and clr[n[0]][n[1]] >= gate):
                seen.add(n)
                q.append(n)

    def reachable(cell):
        if cell in seen:
            return True
        r, c = cell
        return any((r + dr, c + dc) in seen
                   for dr in (-1, 0, 1) for dc in (-1, 0, 1))

    for sid, cell in g.stations.items():
        assert reachable(cell), f"station {sid} {cell} unreachable"
    slots = RackSlots.from_yaml(str(YAML))
    bad = []
    for label in all_labels():
        cell = slots.slot(label).approach_cell
        if not reachable(cell):
            bad.append((label, cell))
        # the measured minimum approach clearance is 0.25 m - must stay legal
        assert clr[cell[0]][cell[1]] >= 0.24, (label, cell)
    assert not bad, f"unreachable approach cells: {bad[:8]} (n={len(bad)})"


def test_spot_route_still_plans_end_to_end_with_astar():
    g = _grid()
    for a, b in ((g.stations["L1"], g.stations["P2"]),
                 (g.stations["L3"], g.stations["P1"]),
                 ((10, 60), g.stations["L2"])):
        path = astar(g, a, b, max_expansions=200000)
        assert path, (a, b)
        clr = g.clearance_m()
        gate = FREE_MIN_CLEARANCE_M - 0.5 * g.resolution
        for cell in path[1:-1]:
            assert clr[cell[0]][cell[1]] >= gate
