"""Road model in/out lanes (user order, 2026-10-04).

"Instead of moving in the centre, move the robots on the in/out paths":
every aisle is a dead-end spur off the spine, entered on one keep-right lane
and left on the other, so a robot coming the opposite way has a completely
separate path and the two only interact at a junction box.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.astar import astar                       # noqa: E402
from amr_fleet.core.gridmap import GridMap, lane_step_cost   # noqa: E402

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
RACKS = [(-5.8, -0.8, 2.8, 3.4), (0.8, 5.8, 2.8, 3.4), (-5.8, -0.8, 0.6, 1.2),
         (0.8, 5.8, 0.6, 1.2), (-5.8, -0.8, -1.6, -1.0), (0.8, 5.8, -1.6, -1.0)]
PLACES = {"dock1": (-3.0, -4.0), "dock2": (0.0, -4.0), "dock3": (3.0, -4.0),
          "L1": (-3.45, 3.85), "L2": (3.45, 3.85), "L3": (-3.45, 2.45),
          "B_w": (-3.0, 0.25), "A_e": (3.0, 1.55), "P1": (-3.0, -2.75)}
# (aisle rows, EB lane row, WB lane row) - warehouse_grid.yaml traffic.lanes
# (aisle N WB moved 95 -> 94, corner/rotation fixes 2026-10-04: y 4.55 left
# the collision monitor's forward StopZone overlapping the north wall).
AISLES = {"N": ((84, 98), 87, 94), "A": ((62, 77), 65, 74), "B": ((40, 55), 43, 52)}
SPINE_COLS, NB_COL, SB_COL = (52, 67), 64, 55


def _route(a, b):
    p = astar(GRID, GRID.world_to_cell(*PLACES[a]), GRID.world_to_cell(*PLACES[b]))
    assert p, f"{a}->{b} must plan"
    return p


def _rack_clearance(cell):
    x, y = GRID.cell_to_world(cell)
    return min((max(a - x, 0, x - b) ** 2 + max(c - y, 0, y - e) ** 2) ** .5
               for a, b, c, e in RACKS)


def _lines(path):
    """{('spine', 'N'|'S') or (aisle, 'E'|'W'): {lane line: step count}}."""
    out = {}
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        in_spine = SPINE_COLS[0] <= c1 <= SPINE_COLS[1]
        key = line = None
        if in_spine and c1 == c0 and r1 != r0 and 25 <= r1 <= 83:
            key, line = ("spine", "N" if r1 > r0 else "S"), c1
        elif not in_spine and r1 == r0 and c1 != c0:
            for name, ((lo, hi), _, _) in AISLES.items():
                if lo <= r1 <= hi:
                    key, line = (name, "E" if c1 > c0 else "W"), r1
        if key is not None:
            out.setdefault(key, {}).setdefault(line, 0)
            out[key][line] += 1
    return out


# A trip may first roll this many cells the way the robot faces before it
# merges / U-turns (astar.LANE_START_TAPER_CELLS): such a run is no lane.
START_ROLL = 8


def test_bands_parsed_and_spine_band_kept():
    assert GRID.lanes is not None and GRID.lanes.rect == (25, 52, 83, 67)
    assert len(GRID.lane_bands) == 5


def test_every_trip_in_and_out_uses_opposite_lanes():
    pairs = [("dock1", "L1"), ("dock3", "L2"), ("dock2", "L3"),
             ("dock1", "B_w"), ("dock3", "A_e")]
    for a, b in pairs:
        for trip in ((a, b), (b, a)):
            for (name, d), steps in _lines(_route(*trip)).items():
                if sum(steps.values()) <= START_ROLL:
                    continue                  # a start roll, not a run
                if name == "spine":
                    want = NB_COL if d == "N" else SB_COL
                else:
                    _, eb, wb = AISLES[name]
                    want = eb if d == "E" else wb
                assert max(steps, key=steps.get) == want, (trip, name, d, steps)
                # anything off the lane line is only the short start roll
                assert sum(steps.values()) - steps[want] <= START_ROLL, \
                    (trip, name, d, steps)
        # in on the NB lane, back on the SB lane: separate spine paths
        assert ("spine", "N") in _lines(_route(a, b))
        assert ("spine", "S") in _lines(_route(b, a))


def test_lanes_keep_rack_clearance():
    for a, b in [("dock1", "L1"), ("L1", "P1"), ("dock3", "L2"),
                 ("dock1", "B_w"), ("dock3", "A_e")]:
        for p in (_route(a, b), _route(b, a)):
            assert min(_rack_clearance(c) for c in p) >= 0.35 - 1e-6


def test_turns_through_junction_boxes_avoid_tight_ground():
    """A diagonal through a box toward a rack corner is what wedged robot_3
    on rack 1's NE corner live. Since the turn-shaping terms (corner &
    rotation fixes, 2026-10-04) corners are placed where the 0.286 m spin
    disc + sigma margin + RPP overshoot fit, so a diagonal step inside a box
    is fine ON CLEAR GROUND - the invariant is that no diagonal (or any
    turning step) inside a box lands on ground with less than ~0.43 m of
    static clearance (disc 0.286 + sigma share 0.071 + half-cell raster)."""
    clr = GRID.clearance_m()

    def junction(c):
        return sum(b.in_band(*c) for b in GRID.lane_bands) >= 2
    for a, b in [("dock1", "L1"), ("L1", "P1"), ("dock3", "L2"),
                 ("L2", "dock3"), ("dock1", "L3"), ("A_e", "dock3")]:
        p = _route(a, b)
        tight = [(u, v, round(clr[v[0]][v[1]], 2))
                 for u, v in zip(p, p[1:])
                 if junction(v) and u[0] != v[0] and u[1] != v[1]
                 and clr[v[0]][v[1]] < 0.43]
        assert not tight, (a, b, tight[:3])


def test_h_band_wrong_side_is_charged_for_both_directions():
    aisle_a = [b for b in GRID.lane_bands if b.rect == (62, 1, 77, 118)][0]
    # EB (+col) on the north (WB) lane is wrong; on the south lane it is not.
    assert aisle_a.step_cost(74, 20, 74, 21) > aisle_a.step_cost(65, 20, 65, 21)
    # WB (-col) on the south (EB) lane is wrong; on the north lane it is not.
    assert aisle_a.step_cost(65, 21, 65, 20) > aisle_a.step_cost(74, 21, 74, 20)


def test_junction_cells_charge_no_lane_changes():
    # (70, 60) is where aisle A crosses the spine: a horizontal step there is
    # a lane change for the spine band but travel for the aisle band.
    assert sum(b.in_band(70, 60) for b in GRID.lane_bands) == 2
    single = [b for b in GRID.lane_bands if b.in_band(70, 60)]
    assert lane_step_cost(GRID.lane_bands, 70, 60, 70, 61) < sum(
        b.step_cost(70, 60, 70, 61) for b in single)


def test_off_lane_start_keeps_heading_before_merging():
    """A robot replanned between the spine lanes first drives straight, then
    merges (no swerve from standstill - _repro_pin.spine_wedge)."""
    p = astar(GRID, (59, 60), (74, 25))
    assert p[1] == (60, 60) and p[2] == (61, 60), p[:4]


def _wrong_way_steps(path):
    """Along-axis steps taken against the direction of the lane the target
    cell belongs to, outside crossing zones (junction boxes, turnarounds).
    STRICT LANES invariant: a planned route never contains one."""
    out = []
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        if GRID.in_crossing_zone(r1, c1):
            continue
        hit = GRID.lane_band_at(r1, c1)
        if hit is None:
            continue
        _, d = hit
        if ((d == "NB" and r1 < r0) or (d == "SB" and r1 > r0)
                or (d == "EB" and c1 < c0) or (d == "WB" and c1 > c0)):
            out.append(((r0, c0), (r1, c1), d))
    return out


# --------------------------------------------- strict directional lanes ----
def test_lane_band_at_names_and_directions():
    assert GRID.lane_band_at(30, 64) == ("spine", "NB")
    assert GRID.lane_band_at(30, 55) == ("spine", "SB")
    assert GRID.lane_band_at(65, 20) == ("aisle_a", "EB")
    assert GRID.lane_band_at(74, 20) == ("aisle_a", "WB")
    assert GRID.lane_band_at(43, 100) == ("aisle_b", "EB")
    assert GRID.lane_band_at(52, 100) == ("aisle_b", "WB")
    # junction boxes (two bands) and off-road cells have no single lane
    assert GRID.lane_band_at(90, 60) is None
    assert GRID.lane_band_at(70, 60) is None
    assert GRID.lane_band_at(10, 60) is None


def test_crossing_zones_are_junction_boxes_and_turnarounds():
    assert [t["id"] for t in GRID.traffic.turnarounds] == [
        "TA_N_W", "TA_N_E", "TA_A_W", "TA_A_E", "TA_B_W", "TA_B_E",
        "TA_SPINE_S", "TA_B_WM", "TA_B_EM", "TA_A_EM"]
    assert len(GRID.turnaround_rects) == 10
    for (r0, c0, r1, c1) in GRID.turnaround_rects:
        assert all(GRID.is_static_free((r, c))
                   for r in range(r0, r1 + 1) for c in range(c0, c1 + 1))
    assert GRID.in_crossing_zone(88, 5)        # TA_N_W (aisle N dead end)
    assert GRID.in_crossing_zone(47, 112)      # TA_B_E
    assert GRID.in_crossing_zone(90, 60)       # J_N box (band overlap)
    assert GRID.in_crossing_zone(70, 60)       # aisle A x spine box
    # south-throat apron: the band mouth rows 25-27 are a merge crossing
    assert GRID.in_crossing_zone(26, 55) and GRID.in_crossing_zone(27, 67)
    assert not GRID.in_crossing_zone(28, 55)   # throat proper: strict again
    # spur-mouth aprons around the three designated retreat cells
    assert GRID.in_crossing_zone(48, 47)       # beside retreat cell (47,47)
    assert GRID.in_crossing_zone(47, 72) and GRID.in_crossing_zone(70, 73)
    assert not GRID.in_crossing_zone(88, 30)   # mid-aisle: NOT a crossing
    assert not GRID.in_crossing_zone(30, 64)   # mid-spine lane
    assert not GRID.in_crossing_zone(10, 60)   # open south floor


def test_wrong_way_is_planner_forbidden_outside_crossing_zones():
    from amr_fleet.core.gridmap import (LANE_WRONG_SIDE,
                                        LANE_WRONG_SIDE_HARD)
    spine = GRID.lanes
    # northbound on the SB lane: forbidden...
    assert spine.step_cost(26, 55, 27, 55) >= LANE_WRONG_SIDE_HARD
    # ...even right at a route start (the old taper let a robot roll up to
    # 8 cells against the flow before its U-turn)
    assert spine.step_cost(26, 55, 27, 55,
                           along_scale=0.1) >= LANE_WRONG_SIDE_HARD
    # ...but soft inside a crossing zone, so U-turns can swing through
    assert LANE_WRONG_SIDE <= spine.step_cost(
        26, 55, 27, 55, crossing=True) < LANE_WRONG_SIDE_HARD


def test_no_route_ever_drives_against_a_lane():
    """The strict-lanes planning invariant over every station trip, both
    directions, in the normal mode AND the degraded-sigma SPINE fallback."""
    pairs = [("dock1", "L1"), ("dock3", "L2"), ("dock2", "L3"),
             ("dock1", "B_w"), ("dock3", "A_e"), ("L1", "P1"), ("L2", "P1")]
    for a, b in pairs:
        for trip in ((a, b), (b, a)):
            p = _route(*trip)
            assert not _wrong_way_steps(p), (trip, _wrong_way_steps(p)[:3])
            q = astar(GRID, GRID.world_to_cell(*PLACES[trip[0]]),
                      GRID.world_to_cell(*PLACES[trip[1]]), mode="SPINE")
            assert q and not _wrong_way_steps(q), (trip, "SPINE")


def test_start_in_the_opposite_lane_crosses_instead_of_rolling():
    """A robot replanned while ON the opposing lane must merge laterally at
    once (in-place turn + sideways steps), never roll with that lane's
    opposing flow."""
    p = astar(GRID, (70, 55), (30, 64))     # on the SB lane, goal south...
    assert p and not _wrong_way_steps(p)
    p = astar(GRID, (30, 64), (70, 55))     # on the NB lane going... north OK
    assert p and not _wrong_way_steps(p)
    # southbound goal from the NB lane: the only wrong-way-free options are
    # crossing to the SB lane or turning through a junction box
    p = astar(GRID, (70, 64), (26, 55))
    assert p and not _wrong_way_steps(p)


def test_spine_fallback_mode_stays_in_lane():
    """sigma >= 0.30 (SPINE fallback) used to route on the centre line =
    BOTH lanes; it must now stay in lane at reduced effectiveness - no mode
    may ever route a robot into the opposite lane (user order)."""
    nb = astar(GRID, (26, 60), (82, 60), mode="SPINE")
    sb = astar(GRID, (82, 60), (26, 60), mode="SPINE")
    nb_c = {c for r, c in nb if 35 <= r <= 75}
    sb_c = {c for r, c in sb if 35 <= r <= 75}
    assert nb_c and sb_c and not (nb_c & sb_c), (nb_c, sb_c)
    assert all(c > GRID.lanes.divider for c in nb_c), nb_c
    assert all(c < GRID.lanes.divider for c in sb_c), sb_c
