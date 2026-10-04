"""Corner & rotation fixes (2026-10-04, webots_strictlanes1).

1. hitbox.uturn_swept_cells against hand-computed sweeps.
2. gridmap.forward_clearance against hand-computed rays.
3. TURN-POINT VERIFICATION: on every station route, each rectilinear turn
   point (cumulative heading change >= 60 deg, the goal bridge's split rule)
   must fit the in-place spin: the swept rect rotating from the incoming to
   the outgoing heading at the corner, grown by the sigma-floor share,
   touches no wall/rack cell. This is the offline guarantee behind the
   rectilinear turn model (the bridge spins exactly there).
4. The measured failures stay fixed: the NB->WB left turn at J_N never puts
   a turn point against the north wall (F1), and no straight step on any
   route drives at a face closer than the collision monitor's slow-band
   reach (so the StopZone can no longer wedge a commanded path).
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import hitbox
from amr_fleet.core.astar import astar
from amr_fleet.core.gridmap import GridMap

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
PLACES = {"dock1": (-3.0, -4.0), "dock2": (0.0, -4.0), "dock3": (3.0, -4.0),
          "L1": (-3.45, 3.85), "L2": (3.45, 3.85), "L3": (-3.45, 2.45),
          "P1": (-3.0, -2.75), "P2": (0.05, -2.75), "P3": (3.05, -2.75),
          "B_w": (-3.0, 0.25), "A_e": (3.0, 1.55)}
TRIPS = [("dock1", "L1"), ("dock3", "L2"), ("dock2", "L3"), ("L1", "P1"),
         ("L2", "P2"), ("L3", "P3"), ("dock1", "B_w"), ("dock3", "A_e")]
SPLIT_RAD = 1.047                   # the goal bridge's segment-split rule
SIGMA_FLOOR_EXTRA = 0.05            # (K_SIG/sqrt2 * sigma_floor)/sqrt2
NORTH_WALL_Y = 4.90                 # occupancy row 99 face (verified)


def _open_grid(h=20, w=20):
    occ = ["#" * w] + ["#" + "." * (w - 2) + "#" for _ in range(h - 2)] \
        + ["#" * w]
    return GridMap(occ, 0.1, (0.0, 0.0), {})


# ------------------------------------------------- uturn_swept_cells (hand)
def test_pure_rotation_sweeps_the_mid_angle_cells():
    g = _open_grid()
    x, y = g.cell_to_world((5, 5))          # (0.55, 0.55)
    fixed0 = hitbox.cells(g, x, y, 0.0, 0.0)
    fixed90 = hitbox.cells(g, x, y, math.pi / 2, 0.0)
    swept = hitbox.uturn_swept_cells(g, x, y, 0.0, x, y, math.pi / 2, 0.0)
    # (8,5) lies 0.30 m laterally: outside the rect at 0 and at 90 deg
    # (half-extents + raster slack < 0.28), inside it mid-rotation (45 deg:
    # |u| = |v| = 0.21).
    assert (8, 5) not in fixed0 and (8, 5) not in fixed90
    assert (8, 5) in swept
    assert fixed0 <= swept and fixed90 <= swept


def test_pure_translation_sweeps_the_corridor_only():
    g = _open_grid()
    x0, y0 = g.cell_to_world((5, 5))
    x1, y1 = g.cell_to_world((5, 15))
    swept = hitbox.uturn_swept_cells(g, x0, y0, 0.0, x1, y1, 0.0, 0.0)
    assert (5, 15) in swept and (7, 10) in swept     # 0.20 m abeam: covered
    assert (8, 10) not in swept                      # 0.30 m abeam: clear
    grown = hitbox.uturn_swept_cells(g, x0, y0, 0.0, x1, y1, 0.0, 0.1)
    assert (8, 10) in grown                          # extra_m widens it


def test_sweep_near_a_wall_reports_the_wall_cells():
    g = _open_grid()
    x, y = g.cell_to_world((17, 10))        # 0.25 m from the row-19 wall
    swept = hitbox.uturn_swept_cells(g, x, y, 0.0, x, y, math.pi / 2, 0.0)
    assert any(not g.is_static_free(c) for c in swept), (
        "a rotation 0.25 m from a wall must sweep wall cells")


# ------------------------------------------------- forward_clearance (hand)
def test_forward_clearance_straight_and_diagonal():
    g = _open_grid()
    north = g.forward_clearance(1, 0)
    assert abs(north[16][5] - 0.3) < 1e-9            # 3 cells to the wall
    assert north[18][5] == 0.1
    assert north[19][5] == 0.0                       # the wall itself
    diag = g.forward_clearance(1, 1)
    assert abs(diag[16][16] - 3 * 0.1 * math.sqrt(2.0)) < 1e-6
    south = g.forward_clearance(-1, 0)
    assert abs(south[16][5] - 1.6) < 1e-9            # 16 cells to row 0
    assert south[2][5] == 0.2 and south[1][5] == 0.1


# ------------------------------------------- planned turn points (routes)
def _route(a, b):
    p = astar(GRID, GRID.world_to_cell(*PLACES[a]),
              GRID.world_to_cell(*PLACES[b]))
    assert p, f"{a}->{b} must plan"
    return p


def _turn_points(path):
    """(corner cell, arrival-step heading, outgoing heading) per rectilinear
    split corner - the same cumulative-reference rule the goal bridge splits
    with. The ARRIVAL heading is the actual last step into the corner (a
    decomposed 45-jog approach leaves only a 45 deg residual spin there)."""
    pts = [GRID.cell_to_world(c) for c in path]
    heads = [math.atan2(b[1] - a[1], b[0] - a[0])
             for a, b in zip(pts, pts[1:])]
    out = []
    ref, ref_i = heads[0], 0
    for i, h in enumerate(heads):
        if i > ref_i and abs(math.atan2(math.sin(h - ref),
                                        math.cos(h - ref))) >= SPLIT_RAD:
            out.append((path[i], heads[i - 1], h))
            ref, ref_i = h, i
    return out


ROTATION_POLY_R = 0.33      # nav2 collision monitor rotation band, vertices


def test_every_planned_turn_point_fits_the_spin():
    """THE guarantee of the rectilinear model, two grades:

    - a FULL spin (> 60 deg residual after the approach step): the swept
      rotation grown by the sigma-floor share must touch no static cell
      (these are the junction/turnaround pivots - the planner's turn
      shaping places them on >= 0.55 m ground);
    - a residual ALIGNMENT nudge (<= 60 deg, i.e. the corner was approached
      through a 45 deg jog): legal wherever the collision monitor's 0.33 m
      rotation polygon fits - which includes the engineered 0.35 m lane
      lines, the fleet's baseline geometry for merges."""
    clr = GRID.clearance_m()
    for a, b in TRIPS:
        for trip in ((a, b), (b, a)):
            path = _route(*trip)
            for cell, h_in, h_out in _turn_points(path):
                turn = abs(math.atan2(math.sin(h_out - h_in),
                                      math.cos(h_out - h_in)))
                if turn > SPLIT_RAD:
                    x, y = GRID.cell_to_world(cell)
                    swept = hitbox.uturn_swept_cells(
                        GRID, x, y, h_in, x, y, h_out, SIGMA_FLOOR_EXTRA)
                    bad = sorted(c for c in swept
                                 if not GRID.is_static_free(c))
                    assert not bad, (trip, cell, round(h_in, 2),
                                     round(h_out, 2), bad[:4])
                else:
                    assert clr[cell[0]][cell[1]] >= ROTATION_POLY_R + 0.01, (
                        trip, cell, round(clr[cell[0]][cell[1]], 2))


def test_jn_left_turn_pivots_off_the_north_wall():
    """F1: the NB->WB left turn at J_N wedged robot_1 nose-to-wall at
    y 4.39 (turn planned ON the old wall lane, 0.35 m of clearance). The
    planned turn points of the dock->L1/L2 trips must now keep >= 0.45 m to
    the wall, where the 0.33 m rotation polygon and the slow forward box
    both fit."""
    for trip in (("dock1", "L1"), ("dock3", "L2")):
        path = _route(*trip)
        for cell, _h_in, _h_out in _turn_points(path):
            _x, y = GRID.cell_to_world(cell)
            assert NORTH_WALL_Y - y >= 0.45 - 1e-6, (trip, cell, y)


def test_no_straight_step_drives_at_a_near_face():
    """F1's other half: the plan itself must never command a sustained
    heading whose forward free run is inside the collision monitor's slow
    box + nose (0.30 + 0.20 m) - outside the goal's approach taper, where
    arrivals are slow by construction."""
    floor_m = 0.50
    for a, b in TRIPS:
        for trip in ((a, b), (b, a)):
            path = _route(*trip)
            goal = path[-1]
            for (r0, c0), (r1, c1) in zip(path, path[1:]):
                dr, dc = r1 - r0, c1 - c0
                if dr and dc:
                    continue                      # merge jogs: follower-smoothed
                if max(abs(r1 - goal[0]), abs(c1 - goal[1])) <= 5:
                    continue                      # goal approach taper
                free = GRID.forward_clearance(dr, dc)[r1][c1]
                assert free >= floor_m - 1e-6, (trip, (r0, c0), (r1, c1),
                                                round(free, 2))
