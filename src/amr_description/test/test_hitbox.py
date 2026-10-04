"""Oriented-rectangle hitbox (core/hitbox.py) - pure geometry (R2)."""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import pytest

from amr_fleet.core import hitbox as hb
from amr_fleet.core.gridmap import GridMap

GRID = GridMap(["." * 60] * 60, 0.1, (-3.0, -3.0), {})


# ----------------------------------------------------------------- corners --
def test_front_edge_midpoint_is_pose_plus_half_length_along_heading():
    """AC11: the drawn front edge's midpoint == pose + 0.20*(cos th, sin th)."""
    for x, y, th in ((0.0, 0.0, 0.0), (1.2, -0.7, 0.83), (-2.0, 3.0, -2.5)):
        geo = hb.marker_geometry(x, y, th, 0.05)
        (ax, ay), (bx, by) = geo["front_edge"]
        mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
        assert abs(mx - (x + hb.HB_HALF_L * math.cos(th))) < 1e-9
        assert abs(my - (y + hb.HB_HALF_L * math.sin(th))) < 1e-9


def test_corners_order_and_inflation():
    cs = hb.corners(0.0, 0.0, 0.0)
    assert cs[0] == pytest.approx((0.20, 0.205))     # front-left
    assert cs[1] == pytest.approx((0.20, -0.205))    # front-right
    assert cs[2] == pytest.approx((-0.20, -0.205))
    assert cs[3] == pytest.approx((-0.20, 0.205))
    big = hb.corners(0.0, 0.0, 0.0, infl=0.1)
    assert big[0] == pytest.approx((0.30, 0.305))


# ---------------------------------------------------------------- rect_gap --
def test_parallel_lanes_on_the_spine_gap():
    """Two robots at +-0.4 m of the 1.6 m spine, both heading along the
    aisle: wheel-to-wheel gap is 0.8 - 2*0.205 = 0.39 m."""
    a = (-0.4, 0.0, math.pi / 2)
    b = (0.4, 1.0, -math.pi / 2)
    g_abreast = hb.rect_gap((-0.4, 0.0, math.pi / 2), (0.4, 0.0, -math.pi / 2))
    assert g_abreast == pytest.approx(0.39, abs=1e-9)
    # Offset fore-aft the SAT gap can only be larger or equal.
    assert hb.rect_gap(a, b) >= 0.39 - 1e-9


def test_overlap_gives_zero_and_ninety_degree_corner():
    assert hb.rect_gap((0.0, 0.0, 0.0), (0.1, 0.05, 0.4)) == 0.0
    # 90-degree pair, axis-aligned: gap along x = 1.0 - 0.20 - 0.205.
    g = hb.rect_gap((0.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2))
    assert g == pytest.approx(1.0 - hb.HB_HALF_L - hb.HB_HALF_W, abs=1e-9)


def test_rect_gap_sat_is_a_lower_bound_on_true_gap():
    """Corner-to-corner at 45 deg: SAT must never exceed the true corner
    distance (conservative for a stop threshold)."""
    d = 0.8
    a = (0.0, 0.0, 0.0)
    b = (d / math.sqrt(2), d / math.sqrt(2), 0.0)
    true_gap = math.hypot(b[0] - 0.20 - (b[0] - 0.20), 0) # placeholder
    # true corner gap: distance between corner (0.2,0.205) and
    # (b0-0.2, b1-0.205)
    cx, cy = 0.20, 0.205
    true_gap = math.hypot((b[0] - 0.20) - cx, (b[1] - 0.205) - cy)
    sat = hb.rect_gap(a, b)
    assert 0.0 < sat <= true_gap + 1e-9


def test_rect_gap_cutoff_early_out_is_a_lower_bound():
    a, b = (0.0, 0.0, 0.0), (5.0, 0.0, 0.0)
    fast = hb.rect_gap(a, b, cutoff=1.0)
    exact = hb.rect_gap(a, b)
    assert fast <= exact
    assert fast == pytest.approx(5.0 - 2 * hb.HB_R_CIRC)


def test_rect_gap_symmetry():
    a, b = (0.1, -0.2, 0.7), (0.9, 0.5, -1.2)
    assert hb.rect_gap(a, b) == pytest.approx(hb.rect_gap(b, a), abs=1e-12)


# ------------------------------------------------------------------ sigmas --
def test_clean_sigma_floors_degraded_and_nonfinite():
    assert hb.clean_sigma(0.0) == hb.SIGMA_FLOOR
    assert hb.clean_sigma(0.12) == 0.12
    assert hb.clean_sigma(0.08, degraded=True) == hb.SIGMA_DEGRADED_MIN
    assert hb.clean_sigma(float("nan")) == hb.SIGMA_NONFINITE
    assert hb.clean_sigma(float("inf")) == hb.SIGMA_NONFINITE
    assert hb.clean_sigma(None) == hb.SIGMA_NONFINITE


def test_gap_stop_formula():
    s = 0.08
    expect = 0.12 + hb.K_SIG * math.sqrt(2 * s * s)
    assert hb.gap_stop(s, s) == pytest.approx(expect)
    # closing-speed margin saturates at V_CLOSE_REF_MPS (the cruise speed)
    assert hb.gap_stop(s, s, v_close=0.5 * hb.V_CLOSE_REF_MPS) \
        == pytest.approx(expect + 0.5 * hb.G_MARGIN_VCLOSE)
    assert hb.gap_stop(s, s, v_close=5.0) \
        == pytest.approx(expect + hb.G_MARGIN_VCLOSE)
    assert hb.gap_stop(s, s, infl=0.3) == pytest.approx(expect + 0.3)
    # floors applied inside
    assert hb.gap_stop(0.0, 0.0) == pytest.approx(
        0.12 + hb.K_SIG * math.sqrt(2) * hb.SIGMA_FLOOR)


def test_two_safety_outlines_touch_at_exactly_the_pairwise_threshold():
    """per_robot_inflation contract: infl(s) + infl(s) == gap_stop(s, s, 0)."""
    for s in (0.05, 0.08, 0.16, 0.3):
        assert 2 * hb.per_robot_inflation(s) == pytest.approx(
            hb.gap_stop(s, s, 0.0), abs=1e-12)


def test_per_robot_inflation_stale_reach_caps():
    base = hb.per_robot_inflation(0.05)
    assert hb.per_robot_inflation(0.05, age_s=1.0, stale=True) == pytest.approx(
        base + min(hb.REACH_CAP_M, hb.V_MAX * 1.0))
    assert hb.per_robot_inflation(0.05, age_s=60.0, stale=True) == pytest.approx(
        base + hb.REACH_CAP_M)
    assert hb.per_robot_inflation(0.05, age_s=60.0, stale=False) == base


# ------------------------------------------------------------------- spin ---
def test_is_spinning_on_rate_and_on_path_turn():
    assert hb.is_spinning(0.4, None)
    assert hb.is_spinning(-0.35, [])
    assert not hb.is_spinning(0.1, None)
    straight = [(0.1 * i, 0.0) for i in range(10)]
    assert not hb.is_spinning(0.0, straight)
    # 90-degree turn 0.3 m ahead -> spin disc
    corner = [(0.0, 0.0), (0.15, 0.0), (0.3, 0.0), (0.3, 0.15), (0.3, 0.3)]
    assert hb.is_spinning(0.0, corner)
    # same turn but 1 m ahead -> not yet
    far = [(0.2 * i, 0.0) for i in range(6)] + [(1.0, 0.2), (1.0, 0.4)]
    assert not hb.is_spinning(0.0, far)


# -------------------------------------------------------------- rasterise ---
def test_cells_cover_the_rect_and_grow_with_extra():
    got = hb.cells(GRID, 0.05, 0.05, 0.0, 0.0)
    # pose cell always included
    assert GRID.world_to_cell(0.05, 0.05) in got
    # every included centre is inside the grown rect (brute check)
    half_diag = 0.5 * GRID.resolution * math.sqrt(2)
    for cell in got:
        cx, cy = GRID.cell_to_world(cell)
        assert abs(cx - 0.05) <= hb.HB_HALF_L + half_diag + 1e-9
        assert abs(cy - 0.05) <= hb.HB_HALF_W + half_diag + 1e-9
    # extra_m strictly grows the footprint and keeps the original
    grown = hb.cells(GRID, 0.05, 0.05, 0.0, 0.2)
    assert got < grown


def test_cells_respect_heading():
    """Grown asymmetric (longer than wide): heading 0 spans more columns,
    heading pi/2 more rows. (The bare rect is nearly square, so the
    orientation only shows once the length dominates.)"""
    # stretch the rect along +x with a probe: compare the extreme extents
    got = hb.cells(GRID, 0.05, 0.05, 0.0, 0.0)
    rot = hb.cells(GRID, 0.05, 0.05, math.pi / 2, 0.0)
    # rotating the near-square rect by 90 deg about a cell centre transposes
    # the footprint around (30, 30)
    assert {(30 + (c - 30), 30 + (r - 30)) for r, c in got} == rot


def test_obstacle_cells_grow_with_sigma_and_are_bounded():
    small = hb.obstacle_cells(GRID, 0.05, 0.05, 0.0, 0.05)
    big = hb.obstacle_cells(GRID, 0.05, 0.05, 0.0, 0.20)
    assert small < big
    # bounded by the inflated circumscribed radius (+ half-diag growth)
    half_diag = 0.5 * GRID.resolution * math.sqrt(2)
    r_lim = math.hypot(hb.HB_HALF_L + 0.10 + 2 * 0.05 + half_diag,
                       hb.HB_HALF_W + 0.10 + 2 * 0.05 + half_diag)
    for r, c in small:
        cx, cy = GRID.cell_to_world((r, c))
        assert math.hypot(cx - 0.05, cy - 0.05) <= r_lim + 0.1


# ------------------------------------------------------- point / disc gaps --
def test_point_and_disc_gaps():
    pose = (0.0, 0.0, 0.0)
    assert hb.point_rect_gap(pose, 0.0, 0.0, 0.0) == 0.0          # inside
    assert hb.point_rect_gap(pose, 0.0, 1.0, 0.0) == pytest.approx(0.8)
    assert hb.rect_disc_gap(pose, 0.0, 1.0, 0.0, 0.3) == pytest.approx(0.5)
    assert hb.rect_disc_gap(pose, 0.0, 0.3, 0.0, 0.3) == 0.0      # overlap
    assert hb.disc_gap(0.0, 0.0, 0.286, 1.0, 0.0, 0.286) == pytest.approx(
        1.0 - 0.572)


# ------------------------------------------------------------------- RViz ---
def test_marker_geometry_safety_outline_is_separate_and_wider_than_body():
    geo = hb.marker_geometry(1.0, 2.0, 0.3, 0.08)
    assert set(geo) == {"body", "front_edge", "arrow", "nose", "safety"}
    # closed polylines
    assert geo["body"][0] == pytest.approx(geo["body"][-1])
    assert geo["safety"][0] == pytest.approx(geo["safety"][-1])
    # safety outline strictly contains the body outline (same centre)
    infl = hb.per_robot_inflation(0.08)
    for px, py in geo["body"]:
        # transform back to body frame and check inside the inflated rect
        u = math.cos(0.3) * (px - 1.0) + math.sin(0.3) * (py - 2.0)
        v = -math.sin(0.3) * (px - 1.0) + math.cos(0.3) * (py - 2.0)
        assert abs(u) <= hb.HB_HALF_L + infl + 1e-9
        assert abs(v) <= hb.HB_HALF_W + infl + 1e-9
    # arrow is 0.35 m along the heading
    (x0, y0), (x1, y1) = geo["arrow"]
    assert math.hypot(x1 - x0, y1 - y0) == pytest.approx(0.35)
    assert math.atan2(y1 - y0, x1 - x0) == pytest.approx(0.3)
    # wheel bumps present: body reaches the wheel half-width
    vmax = max(abs(-math.sin(0.3) * (px - 1.0) + math.cos(0.3) * (py - 2.0))
               for px, py in geo["body"])
    assert vmax == pytest.approx(hb.HB_HALF_W, abs=1e-9)
