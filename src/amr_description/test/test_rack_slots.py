"""R8 rack slot naming: '<side>R<section>' on 6 racks x 2 faces x 20 sections.

Runs against config/warehouse_grid.yaml when it carries the 'racks:' section
(R7 geometry), else against a synthetic W=1.5 fixture of the proposed layout.
"""
import collections
import math
import pathlib
import sys

import pytest
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.astar import astar
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.rack_slots import (SECTIONS_PER_FACE, SIDES, RackSlots,
                                       all_labels, format_label, parse_label)

PKG = pathlib.Path(__file__).resolve().parents[1]
GRID = PKG / "config" / "warehouse_grid.yaml"
SLOTS_YAML = PKG / "config" / "rack_slots.yaml"
DOCKS_XY = [(-3.0, -4.0), (0.0, -4.0), (3.0, -4.0)]
EPS = 1e-9


def _fixture_cfg(W=1.5):
    """Proposed equal-aisle layout: walls at x=+-6, y=+-5; racks 0.6 deep."""
    y1max = 5.0 - W
    rows = [(y1max - 0.6, y1max)]
    rows.append((rows[0][0] - W - 0.6, rows[0][0] - W))
    rows.append((rows[1][0] - W - 0.6, rows[1][0] - W))
    racks, rid = [], 1
    for y0, y1 in rows:
        for x0, x1 in ((-5.8, -W / 2), (W / 2, 5.8)):
            racks.append({"id": rid, "x_min": x0, "x_max": x1,
                          "y_min": y0, "y_max": y1})
            rid += 1
    occ = []
    for r in range(100):
        row = []
        for c in range(120):
            cx, cy = -6.0 + (c + 0.5) * 0.1, -5.0 + (r + 0.5) * 0.1
            wall = r in (0, 99) or c in (0, 119)
            rack = any(rk["x_min"] < cx + 0.05 - EPS and cx - 0.05 + EPS < rk["x_max"]
                       and rk["y_min"] < cy + 0.05 - EPS and cy - 0.05 + EPS < rk["y_max"]
                       for rk in racks)
            row.append("#" if wall or rack else ".")
        occ.append("".join(row))
    return {"meta": {"resolution": 0.1, "width_cells": 120, "height_cells": 100,
                     "origin": [-6.0, -5.0]},
            "aisle_width_m": W, "racks": racks, "occupancy": occ}


def _load():
    cfg = yaml.safe_load(GRID.read_text())
    if "racks" in cfg and "aisle_width_m" in cfg:
        return RackSlots.from_config(cfg), GridMap.from_yaml(str(GRID)), "yaml"
    cfg = _fixture_cfg()
    gm = GridMap(cfg["occupancy"], 0.1, (-6.0, -5.0), {})
    return RackSlots.from_config(cfg), gm, "fixture"


@pytest.fixture(scope="module")
def world():
    return _load()


# ------------------------------------------------------------ convention ----
def test_convention_anchor_labels(world):
    rs, _, _ = world
    r1, r2, r6 = rs.racks[1], rs.racks[2], rs.racks[6]
    L1 = rs.section_length(1)
    s = rs.slot("1R1")       # west end of rack 1 north face
    assert (s.rack, s.face) == (1, "N")
    assert s.x0 == pytest.approx(r1.x_min) and s.face_y == pytest.approx(r1.y_max)
    s = rs.slot("1R20")      # east end of rack 1 north face
    assert s.x1 == pytest.approx(r1.x_max) and s.face_y == pytest.approx(r1.y_max)
    s = rs.slot("2R1")       # rack 1 south face, west end
    assert (s.rack, s.face) == (1, "S")
    assert s.x0 == pytest.approx(r1.x_min) and s.face_y == pytest.approx(r1.y_min)
    s = rs.slot("3R1")       # rack 2 north face, west end
    assert (s.rack, s.face) == (2, "N")
    assert s.x0 == pytest.approx(r2.x_min) and s.face_y == pytest.approx(r2.y_max)
    s = rs.slot("12R20")     # rack 6 south face, east end
    assert (s.rack, s.face) == (6, "S")
    assert s.x1 == pytest.approx(r6.x_max) and s.face_y == pytest.approx(r6.y_min)
    assert rs.slot("1R6").centre[0] == pytest.approx(r1.x_min + 5.5 * L1)


def test_rack_numbering_is_left_right_top_bottom(world):
    rs, _, _ = world
    c = {k: ((r.x_min + r.x_max) / 2, (r.y_min + r.y_max) / 2)
         for k, r in rs.racks.items()}
    for west, east in ((1, 2), (3, 4), (5, 6)):
        assert c[west][0] < 0 < c[east][0]
        assert c[west][1] == pytest.approx(c[east][1])
    assert c[1][1] > c[3][1] > c[5][1]
    # The yaml ids (when present) agree with the positional numbering.
    for k, b in rs.yaml_ids.items():
        if k in rs.racks:
            assert rs.racks[k].x_min == pytest.approx(b.x_min)
            assert rs.racks[k].y_min == pytest.approx(b.y_min)


def test_240_unique_labels_and_round_trip(world):
    rs, _, _ = world
    labels = all_labels()
    assert len(labels) == 240 == len(set(labels)) == len(SIDES) * SECTIONS_PER_FACE
    assert labels[0] == "1R1" and labels[-1] == "12R20"
    for lab in labels:
        side, sec = parse_label(lab)
        assert format_label(side, sec) == lab
        assert rs.slot(lab).label == lab
        assert rs.label_at(*rs.approach_pose(lab)[:2]) == lab
    assert parse_label(" 12r5 ") == (12, 5)
    for bad in ("0R1", "13R1", "1R0", "1R21", "R5", "1-5", ""):
        with pytest.raises(ValueError):
            parse_label(bad)


def test_sections_tile_each_face_equally(world):
    rs, _, _ = world
    for side, (rack_id, _) in SIDES.items():
        r = rs.racks[rack_id]
        segs = [rs.slot(format_label(side, n)) for n in range(1, SECTIONS_PER_FACE + 1)]
        L = (r.x_max - r.x_min) / SECTIONS_PER_FACE
        assert segs[0].x0 == pytest.approx(r.x_min)
        assert segs[-1].x1 == pytest.approx(r.x_max)
        for a, b in zip(segs, segs[1:]):
            assert b.x0 == pytest.approx(a.x1)               # contiguous, W->E
        for s in segs:
            assert s.x1 - s.x0 == pytest.approx(L)
            assert s.centre[1] == pytest.approx(rs.face_y(side))


# ------------------------------------------------------------- geometry -----
def test_approach_pose_in_aisle_at_quarter_width(world):
    rs, _, _ = world
    W = rs.aisle_width_m
    rack_boxes = list(rs.racks.values())
    for s in rs.slots():
        ax, ay, yaw = s.approach
        assert yaw == 0.0
        sign = 1 if s.face == "N" else -1
        assert (ay - s.face_y) * sign == pytest.approx(W / 4)
        assert ax == pytest.approx(s.centre[0])
        # inside the aisle: not inside any rack, and inside the walls
        assert -5.0 < ay < 5.0 and -6.0 < ax < 6.0
        for b in rack_boxes:
            assert not (b.x_min <= ax <= b.x_max and b.y_min <= ay <= b.y_max)
        # the far half of the aisle (W/2 wide) is clear of every rack face
        far = s.face_y + sign * W / 2
        for b in rack_boxes:
            if b.x_min < ax < b.x_max:
                assert not (min(s.face_y, far) + EPS < b.y_max
                            and b.y_min < max(s.face_y, far) - EPS)


def test_approach_cells_free_clear_and_reachable_from_docks(world):
    rs, gm, _ = world
    clear = gm.clearance_m()
    # BFS over free cells from every dock: every approach cell is connected.
    docks = [gm.world_to_cell(x, y) for x, y in DOCKS_XY]
    seen = set(docks)
    q = collections.deque(docks)
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (r + dr, c + dc)
            if n not in seen and gm.in_bounds(n) and gm.is_static_free(n):
                seen.add(n)
                q.append(n)
    for s in rs.slots():
        cell = s.approach_cell
        assert gm.is_static_free(cell), f"{s.label} approach {cell} occupied"
        # the 0.32 m chassis fits (half-width 0.16 m) at the approach cell
        assert clear[cell[0]][cell[1]] >= 0.16, (s.label, clear[cell[0]][cell[1]])
        assert cell in seen, f"{s.label} approach {cell} not reachable"
    # Real A* from the centre dock to both ends and the middle of every side.
    for side in SIDES:
        for sec in (1, 10, SECTIONS_PER_FACE):
            cell = rs.approach_cell(format_label(side, sec))
            path = astar(gm, docks[1], cell, max_expansions=200000)
            assert path and path[-1] == cell, (side, sec)


def test_world_to_cell_matches_gridmap(world):
    rs, gm, _ = world
    for s in rs.slots():
        r, c = s.approach_cell
        x, y = gm.cell_to_world((r, c))
        assert abs(x - s.approach[0]) <= 0.05 + 1e-6
        assert abs(y - s.approach[1]) <= 0.05 + 1e-6


def test_generated_rack_slots_yaml_is_current(world):
    rs, _, src = world
    if src != "yaml":
        pytest.skip("grid has no racks: section yet")
    assert SLOTS_YAML.exists(), "run scripts/gen_rack_slots.py"
    data = yaml.safe_load(SLOTS_YAML.read_text())
    assert len(data["slots"]) == 240
    assert data["aisle_width_m"] == pytest.approx(rs.aisle_width_m)
    for s in rs.slots():
        row = data["slots"][s.label]
        assert tuple(row["approach_cell"]) == s.approach_cell
        assert row["approach"][0] == pytest.approx(s.approach[0], abs=1e-4)
        assert row["approach"][1] == pytest.approx(s.approach[1], abs=1e-4)


def test_fixture_geometry_also_valid():
    """The proposed W=1.5 layout (fixture) satisfies the same checks."""
    cfg = _fixture_cfg(1.5)
    rs = RackSlots.from_config(cfg)
    gm = GridMap(cfg["occupancy"], 0.1, (-6.0, -5.0), {})
    for s in rs.slots():
        assert gm.is_static_free(s.approach_cell), s.label
    assert math.isclose(rs.approach_offset_m, 0.375)
