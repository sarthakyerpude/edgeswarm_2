"""R7/R8 layout consistency: Webots wbt == Gazebo sdf == grid yaml == Nav2 PGM.

Independent of scripts/warehouse_map_tool.py (its own wbt/sdf/PGM parsing), so
a drift in any one source fails here. Also pins: the four equal 1.6 m gaps,
the closed rack-end gaps (no planner route through a gap a robot cannot fit),
unchanged docks/drop-offs, pickups == rack-slot approach cells, and
fleet_sim's ground-truth zone rects == the yaml zones.
"""
import math
import pathlib
import re
import sys

import pytest
import yaml

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parent
SRC = PKG.parent
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(HERE))

from amr_fleet.core.astar import astar                  # noqa: E402
from amr_fleet.core.gridmap import GridMap              # noqa: E402
from amr_fleet.core.rack_slots import RackSlots         # noqa: E402

GRID_YAML = PKG / "config" / "warehouse_grid.yaml"
LEGACY_YAML = HERE / "fixtures" / "warehouse_grid_legacy.yaml"
WBT = SRC / "webots_warehouse_sim" / "worlds" / "warehouse.wbt"
SDF = SRC / "warehouse_sim" / "worlds" / "warehouse.sdf"
PGM = SRC / "amr_navigation_runtime" / "config" / "warehouse_map.pgm"
ITEM_MGR = SRC / "webots_warehouse_sim" / "webots_warehouse_sim" / "item_manager.py"
EPS = 1e-9
RES, OX, OY = 0.1, -6.0, -5.0
PICKUP_SLOTS = {"L1": "1R10", "L2": "3R11", "L3": "2R10"}

needs_src = pytest.mark.skipif(not (WBT.exists() and SDF.exists() and PGM.exists()),
                               reason="source tree (wbt/sdf/pgm) not available")


@pytest.fixture(scope="module")
def cfg():
    return yaml.safe_load(GRID_YAML.read_text())


@pytest.fixture(scope="module")
def gm():
    return GridMap.from_yaml(str(GRID_YAML))


def _boxes(cfg):
    return sorted([(r["x_min"], r["x_max"], r["y_min"], r["y_max"]) for r in cfg["racks"]],
                  key=lambda b: (-b[2], b[0]))


def _wbt_solids(prefix):
    text = WBT.read_text()
    return [(m.group(3), float(m.group(1)), float(m.group(2))) for m in re.finditer(
        r'translation\s+(\S+)\s+(\S+)\s+\S+\s+name\s+"(%s\w*)"' % prefix, text)]


def _wbt_box_size(def_name):
    m = re.search(r'DEF %s Box \{ size (\S+) (\S+) (\S+)' % def_name, WBT.read_text())
    return float(m.group(1)), float(m.group(2))


def _footprints(solids, size):
    return [(x - size[0] / 2, x + size[0] / 2, y - size[1] / 2, y + size[1] / 2)
            for _, x, y in solids]


def _same(a, b):
    return len(a) == len(b) and all(abs(p - q) < 1e-6 for u, v in zip(a, b) for p, q in zip(u, v))


@needs_src
def test_wbt_racks_match_yaml_racks(cfg):
    racks = _footprints(_wbt_solids("rack_"), _wbt_box_size("RACK_BOX"))
    assert _same(sorted(racks, key=lambda b: (-b[2], b[0])), _boxes(cfg))


@needs_src
def test_sdf_racks_and_endcaps_match_wbt():
    sdf = SDF.read_text()
    links = {n: (float(x), float(y)) for n, x, y in re.findall(
        r'<link name="((?:rack_row|wall_endcap_row)\d_\w+)">\s*<pose>(\S+) (\S+) ', sdf)}
    wbt = {n: (x, y) for n, x, y in _wbt_solids("rack_row") + _wbt_solids("wall_endcap_")}
    assert len(wbt) == 12 and links == wbt
    assert len(re.findall(r'<size>5\.0 0\.6 1\.6</size>', sdf)) == 12
    assert len(re.findall(r'<size>0\.2 0\.6 1\.6</size>', sdf)) == 12


def _raster(boxes, w, h):
    """Cell occupied iff its square overlaps a box with positive area; plus
    the outermost ring (the wall faces)."""
    g = [[0] * w for _ in range(h)]
    for r in range(h):
        for c in range(w):
            if r in (0, h - 1) or c in (0, w - 1):
                g[r][c] = 1
                continue
            x0, y0 = OX + c * RES, OY + r * RES
            for b in boxes:
                if (min(b[1], x0 + RES) - max(b[0], x0) > 1e-6
                        and min(b[3], y0 + RES) - max(b[2], y0) > 1e-6):
                    g[r][c] = 1
                    break
    return g


@needs_src
def test_grid_and_pgm_equal_rasterised_world(cfg, gm):
    boxes = (_footprints(_wbt_solids("rack_"), _wbt_box_size("RACK_BOX"))
             + _footprints(_wbt_solids("wall_endcap_"), _wbt_box_size("ENDCAP_BOX")))
    truth = _raster(boxes, gm.width, gm.height)
    grid = [[0 if gm.is_static_free((r, c)) else 1 for c in range(gm.width)]
            for r in range(gm.height)]
    assert sum(truth[r][c] != grid[r][c] for r in range(gm.height) for c in range(gm.width)) == 0
    toks = [t for t in PGM.read_bytes().split() if not t.startswith(b"#")]
    assert toks[0] == b"P2" and (int(toks[1]), int(toks[2])) == (gm.width, gm.height)
    pix = [int(t) for t in toks[4:]]
    bad = sum((pix[(gm.height - 1 - r) * gm.width + c] < 128) != bool(grid[r][c])
              for r in range(gm.height) for c in range(gm.width))
    assert bad == 0


def test_every_gap_equals_aisle_width(cfg):
    W = cfg["aisle_width_m"]
    boxes = _boxes(cfg)
    rows = sorted({(b[2], b[3]) for b in boxes}, reverse=True)
    cols = sorted({(b[0], b[1]) for b in boxes})
    assert len(rows) == 3 and len(cols) == 2
    gaps = [5.0 - rows[0][1], rows[0][0] - rows[1][1], rows[1][0] - rows[2][1],
            cols[1][0] - cols[0][1]]
    assert all(abs(g - W) < EPS for g in gaps), gaps
    assert rows[2][0] - (-5.0) >= 3.4 - EPS          # south operating area
    assert all(abs(b[3] - b[2] - 0.6) < EPS for b in boxes)
    # 2 robots (0.40 x 0.32) abreast at W/4 from each face, spinning in place
    spin_r = math.hypot(0.20, 0.16)
    assert W / 4 - spin_r > 0.1 and W / 2 - 2 * spin_r > 0.25


def test_rack_end_gaps_are_closed(cfg, gm):
    """No free cell between a rack end and the side wall, so A* cannot plan
    through the 0.2 m gap even when the spine is blocked."""
    for b in _boxes(cfg):
        r0 = int(round((b[2] - OY) / RES))
        r1 = int(round((b[3] - OY) / RES)) - 1
        for r in range(r0, r1 + 1):
            for c in range(gm.width):
                x = OX + (c + 0.5) * RES
                if x < b[0] if b[0] < 0 else x > b[1]:
                    assert not gm.is_static_free((r, c)), (r, c)
    spine = {(r, c) for r in range(28, 84) for c in range(52, 68)}
    p = astar(gm, gm.stations["L3"], gm.stations["P1"], blocked=spine,
              max_expansions=200000)
    assert p == []


def test_docks_and_dropoffs_unchanged(cfg):
    legacy = yaml.safe_load(LEGACY_YAML.read_text())
    for k in ("P1", "P2", "P3", "DROPOFF_1", "DROPOFF_2", "DROPOFF_3",
              "CHG1", "CHG2", "CHG3"):
        assert cfg["stations"][k] == legacy["stations"][k], k
    if WBT.exists():
        text = WBT.read_text()
        spawns = re.findall(r'DEF ROBOT_\d Robot \{\s*translation (\S+) (\S+)', text)
        assert [tuple(map(float, s)) for s in spawns] == [(-3, -4), (0, -4), (3, -4)]
        drops = re.findall(r'translation (\S+) (\S+) 0\.006\s+children \[\s*DEF DROPOFF_DISC', text)
        drops += re.findall(r'Pose \{ translation (\S+) (\S+) 0\.006 children \[ USE DROPOFF_DISC', text)
        assert sorted(tuple(map(float, d)) for d in drops) == [(-3, -2.8), (0, -2.8), (3, -2.8)]


def test_pickups_are_rack_slot_approach_cells_with_items(cfg, gm):
    rs = RackSlots.from_config(cfg)
    homes = []
    for st, label in PICKUP_SLOTS.items():
        cell = tuple(cfg["stations"][st]["cell"])
        assert cell == rs.approach_cell(label) == tuple(cfg["stations"]["PICKUP_" + st[1]]["cell"])
        homes.append(gm.cell_to_world(cell))
    if WBT.exists():
        items = [(float(x), float(y)) for x, y in re.findall(
            r'DEF ITEM_\d Solid \{\s*translation (\S+) (\S+)', WBT.read_text())]
        assert _same(items, homes)
    if ITEM_MGR.exists():
        m = re.search(r'ITEM_HOMES = (\[[^\]]*\])', ITEM_MGR.read_text())
        assert _same(eval(m.group(1)), homes)     # literal list of tuples


def test_slot_convention_spot_checks(cfg):
    rs = RackSlots.from_config(cfg)
    W = cfg["aisle_width_m"]
    boxes = _boxes(cfg)
    for label in ("1R1", "1R20", "2R1", "6R10", "12R5", "12R20"):
        side, sec = map(int, label.split("R"))
        rack = (side + 1) // 2
        b = boxes[rack - 1]
        north = side % 2 == 1
        x = b[0] + (sec - 0.5) * (b[1] - b[0]) / 20
        y = (b[3] + W / 4) if north else (b[2] - W / 4)
        s = rs.slot(label)
        assert s.rack == rack
        assert abs(s.approach[0] - x) < EPS and abs(s.approach[1] - y) < EPS
        assert s.approach[2] == 0.0


def test_fleet_sim_ground_truth_rects_match_yaml(cfg):
    import fleet_sim as fs
    for zid, rect in fs.PLAN_ZONES.items():
        assert list(rect) == cfg["zones"][zid]["rect"], zid
    rows = sorted((int(round((b[2] - OY) / RES)), int(round((b[3] - OY) / RES)) - 1)
                  for b in _boxes(cfg))
    assert sorted(set(rows)) == sorted(fs.RACK_ROWS)
