"""
Validates config/warehouse_grid.yaml.

This test caught two real bugs during development: a row that was 51 characters
instead of 50, and zone rectangles that overlapped racks (making those zones
permanently unenterable). Run it after EVERY edit to the grid.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.astar import astar, path_length_m
from amr_fleet.core.gridmap import GridMap

GRID = pathlib.Path(__file__).resolve().parents[1] / "config" / "warehouse_grid.yaml"


def load():
    return GridMap.from_yaml(str(GRID))


def test_grid_loads_and_dimensions_match():
    gm = load()
    assert gm.width > 0 and gm.height > 0


def test_all_rows_same_width():
    """A single short row silently shifts every coordinate to its right."""
    gm = load()
    assert all(len(r) == gm.width for r in gm.static)


def test_zones_do_not_overlap_static_obstacles():
    """A zone inside a rack can never be entered, so any robot that plans
    through it deadlocks forever."""
    gm = load()
    bad = {zid: sorted(c for c in z.cells if not gm.is_static_free(c))[:5]
           for zid, z in gm.zones.items()
           if any(not gm.is_static_free(c) for c in z.cells)}
    assert not bad, f"zones overlapping obstacles: {bad}"


def test_stations_are_on_free_cells():
    gm = load()
    bad = {s: c for s, c in gm.stations.items() if not gm.is_static_free(c)}
    assert not bad, f"stations inside obstacles: {bad}"


def test_every_station_reachable_from_every_other():
    """If any station pair is unreachable, tasks between them hang forever."""
    gm = load()
    cells = list(gm.stations.items())
    for i, (sa, ca) in enumerate(cells):
        for sb, cb in cells[i + 1:]:
            assert astar(gm, ca, cb), f"{sa} -> {sb} unreachable"


def test_a_long_path_traverses_zones():
    """If no path crosses a zone, the coordination layer never activates and
    the whole project would be untestable."""
    gm = load()
    p = astar(gm, gm.stations["L1"], gm.stations["P1"])
    assert p and gm.zones_on_path(p), "no zones on a cross-warehouse path"


def test_astar_forbids_corner_cutting():
    """A diagonal step must never slip between two blocked cells, or the real
    robot clips a rack corner."""
    gm = load()
    path = astar(gm, (2, 2), gm.stations["P3"])
    assert path
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        if r1 != r2 and c1 != c2:                 # diagonal step
            assert gm.is_static_free((r2, c1)) and gm.is_static_free((r1, c2))
