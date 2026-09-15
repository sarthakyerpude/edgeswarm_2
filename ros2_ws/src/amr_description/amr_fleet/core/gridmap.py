"""
Occupancy grid + named zone registry, loaded from YAML.

IMPORTANT FOR YOUR TEAM: this grid must describe the SAME warehouse as the
Gazebo world owned by warehouse_sim. Because I am not permitted to modify that
package, the grid lives here in amr_description/config/warehouse_grid.yaml and
must be kept aligned by hand. docs/GRID_ALIGNMENT.md explains how to verify the
alignment in under five minutes.
"""
import math
from typing import Dict, List, Optional, Set, Tuple

from .models import Cell, Pose2D


class Zone:
    """A named shared resource with a capacity.

    Modelling an aisle as ONE capacity-1 resource, rather than as a series of
    individual cells, is what makes head-on deadlock inside that aisle
    structurally impossible: two robots can never each own half of it.
    """

    def __init__(self, zone_id: str, kind: str, cells: Set[Cell], capacity: int = 1):
        self.zone_id = zone_id
        self.kind = kind              # intersection | single_lane | choke | corridor
        self.cells = cells
        self.capacity = capacity

    def contains(self, cell: Cell) -> bool:
        return cell in self.cells

    def __repr__(self):
        return f"<Zone {self.zone_id} {self.kind} cap={self.capacity} n={len(self.cells)}>"


class DynamicCell:
    """A runtime-discovered blockage with decaying trust."""

    __slots__ = ("confidence", "expiry", "sources")

    def __init__(self, confidence: float, expiry: float):
        self.confidence = confidence
        self.expiry = expiry
        self.sources: Set[str] = set()


class GridMap:
    def __init__(self, occupancy: List[str], resolution: float,
                 origin: Tuple[float, float], zones: Dict[str, Zone],
                 stations: Optional[Dict[str, Cell]] = None):
        self.static = [[1 if ch == '#' else 0 for ch in row] for row in occupancy]
        self.height = len(self.static)
        self.width = len(self.static[0]) if self.height else 0
        self.resolution = resolution
        self.origin = origin
        self.zones = zones
        self.stations = stations or {}
        self.dynamic: Dict[Cell, DynamicCell] = {}

    # ------------------------------------------------------------------ load
    @classmethod
    def from_yaml(cls, path: str) -> "GridMap":
        import yaml
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)

        meta = cfg["meta"]
        occ = cfg["occupancy"]

        # Fail loudly on a ragged grid. A single short row silently shifts every
        # coordinate to its right and produces navigation bugs that look random.
        widths = {len(r) for r in occ}
        if len(widths) != 1:
            raise ValueError(
                f"occupancy rows have inconsistent widths {sorted(widths)} - "
                f"every row must be exactly {meta['width_cells']} characters")
        if occ and len(occ[0]) != meta["width_cells"]:
            raise ValueError(
                f"width_cells={meta['width_cells']} but rows are {len(occ[0])} chars")
        if len(occ) != meta["height_cells"]:
            raise ValueError(
                f"height_cells={meta['height_cells']} but got {len(occ)} rows")

        zones: Dict[str, Zone] = {}
        for zid, z in (cfg.get("zones") or {}).items():
            cells = set()
            if "cells" in z:
                cells = {(int(r), int(c)) for r, c in z["cells"]}
            elif "rect" in z:
                r0, c0, r1, c1 = z["rect"]
                cells = {(r, c)
                         for r in range(int(r0), int(r1) + 1)
                         for c in range(int(c0), int(c1) + 1)}
            zones[zid] = Zone(zid, z.get("type", "intersection"), cells,
                              int(z.get("capacity", 1)))

        stations = {sid: (int(s["cell"][0]), int(s["cell"][1]))
                    for sid, s in (cfg.get("stations") or {}).items()}

        return cls(occ, float(meta["resolution"]),
                   tuple(meta.get("origin", [0.0, 0.0])), zones, stations)

    # ------------------------------------------------- coordinate conversion
    # Convention, fixed once here and used nowhere else:
    #     grid ROW    -> world Y
    #     grid COLUMN -> world X
    # Getting this transposed is the single most common source of "the robot
    # drives into a rack" bugs. docs/GRID_ALIGNMENT.md gives a 2-minute check.
    def world_to_cell(self, x: float, y: float) -> Cell:
        return (int((y - self.origin[1]) / self.resolution),
                int((x - self.origin[0]) / self.resolution))

    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        r, c = cell
        return ((c + 0.5) * self.resolution + self.origin[0],
                (r + 0.5) * self.resolution + self.origin[1])

    def cell_distance_m(self, a: Cell, b: Cell) -> float:
        ax, ay = self.cell_to_world(a)
        bx, by = self.cell_to_world(b)
        return math.hypot(bx - ax, by - ay)

    # ------------------------------------------------------------- queries
    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_static_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.static[cell[0]][cell[1]] == 0

    def blocked_cells(self, now: Optional[float] = None) -> Set[Cell]:
        """Cells treated as hard obstacles right now.

        A cell is hard-blocked only if it is high-confidence OR corroborated by
        two independent robots. One noisy scan must not let a single robot wall
        off an aisle for the entire fleet.
        """
        now = 0.0 if now is None else now
        return {c for c, d in self.dynamic.items()
                if d.expiry > now and (d.confidence > 0.6 or len(d.sources) >= 2)}

    def zones_on_path(self, cells: List[Cell]) -> List[str]:
        """Zones the path enters, in first-entry order, deduplicated."""
        out: List[str] = []
        for cell in cells:
            for zid, z in self.zones.items():
                if z.capacity >= 1 and z.contains(cell) and zid not in out:
                    out.append(zid)
        return out

    def zone_of(self, cell: Cell) -> Optional[str]:
        for zid, z in self.zones.items():
            if z.contains(cell):
                return zid
        return None

    # -------------------------------------------------------------- updates
    def report_blocked(self, cells, reporter: str, confidence: float, expiry: float):
        for cell in cells:
            d = self.dynamic.get(cell)
            if d is None:
                d = DynamicCell(0.0, expiry)
                self.dynamic[cell] = d
            d.sources.add(reporter)
            d.confidence = min(1.0, d.confidence + confidence * 0.5)
            d.expiry = max(d.expiry, expiry)

    def report_cleared(self, cells, reporter: str):
        for cell in cells:
            d = self.dynamic.get(cell)
            if d is None:
                continue
            d.confidence -= 0.4
            d.sources.discard(reporter)
            if d.confidence <= 0.0:
                del self.dynamic[cell]

    def merge_remote_update(self, reporter: str, blocked, cleared,
                            confidence: float, expiry: float,
                            my_pose: Optional[Pose2D] = None):
        """Merge a peer's MapUpdate, scaling trust by reporter distance."""
        trust = confidence
        if my_pose is not None and blocked:
            d = self.cell_distance_m(self.world_to_cell(my_pose.x, my_pose.y),
                                     blocked[0])
            trust = confidence * (1.0 if d < 5.0 else 0.7)
        self.report_blocked(blocked, reporter, trust, expiry)
        self.report_cleared(cleared, reporter)

    def expire(self, now: Optional[float] = None) -> int:
        """Drop stale observations. MUST be called every tick.

        Without this, a forklift that pauses in an aisle blocks that aisle
        permanently in every robot's map for the rest of the run.
        """
        now = 0.0 if now is None else now
        dead = [c for c, d in self.dynamic.items() if d.expiry <= now]
        for c in dead:
            del self.dynamic[c]
        return len(dead)
