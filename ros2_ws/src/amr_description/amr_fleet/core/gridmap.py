"""
Occupancy grid + named zone registry, loaded from YAML.

IMPORTANT FOR YOUR TEAM: this grid must describe the SAME warehouse as the
Gazebo world owned by warehouse_sim. Because I am not permitted to modify that
package, the grid lives here in amr_description/config/warehouse_grid.yaml and
must be kept aligned by hand. docs/GRID_ALIGNMENT.md explains how to verify the
alignment in under five minutes.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from .models import Cell, Pose2D

# Clearance-aware planning (see GridMap.clearance_cost).
# A cell whose centre is closer than this to a wall/rack face cannot safely
# hold the robot: wheel-inclusive half-width 0.205 m + 2 cm margin, i.e. the
# lethal band now covers the wheels, not only the 0.32 m chassis (was 0.18).
# astar.free() additionally rejects clearance below the bare chassis
# half-width 0.16 m as a HARD wall, closing the cost-1000 leak the layout
# verifier measured (26/55 routes through a 0.1 m strip when the spine was
# blocked, pre end-caps).
CLEARANCE_LETHAL_M = 0.225
# Must dominate stacked soft penalties (coordinator replan(avoid_zones=held)
# adds 50/cell per zone on top of the 8/cell timeout penalty and congestion),
# or a penalised aisle makes the side strips look cheaper again.
CLEARANCE_LETHAL_COST = 1000.0
# Clearance beyond this counts as "open floor": no centring pull. It is also
# the radius of the window used to find the local corridor centre, so it must
# exceed the half-width of the widest aisle that should be centred (the 2 m
# centre corridor -> 1.0 m) but stay below rack thickness + narrow-aisle
# half-width so the window does not read the neighbouring aisle's centre.
CLEARANCE_CAP_M = 1.0
# Extra cost per cell (in units of one straight step) at zero clearance,
# falling linearly to 0 on the local centre line. 4.0 makes one row (0.1 m)
# off-centre in the 1.5 m aisle cost ~0.5 per cell, above the 0.4 congestion
# penalty, so robots hold the centre line instead of drifting toward racks.
CLEARANCE_WEIGHT = 4.0

_SQRT2 = 1.4142136

# ------------------------------------------------------------- lanes (R3/R7)
# Directional A* step costs inside the lane band (road model: every band,
# every mode; legacy reservations: the spine band in mode == 'LANES' only).
#
# STRICT DIRECTIONAL LANES (user order, 2026-10-04): driving ALONG a band
# against its keep-right direction is PLANNER-FORBIDDEN - it costs
# LANE_WRONG_SIDE_HARD (CLEARANCE_LETHAL_COST-class) per step - EXCEPT
# inside a crossing zone (GridMap.in_crossing_zone: a junction box where two
# bands overlap, or a yaml traffic.turnarounds box at an aisle dead end),
# where the soft LANE_WRONG_SIDE applies so a U-turn can swing through.
# LATERAL steps stay soft everywhere: crossing the divider sideways is how a
# robot merges onto its own lane or reaches a far-side rack slot, and the
# coordinator gates the actual crossing against oncoming traffic at runtime.
# A band therefore always stays escapable (sideways), while planning can
# never route WITH the flow of the opposite lane outside a crossing zone.
LANE_WRONG_SIDE = 15.0      # soft: inside crossing zones only
# The forbidden wrong-way cost must dominate EVERY other per-cell toll a
# route can stack (refuge cost 1000 + zone penalties + congestion): a
# wrong-way step beats a legal detour only if its surcharge is below the
# per-cell toll it saves (measured: inside refuge R_BW, wrong-way diagonals
# at HARD=1000 halved the 1000/cell refuge count and still won). 20x the
# lethal class keeps it a cost (a trapped robot can still plan out through
# it as an absolute last resort) while no realistic toll stack reaches it.
LANE_WRONG_SIDE_HARD = 20.0 * CLEARANCE_LETHAL_COST   # forbidden elsewhere
LANE_OFF_CENTRE = 4.0       # at saturation, LANE_OFF_SAT_CELLS off centre
LANE_OFF_SAT_CELLS = 5
LANE_LATERAL = 3.0          # per lateral (lane-changing) step
NEAR_WALL_KEEP_M = 0.30     # centring kept inside the band below this


@dataclass
class LaneBand:
    """The two-way lane band on the spine (yaml traffic.lanes.spine).

    axis 'v': travel runs along ROWS; lanes are split by COLUMN around
    `divider` - positive travel (increasing row = north) keeps columns above
    the divider (centre_pos), negative travel below it (centre_neg).
    axis 'h' mirrors rows and columns."""
    rect: Tuple[int, int, int, int]       # r0, c0, r1, c1 inclusive
    axis: str
    centre_pos: float
    centre_neg: float
    name: str = ""                        # yaml key (spine, aisle_n, ...)

    @property
    def divider(self) -> float:
        return 0.5 * (self.centre_pos + self.centre_neg)

    def in_band(self, r: int, c: int) -> bool:
        r0, c0, r1, c1 = self.rect
        return r0 <= r <= r1 and c0 <= c <= c1

    def step_cost(self, r0: int, c0: int, r1: int, c1: int,
                  lateral: bool = True, along_scale: float = 1.0,
                  lateral_scale: float = 1.0,
                  crossing: bool = False) -> float:
        """Directional surcharge for the step (r0,c0)->(r1,c1); 0 outside
        the band.

        STRICT LANES: an along-travel step on the wrong side of the divider
        costs LANE_WRONG_SIDE_HARD (planner-forbidden) unless `crossing`
        (the target cell lies in a junction box or turnaround - see
        GridMap.in_crossing_zone), where the soft LANE_WRONG_SIDE applies.
        The wrong-side term is NEVER scaled down by along_scale: the start
        taper may fade the off-centre pull (merge on the move) but a robot
        must never ROLL against the flow, not even on its first cells.
        The band stays escapable through LATERAL steps, which cost
        LANE_LATERAL * lateral_scale whatever the side. lateral=False drops
        the lane-change term (junction boxes: a step across one band is a
        step along the crossing one)."""
        if not self.in_band(r1, c1):
            return 0.0
        if self.axis == "v":
            along, across, pos = r1 - r0, c1 - c0, float(c1)
        else:
            along, across, pos = c1 - c0, r1 - r0, float(r1)
        cost = 0.0
        if along != 0 and along_scale > 0.0:
            centre = self.centre_pos if along > 0 else self.centre_neg
            # Wrong side = the other half of the divider from my lane. Works
            # for either orientation (aisle bands keep EB on the LOWER rows,
            # centre_pos < centre_neg; the spine has centre_pos above).
            if (pos - self.divider) * (centre - self.divider) < 0:
                cost += LANE_WRONG_SIDE if crossing else LANE_WRONG_SIDE_HARD
            cost += LANE_OFF_CENTRE * along_scale * min(
                1.0, abs(pos - centre) / LANE_OFF_SAT_CELLS)
        if across != 0 and lateral:
            cost += LANE_LATERAL * lateral_scale
        return cost


def lane_step_cost(bands, r0: int, c0: int, r1: int, c1: int,
                   along_scale: float = 1.0,
                   lateral_scale: float = 1.0,
                   crossing: bool = False) -> float:
    """Keep-right cost of one A* step over ALL lane bands (road model).

    One band at the target cell: its full cost. Two or more (a junction
    box where an aisle crosses the spine): every band charges its along-
    travel terms but none charges lane changes, so a vertical step obeys
    the spine lanes and a horizontal one the aisle lanes - a turn becomes
    a square L onto the target lane instead of a diagonal through the box
    toward a rack corner (robot_3 wedged on rack 1 NE corner, userrestart9).

    `crossing` (from GridMap.in_crossing_zone at the TARGET cell) softens
    the otherwise planner-forbidden wrong-way term; see LaneBand.step_cost."""
    hit = [b for b in bands if b.in_band(r1, c1)]
    if not hit:
        return 0.0
    if len(hit) == 1:
        return hit[0].step_cost(r0, c0, r1, c1, along_scale=along_scale,
                                lateral_scale=lateral_scale,
                                crossing=crossing)
    return sum(b.step_cost(r0, c0, r1, c1, lateral=False,
                           along_scale=along_scale, crossing=crossing)
               for b in hit)


class Zone:
    """A named shared resource with a capacity.

    Modelling an aisle as ONE capacity-1 resource, rather than as a series of
    individual cells, is what makes head-on deadlock inside that aisle
    structurally impossible: two robots can never each own half of it.
    """

    def __init__(self, zone_id: str, kind: str, cells: Set[Cell], capacity: int = 1,
                 rank: Optional[int] = None,
                 modes: Optional[Tuple[str, ...]] = None):
        self.zone_id = zone_id
        self.kind = kind              # intersection | single_lane | choke | corridor | spur
        self.cells = cells
        self.capacity = capacity
        # Traffic layer (core/traffic.py): a ranked zone is acquired at an
        # acquisition point in ascending (rank, zone_id) order. None = a
        # legacy zone gated in path order by the coordinator.
        self.rank = rank
        # Traffic modes in which the zone is required (None = every mode).
        self.modes = modes

    def contains(self, cell: Cell) -> bool:
        return cell in self.cells

    def __repr__(self):
        return f"<Zone {self.zone_id} {self.kind} cap={self.capacity} n={len(self.cells)}>"


class TrafficConfig:
    """The unranked part of the yaml 'traffic:' section: refuges, wait bays,
    retreat cells and the traffic mode. Empty when the section is absent."""

    def __init__(self, mode: str = "SPINE",
                 refuges: Optional[Dict[str, Tuple[Set[Cell], float]]] = None,
                 wait_bays: Optional[Dict[str, Cell]] = None,
                 retreat_cells: Optional[List[Cell]] = None,
                 road_lanes: Optional[List[dict]] = None,
                 junctions: Optional[List[dict]] = None,
                 reservations: bool = True,
                 turnarounds: Optional[List[dict]] = None):
        self.mode = mode
        # ROAD MODEL kill-switch (user supersede, webots_final4): with
        # reservations False NO zone/segment/junction mutex is ever
        # requested - robots move freely; conflicts are handled by
        # keep-right lanes, right-of-way + don't-block-the-box at the
        # junction lines, and the safety envelope. True keeps the legacy
        # reservation machinery (old fixtures, unit tests).
        self.reservations = bool(reservations)
        self.refuges = refuges or {}             # id -> (cells, cost per cell)
        self.wait_bays = wait_bays or {}         # id -> cell
        self.retreat_cells = list(retreat_cells or [])
        # ROAD MODEL geometry published for the webapp (yaml traffic:
        # road_lanes / junctions; world metres). The core's own lane logic
        # reads the LaneBand; test_lanes pins the two to the same numbers.
        self.road_lanes = list(road_lanes or [])
        self.junctions = list(junctions or [])
        # TURNAROUND zones (strict lanes): the designated U-turn boxes at
        # the aisle dead ends, raw yaml schema [{id, center:[x,y],
        # half_size:[w,h]}] in world metres (the webapp renders these).
        # GridMap.from_yaml also rasterises them into turnaround_rects for
        # in_crossing_zone().
        self.turnarounds = [dict(t) for t in (turnarounds or [])]
        # cell -> (refuge id, cost), for the A* neighbour loop.
        self.refuge_cost: Dict[Cell, Tuple[str, float]] = {}
        for rid, (cells, cost) in self.refuges.items():
            for c in cells:
                self.refuge_cost[c] = (rid, float(cost))

    def refuge_of(self, cell: Cell) -> Optional[str]:
        hit = self.refuge_cost.get(cell)
        return hit[0] if hit else None


def _rect_cells(z: dict) -> Set[Cell]:
    if "cells" in z:
        return {(int(r), int(c)) for r, c in z["cells"]}
    if "rect" in z:
        r0, c0, r1, c1 = z["rect"]
        return {(r, c)
                for r in range(int(r0), int(r1) + 1)
                for c in range(int(c0), int(c1) + 1)}
    return set()


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
                 stations: Optional[Dict[str, Cell]] = None,
                 traffic: Optional[TrafficConfig] = None):
        self.static = [[1 if ch == '#' else 0 for ch in row] for row in occupancy]
        self.height = len(self.static)
        self.width = len(self.static[0]) if self.height else 0
        self.resolution = resolution
        self.origin = origin
        self.zones = zones
        self.stations = stations or {}
        self.traffic = traffic or TrafficConfig()
        # Lane band + pull-over pockets (yaml traffic.lanes / traffic.pockets;
        # set by from_yaml, None/[] on hand-built grids and legacy yamls).
        self.lanes: Optional[LaneBand] = None
        # All lane bands (road model keep-right everywhere); see from_yaml.
        self.lane_bands: List[LaneBand] = []
        self.pockets: List[Cell] = []
        # Turnaround boxes as inclusive cell rects (r0, c0, r1, c1), parallel
        # to traffic.turnarounds; set by from_yaml.
        self.turnaround_rects: List[Tuple[int, int, int, int]] = []
        self._crossing_cells: Optional[Set[Cell]] = None
        self.dynamic: Dict[Cell, DynamicCell] = {}
        self._clearance: Optional[List[List[float]]] = None
        self._clearance_cost: Optional[List[List[float]]] = None
        # Directional forward-clearance maps (step dir -> 2D metres array).
        self._fwd_clear: Dict[Tuple[int, int], List[List[float]]] = {}
        # zone_id -> (n_cells, r0, c0, r1, c1), for zones_within.
        self._zone_bbox: Dict[str, Tuple[int, int, int, int, int]] = {}

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
            rank = z.get("rank")
            modes = z.get("modes")
            zones[zid] = Zone(zid, z.get("type", "intersection"), _rect_cells(z),
                              int(z.get("capacity", 1)),
                              rank=None if rank is None else int(rank),
                              modes=None if modes is None
                              else tuple(str(m) for m in modes))

        stations = {sid: (int(s["cell"][0]), int(s["cell"][1]))
                    for sid, s in (cfg.get("stations") or {}).items()}

        t = cfg.get("traffic") or {}
        traffic = TrafficConfig(
            mode=str(t.get("mode", "SPINE")),
            refuges={rid: (_rect_cells(r), float(r.get("cost", 1000.0)))
                     for rid, r in (t.get("refuges") or {}).items()},
            wait_bays={bid: (int(c[0]), int(c[1]))
                       for bid, c in (t.get("wait_bays") or {}).items()},
            retreat_cells=[(int(c[0]), int(c[1]))
                           for c in (t.get("retreat_cells") or [])],
            road_lanes=[dict(l) for l in (t.get("road_lanes") or [])],
            junctions=[dict(j) for j in (t.get("junctions") or [])],
            reservations=bool(t.get("reservations", True)),
            turnarounds=[dict(ta) for ta in (t.get("turnarounds") or [])])

        grid = cls(occ, float(meta["resolution"]),
                   tuple(meta.get("origin", [0.0, 0.0])), zones, stations,
                   traffic)
        lanes = t.get("lanes") or {}
        # Every yaml lane band (spine, its junction extension, the aisles).
        # grid.lanes stays the SPINE band: the LANES/SPINE mode gate and
        # its tests are about the spine only.
        for name, spec in lanes.items():
            r0, c0, r1, c1 = (int(v) for v in spec["rect"])
            band = LaneBand(rect=(r0, c0, r1, c1),
                            axis=str(spec.get("axis", "v")),
                            centre_pos=float(spec["centre_pos"]),
                            centre_neg=float(spec["centre_neg"]),
                            name=str(name))
            grid.lane_bands.append(band)
            if name == "spine":
                grid.lanes = band
        grid.pockets = [(int(r), int(c)) for r, c in (t.get("pockets") or [])]
        # Turnaround boxes: world-metre schema -> inclusive cell rects over
        # the cells whose CENTRES lie inside the box.
        res, (ox, oy) = grid.resolution, grid.origin
        for ta in traffic.turnarounds:
            cx, cy = (float(v) for v in ta["center"])
            hx, hy = (float(v) for v in ta["half_size"])
            r0 = math.ceil((cy - hy - oy) / res - 0.5)
            r1 = math.floor((cy + hy - oy) / res - 0.5)
            c0 = math.ceil((cx - hx - ox) / res - 0.5)
            c1 = math.floor((cx + hx - ox) / res - 0.5)
            if r0 <= r1 and c0 <= c1:
                grid.turnaround_rects.append((int(r0), int(c0),
                                              int(r1), int(c1)))
        return grid

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

    # -------------------------------------------------- strict lanes (API)
    def lane_band_at(self, row: int, col: int
                     ) -> Optional[Tuple[str, str]]:
        """(lane_id, direction) of the directed lane the cell belongs to.

        lane_id is the yaml band key ('spine', 'spine_n', 'aisle_n',
        'aisle_a', 'aisle_b'); direction is the keep-right travel direction
        of the band HALF the cell sits on: 'NB'/'SB' for a vertical band,
        'EB'/'WB' for a horizontal one (NB=+y, EB=+x). Returns None off
        every band AND inside a junction box (two bands overlap there, so
        the lane is turn-dependent - in_crossing_zone() is True instead).
        A cell exactly on the divider belongs to the nearer lane centre
        (ties -> the positive-travel lane)."""
        hit = [b for b in self.lane_bands if b.in_band(row, col)]
        if len(hit) != 1:
            return None
        band = hit[0]
        pos = float(col if band.axis == "v" else row)
        d = pos - band.divider
        pos_dir, neg_dir = ("NB", "SB") if band.axis == "v" else ("EB", "WB")
        if d * (band.centre_pos - band.divider) > 0:
            direction = pos_dir
        elif d * (band.centre_neg - band.divider) > 0:
            direction = neg_dir
        else:                   # exactly on an integer divider row/col
            direction = (pos_dir if abs(pos - band.centre_pos)
                         <= abs(pos - band.centre_neg) else neg_dir)
        return (band.name or f"band_{self.lane_bands.index(band)}",
                direction)

    def crossing_cells(self) -> Set[Cell]:
        """Every cell where a lane crossing / U-turn is legal to PLAN:
        junction boxes (cells covered by two or more lane bands) plus the
        yaml traffic.turnarounds boxes. Cached; static geometry."""
        if self._crossing_cells is not None:
            return self._crossing_cells
        cells: Set[Cell] = set()
        for i, a in enumerate(self.lane_bands):
            for b in self.lane_bands[i + 1:]:
                r0 = max(a.rect[0], b.rect[0])
                c0 = max(a.rect[1], b.rect[1])
                r1 = min(a.rect[2], b.rect[2])
                c1 = min(a.rect[3], b.rect[3])
                cells.update((r, c) for r in range(r0, r1 + 1)
                             for c in range(c0, c1 + 1))
        for (r0, c0, r1, c1) in self.turnaround_rects:
            cells.update((r, c) for r in range(r0, r1 + 1)
                         for c in range(c0, c1 + 1))
        self._crossing_cells = cells
        return cells

    def in_crossing_zone(self, row: int, col: int) -> bool:
        """True inside any junction box (two lane bands overlap) or any
        turnaround box (yaml traffic.turnarounds). There - and ONLY there -
        the planner's wrong-way lane cost is the soft LANE_WRONG_SIDE
        instead of the forbidden LANE_WRONG_SIDE_HARD; the coordinator's
        U-turn gate and the trajectory audit use the same predicate."""
        return (row, col) in self.crossing_cells()

    # ------------------------------------------------------------ clearance
    def clearance_m(self) -> List[List[float]]:
        """Distance (m) from each free cell centre to the nearest static
        wall/rack face; 0 on obstacle cells. Computed once and cached - the
        static map never changes after load, and dynamic obstacles are
        deliberately excluded so this stays a one-off cost.

        Two-pass 8-neighbour chamfer transform: exact along the axis-aligned
        aisles, within ~8% near rack corners, and pure Python in ~10 ms.
        """
        if self._clearance is not None:
            return self._clearance
        h, w = self.height, self.width
        inf = float("inf")
        # Padded by one obstacle ring so off-grid counts as a wall.
        d = [[0.0] * (w + 2) for _ in range(h + 2)]
        for r in range(h):
            row, src = d[r + 1], self.static[r]
            for c in range(w):
                if not src[c]:
                    row[c + 1] = inf
        for r in range(1, h + 1):
            up, row = d[r - 1], d[r]
            for c in range(1, w + 1):
                v = row[c]
                if v:
                    v = min(v, row[c - 1] + 1.0, up[c] + 1.0,
                            up[c - 1] + _SQRT2, up[c + 1] + _SQRT2)
                    row[c] = v
        for r in range(h, 0, -1):
            dn, row = d[r + 1], d[r]
            for c in range(w, 0, -1):
                v = row[c]
                if v:
                    v = min(v, row[c + 1] + 1.0, dn[c] + 1.0,
                            dn[c + 1] + _SQRT2, dn[c - 1] + _SQRT2)
                    row[c] = v
        res = self.resolution
        # Chamfer distance is centre-to-obstacle-centre; subtract half a cell
        # to get centre-to-face.
        self._clearance = [[max(0.0, d[r + 1][c + 1] - 0.5) * res
                            for c in range(w)] for r in range(h)]
        return self._clearance

    def clearance_cost(self) -> List[List[float]]:
        """Cached per-cell A* surcharge that pulls paths onto aisle centres.

        The cost is relative to the LOCAL corridor centre (the largest
        clearance within CLEARANCE_CAP_M, capped at that value), not to
        absolute clearance. So the centre line of every aisle - the 1.6 m
        aisles and 1.6 m centre spine of the R7 layout - costs 0, and route choice
        between aisles stays length-based; only lateral offset is penalised.
        On open floor it keeps paths >= CLEARANCE_CAP_M from walls and racks.
        """
        if self._clearance_cost is not None:
            return self._clearance_cost
        clr = self.clearance_m()
        h, w = self.height, self.width
        k = max(1, int(round(CLEARANCE_CAP_M / self.resolution)))
        # Separable square max filter: O(cells * k) instead of O(cells * k^2).
        rowmax = [[max(row[max(0, c - k):c + k + 1]) for c in range(w)]
                  for row in clr]
        cost = [[0.0] * w for _ in range(h)]
        for c in range(w):
            col = [rowmax[r][c] for r in range(h)]
            for r in range(h):
                if self.static[r][c]:
                    continue
                v = clr[r][c]
                if v < CLEARANCE_LETHAL_M:
                    cost[r][c] = CLEARANCE_LETHAL_COST
                    continue
                ref = min(CLEARANCE_CAP_M, max(col[max(0, r - k):r + k + 1]))
                if v < ref:
                    cost[r][c] = CLEARANCE_WEIGHT * (1.0 - v / ref)
        self._clearance_cost = cost
        return cost

    # Forward-clearance cap: rays longer than this are "open" (the planner's
    # forward-hazard term only cares about the first ~0.65 m).
    FWD_CLEAR_CAP_M = 2.0

    def forward_clearance(self, dr: int, dc: int) -> List[List[float]]:
        """Metres of static-free travel from each cell's CENTRE along the
        step direction (dr, dc) before the first wall/rack cell centre,
        capped at FWD_CLEAR_CAP_M; 0.0 on blocked cells, and off-grid counts
        as a wall. Cached per direction (static geometry).

        This is the planner's 'don't drive AT a nearby wall' input (corner &
        rotation fixes, 2026-10-04): Nav2's collision monitor stops any
        forward command whose 0.45 m StopZone reaches an obstacle, so a path
        step whose heading points at a wall closer than StopZone + nose
        (~0.65 m) WILL strand the robot there - the measured robot_1 wedge at
        the J_N wall lane. astar charges such steps so routes turn away from
        walls before the controller ever faces them."""
        key = (int(dr), int(dc))
        hit = self._fwd_clear.get(key)
        if hit is not None:
            return hit
        dr, dc = key
        step = math.hypot(dr, dc) * self.resolution
        cap = self.FWD_CLEAR_CAP_M
        h, w = self.height, self.width
        out = [[0.0] * w for _ in range(h)]
        # DP against the ray direction: d(r,c) = step + d(r+dr, c+dc), with
        # blocked/off-grid cells at 0. Iterate rows/cols so the downstream
        # cell is already final.
        rows = range(h - 1, -1, -1) if dr > 0 else range(h)
        cols = range(w - 1, -1, -1) if dc > 0 else range(w)
        for r in rows:
            srow, orow = self.static[r], out[r]
            nrow = (out[r + dr] if 0 <= r + dr < h else None)
            for c in cols:
                if srow[c]:
                    continue                      # blocked: stays 0.0
                nr_c = c + dc
                if nrow is None or not (0 <= nr_c < w):
                    orow[c] = step                # the next cell is a wall
                else:
                    orow[c] = min(cap, step + nrow[nr_c])
        self._fwd_clear[key] = out
        return out

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
        """Zones the path enters, deduplicated: ranked traffic zones first in
        acquisition (rank, zone_id) order, then unranked zones in first-entry
        order. Intent.zones is built from this, so peers see the rank order."""
        out: List[str] = []
        for cell in cells:
            for zid, z in self.zones.items():
                if z.capacity >= 1 and z.contains(cell) and zid not in out:
                    out.append(zid)
        ranked = sorted((z for z in out if self.zones[z].rank is not None),
                        key=lambda z: (self.zones[z].rank, z))
        return ranked + [z for z in out if self.zones[z].rank is None]

    def zone_of(self, cell: Cell) -> Optional[str]:
        for zid, z in self.zones.items():
            if z.contains(cell):
                return zid
        return None

    def zones_within(self, x: float, y: float, radius_m: float) -> Set[str]:
        """Zones with any cell centre within radius_m (+ half a cell) of the
        world point (x, y)."""
        reach = radius_m + 0.5 * self.resolution
        r2 = reach * reach
        out: Set[str] = set()
        pr, pc = self.world_to_cell(x, y)
        for zid, z in self.zones.items():
            if not z.cells:
                continue
            bb = self._zone_bbox.get(zid)
            if bb is None or bb[0] != len(z.cells):
                rows = [c[0] for c in z.cells]
                cols = [c[1] for c in z.cells]
                bb = (len(z.cells), min(rows), min(cols), max(rows), max(cols))
                self._zone_bbox[zid] = bb
            n, r0, c0, r1, c1 = bb
            # Nearest cell centre of the bounding box: exact for a filled
            # rectangle (every yaml rect zone), a cheap reject otherwise.
            cx, cy = self.cell_to_world((min(max(pr, r0), r1),
                                         min(max(pc, c0), c1)))
            if (cx - x) ** 2 + (cy - y) ** 2 > r2:
                continue
            if n == (r1 - r0 + 1) * (c1 - c0 + 1):
                out.add(zid)
                continue
            for cell in z.cells:
                cx, cy = self.cell_to_world(cell)
                if (cx - x) ** 2 + (cy - y) ** 2 <= r2:
                    out.add(zid)
                    break
        return out

    # -------------------------------------------------------------- updates
    def report_blocked(self, cells, reporter: str, confidence: float, expiry: float):
        for cell in cells:
            d = self.dynamic.get(cell)
            if d is None:
                d = DynamicCell(0.0, expiry)
                self.dynamic[cell] = d
            d.sources.add(reporter)
            # The caller supplies calibrated evidence. A local obstacle
            # detector only calls this after persistent hits, so its high
            # confidence must be actionable immediately. Remote observations
            # are attenuated in merge_remote_update before they arrive here.
            d.confidence = max(d.confidence, min(1.0, max(0.0, confidence)))
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
            distance_trust = 1.0 if d < 5.0 else 0.7
            # A single remote observation is intentionally softer than local
            # persistent sensing. A second independent reporter corroborates
            # it through the source set checked by blocked_cells().
            trust = confidence * distance_trust * 0.5
        elif blocked:
            trust = confidence * 0.5
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
