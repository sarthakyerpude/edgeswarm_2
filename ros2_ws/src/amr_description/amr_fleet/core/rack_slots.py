"""
Rack slot naming: "<side>R<section>", e.g. "1R6" = side 1, section 6.

Pure Python (no ROS). Geometry comes from the 'racks:' and 'aisle_width_m'
keys of config/warehouse_grid.yaml (written by the map generator); this module
only names and locates things on that geometry.

CONVENTION - the ONLY place it is defined. Change a table below to renumber.
  * Racks are numbered left to right, top to bottom (RACK_POSITION):
        1 = north-west   2 = north-east
        3 = mid-west     4 = mid-east
        5 = south-west   6 = south-east
    The yaml rack boxes are matched to these numbers by POSITION (row by y,
    column by x), so the yaml 'id' field cannot silently renumber anything.
  * Rack k's NORTH face is side 2k-1, its SOUTH face is side 2k (SIDES).
  * Each face is split into SECTIONS_PER_FACE equal sections, numbered
    1..20 from WEST to EAST (SECTION_DIRECTION).
  * Slot approach pose: the centre of the face segment, pushed perpendicular
    into the adjacent aisle by APPROACH_OFFSET_M = W/4 (the centre of the near
    half of the aisle, so a second robot can pass in the far half), yaw 0
    (facing east, along the aisle). Approach cell = world_to_cell(pose).
"""
import math
import re
from typing import Dict, List, NamedTuple, Optional, Tuple

Cell = Tuple[int, int]

# ------------------------------------------------------------ convention ----
# rack id -> (row from NORTH 0.., column from WEST 0..)
RACK_POSITION: Dict[int, Tuple[int, int]] = {
    1: (0, 0), 2: (0, 1),
    3: (1, 0), 4: (1, 1),
    5: (2, 0), 6: (2, 1),
}
# side number -> (rack id, face). Face 'N' = north (+y) face, 'S' = south.
SIDES: Dict[int, Tuple[int, str]] = {
    2 * k - 1 + j: (k, face)
    for k in RACK_POSITION for j, face in enumerate(("N", "S"))
}
SECTIONS_PER_FACE = 20
SECTION_DIRECTION = "W2E"        # section 1 at the west end (x_min)
APPROACH_YAW = 0.0               # facing east, along the aisle
APPROACH_FRACTION_OF_W = 0.25    # APPROACH_OFFSET_M = W / 4

_LABEL_RE = re.compile(r"^\s*(\d+)\s*[Rr]\s*(\d+)\s*$")


class Rack(NamedTuple):
    rack_id: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float


class Slot(NamedTuple):
    label: str
    side: int
    rack: int
    face: str                        # 'N' or 'S'
    section: int                     # 1..SECTIONS_PER_FACE, west -> east
    x0: float                        # face segment west end
    x1: float                        # face segment east end
    face_y: float                    # the face line
    centre: Tuple[float, float]      # face-segment centre (on the face)
    approach: Tuple[float, float, float]   # (x, y, yaw) in the aisle
    approach_cell: Cell


# ------------------------------------------------------- label helpers ------
def format_label(side: int, section: int) -> str:
    return f"{int(side)}R{int(section)}"


def parse_label(label: str) -> Tuple[int, int]:
    """'1R6' -> (1, 6). Raises ValueError on a malformed or out-of-range label."""
    m = _LABEL_RE.match(str(label))
    if not m:
        raise ValueError(f"bad rack slot label {label!r} (expected e.g. '1R6')")
    side, section = int(m.group(1)), int(m.group(2))
    if side not in SIDES:
        raise ValueError(f"label {label!r}: side {side} not in 1..{len(SIDES)}")
    if not 1 <= section <= SECTIONS_PER_FACE:
        raise ValueError(
            f"label {label!r}: section {section} not in 1..{SECTIONS_PER_FACE}")
    return side, section


def all_labels() -> List[str]:
    return [format_label(s, n) for s in sorted(SIDES)
            for n in range(1, SECTIONS_PER_FACE + 1)]


def side_of(rack_id: int, face: str) -> int:
    for side, (k, f) in SIDES.items():
        if k == rack_id and f == face.upper():
            return side
    raise ValueError(f"no side for rack {rack_id} face {face!r}")


# ----------------------------------------------------------- geometry -------
def _assign_positions(boxes: List[Rack]) -> Dict[int, Rack]:
    """Match yaml boxes to RACK_POSITION by geometry: rows by descending
    centre y, columns by ascending centre x."""
    n_rows = 1 + max(p[0] for p in RACK_POSITION.values())
    n_cols = 1 + max(p[1] for p in RACK_POSITION.values())
    if len(boxes) != n_rows * n_cols:
        raise ValueError(f"expected {n_rows * n_cols} racks, got {len(boxes)}")
    by_y = sorted(boxes, key=lambda b: -(b.y_min + b.y_max))
    out: Dict[int, Rack] = {}
    for row in range(n_rows):
        row_boxes = sorted(by_y[row * n_cols:(row + 1) * n_cols],
                           key=lambda b: b.x_min + b.x_max)
        for col, b in enumerate(row_boxes):
            rid = next(k for k, p in RACK_POSITION.items() if p == (row, col))
            out[rid] = Rack(rid, b.x_min, b.x_max, b.y_min, b.y_max)
    return out


class RackSlots:
    """All 240 slots for one geometry."""

    def __init__(self, racks: List[dict], aisle_width_m: float,
                 resolution: float = 0.1,
                 origin: Tuple[float, float] = (-6.0, -5.0)):
        boxes = [Rack(int(r.get("id", 0)), float(r["x_min"]), float(r["x_max"]),
                      float(r["y_min"]), float(r["y_max"])) for r in racks]
        for b in boxes:
            if not (b.x_min < b.x_max and b.y_min < b.y_max):
                raise ValueError(f"degenerate rack box {b}")
        self.yaml_ids = {b.rack_id: b for b in boxes}
        self.racks: Dict[int, Rack] = _assign_positions(boxes)
        self.aisle_width_m = float(aisle_width_m)
        self.approach_offset_m = APPROACH_FRACTION_OF_W * self.aisle_width_m
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self._slots: Dict[str, Slot] = {}
        for side in sorted(SIDES):
            for section in range(1, SECTIONS_PER_FACE + 1):
                s = self._build(side, section)
                self._slots[s.label] = s

    # ------------------------------------------------------------- loaders
    @classmethod
    def from_config(cls, cfg: dict) -> "RackSlots":
        if "racks" not in cfg or "aisle_width_m" not in cfg:
            raise KeyError("warehouse grid yaml has no 'racks:'/'aisle_width_m'")
        meta = cfg.get("meta") or {}
        return cls(cfg["racks"], cfg["aisle_width_m"],
                   float(meta.get("resolution", 0.1)),
                   tuple(meta.get("origin", [-6.0, -5.0])))

    @classmethod
    def from_yaml(cls, path: str) -> "RackSlots":
        import yaml
        with open(path, "r") as f:
            return cls.from_config(yaml.safe_load(f))

    # ---------------------------------------------------------- conversion
    def world_to_cell(self, x: float, y: float) -> Cell:
        # GridMap.world_to_cell (row -> y, col -> x) plus a 1e-6 cell nudge:
        # approach lines (face +- W/4) land exactly on cell boundaries
        # (e.g. y=3.8 -> 87.99999999 -> row 87); the nudge makes them
        # resolve to the cell they start, independent of float noise.
        return (int((y - self.origin[1]) / self.resolution + 1e-6),
                int((x - self.origin[0]) / self.resolution + 1e-6))

    def section_length(self, rack_id: int) -> float:
        r = self.racks[rack_id]
        return (r.x_max - r.x_min) / SECTIONS_PER_FACE

    def face_y(self, side: int) -> float:
        rack_id, face = SIDES[side]
        r = self.racks[rack_id]
        return r.y_max if face == "N" else r.y_min

    def approach_y(self, side: int) -> float:
        sign = 1.0 if SIDES[side][1] == "N" else -1.0
        return self.face_y(side) + sign * self.approach_offset_m

    def _build(self, side: int, section: int) -> Slot:
        rack_id, face = SIDES[side]
        r = self.racks[rack_id]
        L = self.section_length(rack_id)
        if SECTION_DIRECTION == "W2E":
            x0 = r.x_min + (section - 1) * L
        else:                                   # "E2W"
            x0 = r.x_max - section * L
        x1 = x0 + L
        fy = self.face_y(side)
        cx = 0.5 * (x0 + x1)
        ay = self.approach_y(side)
        return Slot(format_label(side, section), side, rack_id, face, section,
                    x0, x1, fy, (cx, fy), (cx, ay, APPROACH_YAW),
                    self.world_to_cell(cx, ay))

    # -------------------------------------------------------------- queries
    def slot(self, label: str) -> Slot:
        side, section = parse_label(label)
        return self._slots[format_label(side, section)]

    def slots(self) -> List[Slot]:
        return [self._slots[l] for l in all_labels()]

    def approach_cell(self, label: str) -> Cell:
        return self.slot(label).approach_cell

    def approach_pose(self, label: str) -> Tuple[float, float, float]:
        return self.slot(label).approach

    def label_at(self, x: float, y: float,
                 max_dist_m: Optional[float] = None) -> Optional[str]:
        """Nearest slot to a world point (by approach position)."""
        best, best_d = None, math.inf
        for s in self._slots.values():
            d = math.hypot(s.approach[0] - x, s.approach[1] - y)
            if d < best_d:
                best, best_d = s.label, d
        if max_dist_m is not None and best_d > max_dist_m:
            return None
        return best

    def side_summary(self) -> List[dict]:
        out = []
        for side in sorted(SIDES):
            rack_id, face = SIDES[side]
            r = self.racks[rack_id]
            out.append({"side": side, "rack": rack_id, "face": face,
                        "face_y": round(self.face_y(side), 4),
                        "x_min": round(r.x_min, 4), "x_max": round(r.x_max, 4),
                        "section_length_m": round(self.section_length(rack_id), 4),
                        "approach_y": round(self.approach_y(side), 4),
                        "approach_row": self.world_to_cell(
                            r.x_min, self.approach_y(side))[0]})
        return out
