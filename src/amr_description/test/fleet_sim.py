"""
Offline fleet simulator: N real FleetCoordinators on the real warehouse grid.

Validates traffic rules in seconds instead of 12-minute Webots runs. Pure
Python, no ROS: importable by pytest and runnable as a script

    python3 test/fleet_sim.py head_on 240
    python3 test/fleet_sim.py random 480 --seed 7
    python3 test/fleet_sim.py stress 480 --seed 7          # sih-57 case
    python3 test/fleet_sim.py random 480 --legacy --loc-noise 0.1 --faults link

WHAT IS REAL
  * core/coordinator.FleetCoordinator (and everything it owns: PeerRegistry,
    ZoneArbiter, ConflictDetector, DeadlockManager, safety envelope, A*), one
    instance per robot, each with its own GridMap loaded from
    config/warehouse_grid.yaml.
  * The node glue that decides what reaches the actuator is mirrored line by
    line from nodes/fleet_agent_node.py: _advance_task_lifecycle (0.25 m +
    settled arrival, 2 s dwell, deferred resume), _maybe_retreat /
    _pick_retreat_cell (the node's YIELD manoeuvre), _maybe_publish_intent
    (seq bumped only on plan change, 1 Hz keepalive, header stamp = now),
    publish_state (10 Hz, no intent on the wire), the dock policy
    (core/docking.DockPolicy) and the battery model.
  * Receivers convert intent windows exactly like conversions.intent_from_ros
    (Intent.to_receiver_clock(stamp, rx_time)) and feed
    registry.update_intent(rid, intent, seq, now=rx, stamp=stamp).
  * Task bids use core/tasks.TaskAuction.compute_bid.

WHAT IS MODELLED
  * Robot body: kinematic unicycle, ground truth (x, y, theta). Controller is
    a Regulated-Pure-Pursuit-like follower of coordinator.path (velocity-
    scaled lookahead 0.3-1.2 m, rotate-in-place above 0.785 rad, curvature
    and goal-approach regulation), at v_nominal. velocity_gate: GO/SLOW pass
    with speed_scale applied to v and w; STOP/YIELD/REROUTE give zero.
    Accel limits 2.0 m/s^2 and 3.2 rad/s^2, w limit 1.5 rad/s.
  * Optional localization error: belief = truth + Ornstein-Uhlenbeck bias
    (loc_noise_m, loc_noise_tau_s) + per-tick white jitter (loc_jitter_m,
    loc_heading_noise_rad). Robots and their followers see ONLY the belief;
    metrics use the truth (loc_error_mean_m / loc_error_max_m report the gap).
  * In-memory bus, fixed 0.1 s tick. Optional loss (state/intent; zone
    messages are RELIABLE so only zone_loss if you ask), latency + jitter,
    and per-robot clock offsets. robot_state has the 0.5 s DDS lifespan:
    a state older than that on arrival is discarded.
  * Faults (Scenario.faults, list of Fault(rid, t0, t1, kind, peer)):
      "heartbeat"  rid's robot_state is not delivered (to `peer` only, if
                   set): peers see it SUSPECT then DEAD while its reliable
                   coordination traffic still flows. Models the 10-16 s
                   DEAD links seen at load 66 (wiki/Plan-Fleet-WebApp.md).
      "link"       connection drop, both directions, every kind. State is
                   lost; reliable kinds (intent, zone request/grant) follow
                   DDS RELIABLE KEEP_LAST(10): held and replayed when the
                   link returns, last 10 per writer/reader/topic only
                   (link_replay=False drops them instead).
      "node"       rid's fleet node executor freezes: no tick, no
                   publishing; inbound messages queue (state keeps only the
                   newest, coord keeps the last 10) and are processed when
                   it resumes. velocity_gate zeroes motion once the last
                   permit is older than 0.5 s.
    random_faults() draws such windows; stress() is the sih-57 double-hold
    case (40 % loss + 10-16 s heartbeat drops).
  * No lidar: no dynamic map blockages, robots do not physically collide
    (contacts are measured, not prevented).

Ground truth is used ONLY for metrics; robots see only their belief pose and
messages.

REPRODUCIBILITY AND A/B
  * Runs are deterministic per seed within a process. Across processes, set
    PYTHONHASHSEED=0: core/ iterates some sets of zone ids.
  * grid_yaml=LEGACY_YAML runs the pre-increment-2 zones Webots measured.
  * FLEET_SIM_CORE_ROOT=<dir holding another amr_fleet/> runs the same
    scenarios against that copy of the code (e.g. a frozen baseline).

METRICS NOTES
  * replan_failed counts every "REPLAN FAILED" log line; a deadlock victim's
    blocker-avoiding reroute attempt logs one before each retreat.
  * deadlock recovery times come from DeadlockManager and are 0.0 whenever
    the victim's reroute/retreat calls mark_resolved on the same tick.
  * max_go_stall_s: permitted GO/SLOW yet not translating. Should be a few
    seconds (rotate-in-place); larger means a follower or permit problem.
    SLOW scales w too (as velocity_gate does), so a 180 deg turn at
    speed_scale 0.25 takes ~8 s.
  * peer_dead_s["a<-b"]: seconds robot a's registry held b DEAD.
    peer_failures_detected: PEER FAILURE transitions, all observers.
  * bus_fault_dropped: lost to faults, KEEP_LAST overflow or the state
    lifespan (state_expired); bus_replayed: reliable messages delivered
    late after a link drop. bus_dropped is random loss only.
  * arbiter_double_hold_ticks counts ticks in which two arbiters are HELD on
    one zone (protocol view); zone_/plan_zone_double_occupancy is the ground
    truth (two robot centres inside one capacity-1 rect).
"""
import argparse
import heapq
import json
import math
import os
import pathlib
import random
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

PKG = pathlib.Path(__file__).resolve().parents[1]
# A/B: FLEET_SIM_CORE_ROOT=<dir containing another amr_fleet/> runs the same
# scenarios against that copy of the code (e.g. a frozen pre-change baseline).
# Config yamls still come from this package unless a scenario names one.
_CODE_ROOT = os.environ.get("FLEET_SIM_CORE_ROOT") or str(PKG)
if _CODE_ROOT not in sys.path:
    sys.path.insert(0, _CODE_ROOT)

from amr_fleet.core.coordinator import FleetCoordinator          # noqa: E402
from amr_fleet.core.gridmap import GridMap                       # noqa: E402
from amr_fleet.core.models import (CHARGING, IDLE,                # noqa: E402
                                   YIELDING, Intent, Permit, Pose2D,
                                   RobotState, Task)
from amr_fleet.core.tasks import TaskAuction                     # noqa: E402

try:                                    # owned by another session; optional
    from amr_fleet.core import docking  # noqa: E402
except Exception:                       # pragma: no cover
    docking = None

try:                                    # hitbox builder's module; fallback
    from amr_fleet.core import hitbox as _hitbox  # noqa: E402
except Exception:                       # pragma: no cover
    _hitbox = None

try:
    from amr_fleet.core import priority as _priority  # noqa: E402
except Exception:                       # pragma: no cover
    _priority = None

GRID_YAML = PKG / "config" / "warehouse_grid.yaml"
# The config Webots ran when the 0-delivery symptom was measured (pre-inc-2
# zones inter_X1 / aisle_A1 / aisle_A2 / choke_CP1).
LEGACY_YAML = PKG / "test" / "fixtures" / "warehouse_grid_legacy.yaml"

DT = 0.1                    # tick, s
T0 = 1000.0                 # coordinator clock at sim t=0 (avoid t=0 edge cases)
CONTACT_M = 0.42            # ground-truth centre distance counted as contact
CHASSIS_HALF_WIDTH_M = 0.16

# Station cells (task_generator_node defaults == warehouse_grid.yaml stations)
L1, L2, L3 = (88, 25), (88, 94), (74, 25)     # rack slots 1R10, 3R11, 2R10
D1, D2, D3 = (22, 30), (22, 60), (22, 90)
PICKUPS = [L1, L2, L3]
DROPOFFS = [D1, D2, D3]
SPAWNS = [("robot_1", -3.0, -4.0), ("robot_2", 0.0, -4.0), ("robot_3", 3.0, -4.0)]

# Plan-P2P-Coordination increment-2 zone rects (r0, c0, r1, c1), re-cut for
# the R7 equal-aisle layout (1.6 m aisles/spine; == warehouse_grid.yaml zones,
# pinned by test_layout_consistency.py). Pre-R7 values were SP_AW (67,3,81,49)
# SP_NW (88,1,98,49) SP_NE (88,70,98,118) J_N (88,50,98,69) J_AW (67,50,81,59)
# SPINE (35,50,87,69), rack rows 46-51/61-66/82-87; --legacy runs on the old
# map are scored against the R7 rects below.
PLAN_ZONES = {
    "SP_AW": (62, 1, 77, 51), "SP_NW": (84, 1, 98, 51),
    "SP_NE": (84, 68, 98, 118), "J_N": (84, 52, 98, 67),
    "J_AW": (62, 52, 77, 59),
    # R10: the whole-corridor SPINE mutex is re-cut into 3 junction-bounded
    # segments (ranks SPINE_S 1 < SPINE_M 2 < SPINE_N 3 < J_N 4 < J_AW 5).
    "SPINE_S": (28, 52, 39, 67), "SPINE_M": (40, 52, 61, 67),
    "SPINE_N": (62, 52, 83, 67),
}
# The full spine band (union of the three segments): head-on/lane metrics.
SPINE_RECT = (28, 52, 83, 67)
RACK_ROWS = [(34, 39), (56, 61), (78, 83)]   # R7 rack rows 3 / 2 / 1

# Controller (nav2_params.yaml RPP + velocity_gate.yaml)
LOOKAHEAD_TIME_S = 1.5
LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M = 0.3, 1.2
ROTATE_TO_HEADING_RAD = 0.785
ROTATE_W = 1.5
REGULATED_MIN_RADIUS_M = 1.5   # = nav2_params regulated_linear_scaling_min_radius (1.0 m/s cruise)
REGULATED_MIN_SPEED = 0.10
APPROACH_DIST_M = 0.6
APPROACH_MIN_SPEED = 0.05
GOAL_TOL_M = 0.10
CHORD_CLEARANCE_M = 0.2
ROTATE_DONE_RAD = 0.3       # rotate-in-place hysteresis exit
PRUNE_ARC_M = 0.6           # forward search for the closest path point
REACH_M = 0.12              # a path point this close counts as passed
GATE_MAX_V, GATE_MAX_W = 0.6, 1.5   # = velocity_gate max_linear_x (1.5x)
MAX_ACCEL, MAX_ANG_ACCEL = 2.0, 3.2
PERMIT_TIMEOUT_S = 0.5      # velocity_gate.yaml permit_timeout_s

# qos_profiles.py
STATE_LIFESPAN_S = 0.5      # STATE_QOS lifespan
COORD_DEPTH = 10            # COORD_QOS KEEP_LAST depth (intent, zone req/grant)
TASK_KINDS = ("task_announce", "task_bid", "task_award", "task_complete",
              "task_release", "task_cancel")
RELIABLE_KINDS = ("intent", "zone_request", "zone_grant") + TASK_KINDS

# Ground-truth hitbox (wheel-inclusive, == core/hitbox contract constants).
HB_HALF_L = 0.20
HB_HALF_W = 0.205
HB_R_CIRC = 0.286
NEAR_MISS_M = 0.05          # true rect gap below this counts as a near miss
LANE_BAND = (25, 52, 83, 67)  # the R7 spine lane band incl. the south throat

# R5 deadline budgets by priority (task_generator_node deadline_budget_s;
# == priority.BUDGET_S contract values).
_TASK_BUDGET_S = {0: 225.0, 1: 188.0, 2: 150.0, 3: 113.0}


def _local_rect_gap(a, b):
    """SAT gap between two oriented rects (x, y, th), 0.0 on overlap.
    Fallback for core/hitbox.rect_gap until the hitbox builder lands."""
    def corners(p):
        x, y, th = p
        c, s = math.cos(th), math.sin(th)
        return [(x + c * dx - s * dy, y + s * dx + c * dy)
                for dx, dy in ((HB_HALF_L, HB_HALF_W), (HB_HALF_L, -HB_HALF_W),
                               (-HB_HALF_L, -HB_HALF_W), (-HB_HALF_L, HB_HALF_W))]
    ca, cb = corners(a), corners(b)
    best = 0.0
    overlap = True
    for th in (a[2], a[2] + math.pi / 2, b[2], b[2] + math.pi / 2):
        ux, uy = math.cos(th), math.sin(th)
        pa = [x * ux + y * uy for x, y in ca]
        pb = [x * ux + y * uy for x, y in cb]
        gap = max(min(pb) - max(pa), min(pa) - max(pb))
        if gap > 0.0:
            overlap = False
            best = max(best, gap)
    if overlap:
        return 0.0
    # The max separating-axis gap is a LOWER bound on the true gap; for the
    # acceptance metric (gap <= 0 / gap < 0.05) the axis gap is what the
    # safety contract uses, so report it directly.
    return best


def rect_gap(a, b):
    if _hitbox is not None and hasattr(_hitbox, "rect_gap"):
        return _hitbox.rect_gap(a, b)
    return _local_rect_gap(a, b)


def _class_of_score(score: float) -> int:
    """Priority class 0..4 from a broadcast L (legacy [0,1] -> 0)."""
    fn = getattr(_priority, "class_of_score", None) if _priority else None
    if fn is not None:
        try:
            return int(fn(float(score)))
        except Exception:
            pass
    s = float(score)
    return max(0, min(4, int(s // 1000.0))) if s >= 1.0 else 0

# fleet_params.yaml
PEER_SUSPECT_S, PEER_DEAD_S = 2.0, 5.0
INTENT_KEEPALIVE_S = 1.0
ROUTE_FAILURE_TIMEOUT_S = 5.0
OBSTACLE_STATIC_MARGIN_CELLS = 4
BATTERY_PER_METRE = 0.35
CHARGE_RATE = 1.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _in_rect(cell, rect) -> bool:
    r0, c0, r1, c1 = rect
    return r0 <= cell[0] <= r1 and c0 <= cell[1] <= c1


def in_spine_gap(cell) -> bool:
    """A cell in the 1.6 m spine between two rack faces (rows of a rack row)."""
    return (SPINE_RECT[1] <= cell[1] <= SPINE_RECT[3]
            and any(a <= cell[0] <= b for a, b in RACK_ROWS))


# ===================================================== lane audit (LANES) ==
# Trajectory audit for the strict directional lanes (increment: LANES
# hardening). Per tick, each robot's TRUE pose cell is mapped to its lane
# band half; a VIOLATION is travelling forward along a band against that
# half's direction while coord.uturn_active is False and the cell is not a
# junction / turnaround crossing cell. Prefers the gridmap API
# (lane_band_at / in_crossing_zone) once the LANES planner lands it; until
# then it reads the same traffic.lanes bands the yaml defines.
LANE_AUDIT_V_MIN = 0.05     # m/s forward speed below which no direction
LANE_AUDIT_ALONG_MIN = 0.3  # |cos| of heading vs band axis to count travel
LANE_AUDIT_SAMPLES = 40     # first N violation events kept with detail


class LaneAudit:
    def __init__(self, grid, grid_yaml: Optional[str] = None):
        self.grid = grid
        self.bands = list(getattr(grid, "lane_bands", []) or [])
        if not self.bands and getattr(grid, "lanes", None) is not None:
            self.bands = [grid.lanes]
        self._api_cross = getattr(grid, "in_crossing_zone", None)
        self._api_band = getattr(grid, "lane_band_at", None)
        self._cross_cache: Dict[tuple, bool] = {}
        # junction boxes: intersection-type zones (J_N / J_AW) and any cell
        # inside 2+ overlapping lane bands (an aisle crossing the spine).
        self._junction_cells = set()
        for z in getattr(grid, "zones", {}).values():
            if getattr(z, "kind", "") == "intersection":
                self._junction_cells |= set(z.cells)
        # traffic.turnarounds (LANES planner): read from the grid if it
        # exposes them, else from the yaml; accept rects, cells or dicts.
        self._turnaround_cells = self._parse_turnarounds(
            getattr(grid, "turnarounds", None))
        if not self._turnaround_cells and grid_yaml:
            try:
                import yaml as _yaml
                with open(grid_yaml) as f:
                    doc = _yaml.safe_load(f) or {}
                self._turnaround_cells = self._parse_turnarounds(
                    (doc.get("traffic") or {}).get("turnarounds"))
            except Exception:
                pass

    def _world_rect_cells(self, cx, cy, hx, hy) -> set:
        """Cells of a world-metre box {center, half_size} (yaml turnaround
        schema, same as traffic.junctions)."""
        try:
            r0, c0 = self.grid.world_to_cell(cx - hx + 1e-6, cy - hy + 1e-6)
            r1, c1 = self.grid.world_to_cell(cx + hx - 1e-6, cy + hy - 1e-6)
        except Exception:                        # map constants fallback
            c0, c1 = int((cx - hx + 6.0) / 0.1), int((cx + hx + 6.0) / 0.1 - 1e-9)
            r0, r1 = int((cy - hy + 5.0) / 0.1), int((cy + hy + 5.0) / 0.1 - 1e-9)
        return {(r, c) for r in range(min(r0, r1), max(r0, r1) + 1)
                for c in range(min(c0, c1), max(c0, c1) + 1)}

    def _parse_turnarounds(self, spec) -> set:
        cells = set()
        if not spec:
            return cells
        items = list(spec.values()) if isinstance(spec, dict) else list(spec)
        for it in items:
            if isinstance(it, dict):
                if "center" in it and "half_size" in it:
                    (cx, cy), (hx, hy) = it["center"], it["half_size"]
                    cells |= self._world_rect_cells(float(cx), float(cy),
                                                    float(hx), float(hy))
                    continue
                it = it.get("rect") or it.get("cells") or it.get("cell")
            if it is None:
                continue
            seq = list(it)
            if len(seq) == 4 and all(isinstance(v, (int, float)) for v in seq):
                r0, c0, r1, c1 = (int(v) for v in seq)
                for r in range(min(r0, r1), max(r0, r1) + 1):
                    for c in range(min(c0, c1), max(c0, c1) + 1):
                        cells.add((r, c))
            elif len(seq) == 2 and all(isinstance(v, (int, float)) for v in seq):
                cells.add((int(seq[0]), int(seq[1])))
            else:                       # list of cells
                for sub in seq:
                    try:
                        r, c = sub
                        cells.add((int(r), int(c)))
                    except Exception:
                        pass
        return cells

    def in_crossing(self, cell) -> bool:
        """Junction box or turnaround: legal for either direction."""
        hit = self._cross_cache.get(cell)
        if hit is not None:
            return hit
        out = None
        if self._api_cross is not None:
            try:
                out = bool(self._api_cross(cell))
            except TypeError:
                try:
                    out = bool(self._api_cross(cell[0], cell[1]))
                except Exception:
                    out = None
            except Exception:
                out = None
        fallback = (cell in self._junction_cells
                    or cell in self._turnaround_cells
                    or sum(b.in_band(cell[0], cell[1])
                           for b in self.bands) >= 2)
        out = fallback if out is None else (out or cell in self._turnaround_cells)
        self._cross_cache[cell] = out
        return out

    def half_at(self, cell):
        """Identity of the directed lane half under `cell`: (lane, dir) or
        None off-road / inside a crossing zone. Used to pin a reverse
        manoeuvre (coordinator.reverse_active) to the half it started in."""
        r, c = cell
        if self.in_crossing(cell):
            return None
        if self._api_band is not None:
            try:
                return self._api_band(r, c)
            except TypeError:
                try:
                    return self._api_band((r, c))
                except Exception:
                    pass
            except Exception:
                pass
        for b in self.bands:
            if not b.in_band(r, c):
                continue
            pos = float(c if b.axis == "v" else r)
            side = (pos - b.divider) * (b.centre_pos - b.divider) > 0
            pos_dir, neg_dir = (("NB", "SB") if b.axis == "v"
                                else ("EB", "WB"))
            return (b.rect, pos_dir if side else neg_dir)
        return None

    def violation(self, cell, v: float, th: float) -> bool:
        """True pose (cell, forward speed v, heading th) fights a band's
        direction. Reversing (v < 0, the own-lane retreat) and lateral
        motion (crossing/merging) are never violations; crossing cells and
        a robot outside every band are skipped by the caller or here."""
        if v < LANE_AUDIT_V_MIN:
            return False                       # stationary or reversing
        r, c = cell
        hit = [b for b in self.bands if b.in_band(r, c)]
        if not hit or self.in_crossing(cell):
            return False
        ux, uy = math.cos(th), math.sin(th)
        if self._api_band is not None:         # gridmap.lane_band_at (LANES)
            try:
                res = self._api_band(r, c)
            except TypeError:
                try:
                    res = self._api_band((r, c))
                except Exception:
                    res = ()
            except Exception:
                res = ()                       # () = API unusable, fall back
            if res != ():
                if not res:
                    return False               # junction box / off-band
                _, d = res
                along, across = (uy, ux) if d in ("NB", "SB") else (ux, uy)
                if abs(along) < LANE_AUDIT_ALONG_MIN or \
                        abs(along) < abs(across):
                    return False               # lateral: merging/crossing
                return (along > 0) != (d in ("NB", "EB"))
        for b in hit:
            along, across = (uy, ux) if b.axis == "v" else (ux, uy)
            if abs(along) < LANE_AUDIT_ALONG_MIN or abs(along) < abs(across):
                continue                       # lateral w.r.t. this band
            pos = float(c if b.axis == "v" else r)
            centre = b.centre_pos if along > 0 else b.centre_neg
            if (pos - b.divider) * (centre - b.divider) < 0:
                return True                    # wrong side for my direction
        return False


# =============================================================== scenario ==
@dataclass
class RobotSpec:
    rid: str
    x: float
    y: float
    theta: float = math.pi / 2
    # Scripted tasks: (sim time to award, Task). Empty -> stream / idle.
    tasks: List[Tuple[float, Task]] = field(default_factory=list)
    dock: bool = True                     # return_home_enabled (home = spawn)
    battery_pct: float = 100.0


@dataclass
class StreamSpec:
    """task_generator_node: one task every interval_s, never two live tasks
    on one pickup cell, uniform pickup/dropoff, weighted priority."""
    interval_s: float = 15.0
    num_tasks: int = 20
    pickups: List[tuple] = field(default_factory=lambda: list(PICKUPS))
    dropoffs: List[tuple] = field(default_factory=lambda: list(DROPOFFS))
    priority_weights: List[float] = field(
        default_factory=lambda: [0.5, 0.3, 0.15, 0.05])
    bid_window_s: float = 0.5
    no_bid_retry_s: float = 5.0


FAULT_KINDS = ("heartbeat", "link", "node")


@dataclass
class Fault:
    """A window [t0, t1) of sim time in which robot `rid` misbehaves.

    kind: "heartbeat" (its state is not heard), "link" (connection drop,
    both directions) or "node" (its fleet node executor is frozen). peer:
    restrict a heartbeat/link fault to the rid<->peer link only.
    """
    rid: str
    t0: float
    t1: float
    kind: str = "heartbeat"
    peer: Optional[str] = None

    def __post_init__(self):
        if self.kind not in FAULT_KINDS:
            raise ValueError(f"fault kind {self.kind!r} not in {FAULT_KINDS}")

    def active(self, t: float) -> bool:
        return self.t0 <= t < self.t1

    def covers(self, src: str, dst: str) -> bool:
        if self.kind == "heartbeat":            # only what rid SENDS
            return src == self.rid and self.peer in (None, dst)
        ends = {src, dst}
        return self.rid in ends and (self.peer is None or self.peer in ends)


def random_faults(seed: int, rids, horizon_s: float, kind: str = "heartbeat",
                  dur_s=(10.0, 16.0), gap_s=(30.0, 90.0),
                  first_s=(20.0, 60.0)) -> List[Fault]:
    """Independent fault windows per robot: first one after U(first_s), each
    lasting U(dur_s), separated by U(gap_s). Deterministic per seed."""
    rng = random.Random(seed * 7919 + 17)
    out = []
    for rid in rids:
        t = rng.uniform(*first_s)
        while t < horizon_s:
            d = rng.uniform(*dur_s)
            out.append(Fault(rid, round(t, 1), round(min(horizon_s, t + d), 1),
                             kind))
            t += d + rng.uniform(*gap_s)
    return out


@dataclass
class Scenario:
    name: str
    robots: List[RobotSpec]
    stream: Optional[StreamSpec] = None
    # Tasks ANNOUNCED on the fleet bus at (t, Task): they go through the real
    # distributed auction (unlike RobotSpec.tasks, which are direct awards).
    announced: List[Tuple[float, Task]] = field(default_factory=list)
    # R6 webapp aborts: (t, task_id) published on the task_cancel topic.
    cancels: List[Tuple[float, str]] = field(default_factory=list)
    # True (default): every robot runs its own TaskAuction over the lossy
    # bus, exactly like fleet_agent_node (required to test R4(4)). False
    # restores the old centralised resolver for A/B comparison.
    distributed_auction: bool = True
    # Broadcast loc_sigma_lat override (the LANES gate input); None -> the
    # localization noise knob value, as before.
    force_sigma_lat: Optional[float] = None
    seed: int = 0
    v_nominal: float = 0.6      # = fleet_params v_nominal (1.5x the original 0.4)
    dwell_s: float = 2.0
    mode: str = "proposed"
    grid_yaml: str = str(GRID_YAML)       # LEGACY_YAML = the Webots baseline
    # network knobs
    state_loss: float = 0.0
    intent_loss: float = 0.0
    zone_loss: float = 0.0
    # Loss on the task topics (announce/bid/award/...). DDS runs them
    # RELIABLE, so keep 0 for realism; raise it only to stress R4(4).
    task_loss: float = 0.0
    latency_s: float = 0.0
    jitter_s: float = 0.0
    clock_offsets: Dict[str, float] = field(default_factory=dict)
    # localization knob: 1-sigma of an OU position bias (m), tau in s
    loc_noise_m: float = 0.0
    loc_noise_tau_s: float = 5.0
    loc_jitter_m: float = 0.0             # per-tick white noise, 1-sigma
    loc_heading_noise_rad: float = 0.0    # per-tick white heading noise
    # fault windows (heartbeat / link / node), see Fault
    faults: List[Fault] = field(default_factory=list)
    link_replay: bool = True              # reliable kinds replay after a link drop
    keep_log: bool = False
    trace_every_s: float = 0.0            # >0: record truth poses


# ==================================================================== bus ==
class Bus:
    LOSSY = {"state", "intent"}

    def __init__(self, sc: Scenario, rng: random.Random):
        self.sc, self.rng = sc, rng
        self._q: list = []
        self._n = 0
        self.sent = {"state": 0, "intent": 0, "zone_request": 0,
                     "zone_grant": 0}
        self.sent.update(dict.fromkeys(TASK_KINDS, 0))
        self.dropped = dict.fromkeys(self.sent, 0)       # random loss
        self.fault_dropped = dict.fromkeys(self.sent, 0)  # fault / lifespan / depth
        self.replayed = dict.fromkeys(self.sent, 0)       # held over a link drop
        self.expired = 0                                  # state past its lifespan
        self._net_faults = [f for f in sc.faults if f.kind in ("heartbeat", "link")]
        # (src, dst, kind) -> payloads held while a link fault is active
        self._held: Dict[tuple, list] = {}

    def _fault(self, t: float, src: str, dst: str, kind: str) -> Optional[Fault]:
        for f in self._net_faults:
            if f.kind == "heartbeat" and kind != "state":
                continue
            if f.active(t) and f.covers(src, dst):
                return f
        return None

    def _delay(self) -> float:
        return self.sc.latency_s + (self.rng.uniform(0.0, self.sc.jitter_s)
                                    if self.sc.jitter_s > 0 else 0.0)

    def loopback(self, t: float, dst: str, kind: str, payload) -> None:
        """DDS delivers a node's own publication back to it; the sim bus
        skips src==dst, so self-delivery (needed for the no-bid retry
        announce to re-bid) is queued explicitly, never lost or delayed."""
        self._n += 1
        heapq.heappush(self._q, (t, self._n, dst, kind, payload, t))

    def send(self, t: float, src: str, kind: str, payload, dsts) -> None:
        loss = (self.sc.state_loss if kind == "state" else
                self.sc.intent_loss if kind == "intent" else
                self.sc.task_loss if kind in TASK_KINDS else self.sc.zone_loss)
        for dst in dsts:
            if dst == src:
                continue
            self.sent[kind] += 1
            f = self._fault(t, src, dst, kind) if self._net_faults else None
            if f is not None:
                if (f.kind == "link" and kind in RELIABLE_KINDS
                        and self.sc.link_replay):
                    buf = self._held.setdefault((src, dst, kind), [])
                    buf.append(payload)
                    if len(buf) > COORD_DEPTH:      # KEEP_LAST(10) overwrite
                        buf.pop(0)
                        self.fault_dropped[kind] += 1
                else:
                    self.fault_dropped[kind] += 1
                continue
            if loss > 0.0 and self.rng.random() < loss:
                self.dropped[kind] += 1
                continue
            self._n += 1
            heapq.heappush(self._q, (t + self._delay(), self._n, dst, kind,
                                     payload, t))

    def due(self, t: float):
        if self._held:
            for key in list(self._held):
                src, dst, kind = key
                if self._fault(t, src, dst, kind) is None:
                    for payload in self._held.pop(key):
                        self.replayed[kind] += 1
                        self._n += 1
                        heapq.heappush(self._q, (t + self._delay(), self._n,
                                                 dst, kind, payload, t))
        while self._q and self._q[0][0] <= t + 1e-9:
            item = heapq.heappop(self._q)
            if item[3] == "state" and t - item[5] > STATE_LIFESPAN_S + 1e-9:
                self.expired += 1
                self.fault_dropped["state"] += 1
                continue
            yield item


def _state_wire_copy(s: RobotState) -> RobotState:
    """What conversions.state_to_ros/state_from_ros carry: no intent."""
    return RobotState(
        robot_id=s.robot_id, seq=s.seq, boot_id=s.boot_id, stamp=s.stamp,
        pose=Pose2D(s.pose.x, s.pose.y, s.pose.theta), v=s.v, w=s.w,
        status=s.status, task_id=s.task_id or "",
        task_priority=int(s.task_priority), battery_pct=float(s.battery_pct),
        waiting_time=float(s.waiting_time), waiting_for=s.waiting_for or "",
        priority_score=float(s.priority_score), alive=bool(s.alive),
        intent_seq=int(s.intent_seq) & 0xFFFFFFFF,
        loc_health=int(s.loc_health), loc_sigma_lat=float(s.loc_sigma_lat))


def _task_from_dict(d: dict) -> Task:
    """conversions.task_from_ros equivalent for the sim's dict wire format."""
    return Task(task_id=d["task_id"],
                pickup=(int(d["pickup_row"]), int(d["pickup_col"])),
                dropoff=(int(d["dropoff_row"]), int(d["dropoff_col"])),
                priority=int(d.get("priority", 0)),
                created_at=float(d.get("created_at", 0.0)),
                deadline=float(d.get("deadline", 0.0)))


def _task_to_dict(task: Task) -> dict:
    return dict(task_id=task.task_id, pickup_row=task.pickup[0],
                pickup_col=task.pickup[1], dropoff_row=task.dropoff[0],
                dropoff_col=task.dropoff[1], priority=task.priority,
                created_at=task.created_at, deadline=task.deadline)


def _intent_wire_copy(it: Intent) -> Intent:
    return Intent(cells=[(int(r), int(c)) for r, c in it.cells],
                  t_enter=list(it.t_enter), t_exit=list(it.t_exit),
                  zones=list(it.zones),
                  goal=Pose2D(it.goal.x, it.goal.y, it.goal.theta),
                  active_zone_index=int(it.active_zone_index))


# ================================================================== agent ==
class SimAgent:
    """One robot: FleetCoordinator + mirrored fleet_agent_node glue + body."""

    def __init__(self, sim: "FleetSim", spec: RobotSpec):
        self.sim, self.spec, self.id = sim, spec, spec.rid
        sc = sim.sc
        self.grid = GridMap.from_yaml(str(sc.grid_yaml))
        self.grid.clearance_cost()
        self.v_nominal = sc.v_nominal
        self.coord = FleetCoordinator(
            robot_id=self.id, gridmap=self.grid,
            send_zone_request=self._send_zone_request,
            send_zone_grant=self._send_zone_grant,
            v_nominal=sc.v_nominal, mode=sc.mode, logger=self._clog)
        self.coord.registry.t_suspect = PEER_SUSPECT_S
        self.coord.registry.t_dead = max(PEER_SUSPECT_S + 0.1, PEER_DEAD_S)
        self.coord.state.boot_id = sim.rng.getrandbits(64)
        self.coord.state.battery_pct = spec.battery_pct
        # The real distributed auction over the lossy bus (fleet_agent_node
        # wiring, R4(4)). The announce also loops back to the sender, as DDS
        # does, so a no-bid retry re-bids locally.
        self.auction = TaskAuction(
            self.id,
            announce=self._send_task_announce,
            bid=lambda d: self._send_task("task_bid", d),
            award=self._send_task_award,
            path_length_m=self.coord.path_length_m_between,
            release=lambda d, reason: self._send_task(
                "task_release", dict(d, robot_id=self.id, reason=reason)),
            logger=self._nlog)
        self._near_static = self._build_static_margin(
            OBSTACLE_STATIC_MARGIN_CELLS)
        self.dock = None
        if spec.dock and docking is not None:
            home = self.grid.world_to_cell(spec.x, spec.y)
            if self.grid.is_static_free(home):
                self.dock = docking.DockPolicy(home, idle_s=4.0, low_pct=30.0,
                                               resume_pct=80.0,
                                               tolerance_m=0.25)
        # truth body
        self.x, self.y, self.th = spec.x, spec.y, spec.theta
        self.v = self.w = 0.0
        self.bx = self.by = 0.0          # localization bias (OU)
        self.jx = self.jy = self.jth = 0.0   # localization jitter (white)
        self.loc_err_sum = 0.0
        self.loc_err_max = 0.0
        self.loc_err_n = 0
        # node executor stalls ("node" faults) and the gate's permit age
        self._stalls = [f for f in sim.sc.faults
                        if f.kind == "node" and f.rid == spec.rid]
        self.inbox: list = []
        self.permit_t = -1e9             # sim time of the last permit
        self.stalled_ticks = 0
        self.inbox_dropped = 0
        # node runtime state (fleet_agent_node.__init__)
        self.state_seq = 0
        self.intent_seq = 0
        self._last_intent_pub = 0.0
        self._last_intent_hash = None
        self._task_phase = None
        self._route_failure_since = None
        self._dwell_until: Optional[float] = None
        self._dwell_resume = None
        self._retreat_cell = None
        self._retreat_until = None
        self._retreat_cooldown_until = 0.0
        self._distance_m = 0.0
        self._battery_t: Optional[float] = None
        self._battery_seen = 0.0
        # follower
        self._path_sig = None
        self._pts: List[Tuple[float, float]] = []
        self._prog = 0
        self._rotating = False
        # outputs / metrics
        self.permit = Permit(action="STOP", speed_scale=0.0, reason="boot")
        self.permit_counts: Dict[str, int] = {}
        self.wait_run = 0.0
        self.max_wait = 0.0
        self.stall_run = 0.0
        self.max_stall = 0.0
        self.go_stall_run = 0.0
        self.max_go_stall = 0.0
        self.no_refuge_ticks = 0
        self.pickups = 0
        self.deliveries = 0
        self.coord.state.status = IDLE
        self.coord.state.alive = True
        # Map pose is valid before any task can be awarded (node startup gate).
        self.coord.state.pose = Pose2D(self.x, self.y, self.th)

    # ----------------------------------------------------------- plumbing
    def now(self, t: float) -> float:
        return T0 + t + self.sim.sc.clock_offsets.get(self.id, 0.0)

    def belief(self) -> Tuple[float, float, float]:
        """The pose the robot believes (truth + localization error)."""
        return (self.x + self.bx + self.jx, self.y + self.by + self.jy,
                _wrap(self.th + self.jth))

    def stalled(self, t: float) -> bool:
        return any(f.active(t) for f in self._stalls)

    def enqueue(self, kind: str, payload, t: float) -> None:
        """Executor frozen: the subscription queues hold the message."""
        self.inbox.append((kind, payload, t))

    def drain_inbox(self, t: float) -> None:
        """Executor resumed: process what the subscription queues kept.
        STATE_QOS: depth 1 + 0.5 s lifespan; COORD_QOS: KEEP_LAST(10)."""
        def src_of(kind, p):
            if kind == "state":
                return p.robot_id
            if kind == "intent":
                return p[0]
            if kind == "zone_request":
                return p.get("robot_id")
            if kind == "zone_grant":
                return p.get("granter_id")
            return (p.get("robot_id") or p.get("winner_id")
                    or p.get("requester_id") or "task_gen")
        keep, seen = [], {}
        for kind, p, ta in reversed(self.inbox):
            key = (src_of(kind, p), kind)
            n = seen.get(key, 0)
            depth = 1 if kind == "state" else COORD_DEPTH
            if n >= depth or (kind == "state" and t - ta > STATE_LIFESPAN_S):
                self.inbox_dropped += 1
                continue
            seen[key] = n + 1
            keep.append((kind, p))
        self.inbox = []
        for kind, p in reversed(keep):
            self.receive(kind, p, t)

    def _clog(self, msg: str) -> None:
        self.sim.log_line(self.id, msg)

    def _nlog(self, msg: str) -> None:
        self.sim.log_line(self.id, "[node] " + msg)

    def _send_zone_request(self, d: dict) -> None:
        self.sim.bus.send(self.sim.t, self.id, "zone_request", dict(d),
                          self.sim.agent_ids)

    def _send_zone_grant(self, d: dict) -> None:
        self.sim.bus.send(self.sim.t, self.id, "zone_grant", dict(d),
                          self.sim.agent_ids)

    def _send_task(self, kind: str, d: dict) -> None:
        self.sim.bus.send(self.sim.t, self.id, kind, dict(d),
                          self.sim.agent_ids)

    def _send_task_announce(self, d: dict) -> None:
        self._send_task("task_announce", d)
        self.sim.bus.loopback(self.sim.t, self.id, "task_announce", dict(d))

    def _send_task_award(self, d: dict) -> None:
        """Broadcast my award and apply it locally at once (the node gets
        its own award back through DDS; the bus skips self)."""
        self._send_task("task_award", d)
        self._on_task_award(dict(d), self.sim.t)

    def _my_task_class(self) -> int:
        if self.auction.my_task is None:
            return 0
        return 3 if self._task_phase == "DROPOFF" else 2

    def _on_task_award(self, d: dict, t: float) -> None:
        """fleet_agent_node.cb_task_award, both branches."""
        now = self.now(t)
        self.auction.on_award(dict(task_id=d["task_id"],
                                   winner_id=d["winner_id"]))
        tid, winner = d["task_id"], d["winner_id"]
        already_running = (self.coord.current_task is not None
                           and self.coord.current_task.task_id == tid)
        if winner == self.id and already_running:
            return                          # echo of my own re-assert
        if winner == self.id and self.auction.my_task is not None:
            task = self.auction.my_task
            self.coord.current_task = task
            self.coord.state.task_id = task.task_id
            self.coord.state.task_priority = task.priority
            self._task_phase = "PICKUP"
            self.coord.task_phase = "PICKUP"
            self.coord.task_t0 = self.auction.first_heard.get(tid, now)
            if self.dock is not None:
                self.dock.reset()
            if self._dwell_until is not None:
                self._dwell_resume = (task.task_id, task.pickup)
            else:
                self.coord.set_goal(task.pickup, now=now)
            self.sim.on_award(self, task, t)
            self._nlog(f"awarded {tid} (auction) pickup={task.pickup} "
                       f"dropoff={task.dropoff}")
        elif (winner != self.id and already_running
              and self.auction.my_task is None):
            self._nlog(f"lost claim collision on {tid} to {winner}")
            self._drop_task()

    def _on_task_cancel(self, d: dict, t: float) -> None:
        """fleet_agent_node.cb_task_cancel (R6)."""
        now = self.now(t)
        tid = d["task_id"]
        mine = (self.auction.my_task is not None
                and self.auction.my_task.task_id == tid)
        if mine and self._task_phase == "DROPOFF":
            self.auction.on_cancel(tid, now, my_phase=self._task_phase)
            self._nlog(f"cancel refused for {tid}: already loaded")
            self.sim.events.append(dict(t=round(t, 1), robot=self.id,
                                        kind="cancel_refused", task=tid))
            return
        self.auction.on_cancel(tid, now,
                               my_phase=self._task_phase if mine else None)
        if mine:
            self._drop_task()
            if (self._dwell_resume is not None
                    and self._dwell_resume[0] == tid):
                self._dwell_resume = None
            self._nlog(f"task {tid} cancelled; aborted before pickup")
            self.sim.events.append(dict(t=round(t, 1), robot=self.id,
                                        kind="cancel_aborted", task=tid))

    def receive(self, kind: str, payload, t: float) -> None:
        now = self.now(t)
        if kind == "state":
            if payload.robot_id == self.id:
                return
            st = _state_wire_copy(payload)
            self.coord.on_peer_state(st, now=now)
            if self.sim.sc.distributed_auction:
                # R4(4): the peer's broadcast task_id is claim ground truth.
                lost = self.auction.on_peer_task(
                    st.robot_id, st.task_id,
                    _class_of_score(st.priority_score), now,
                    self._my_task_class())
                if lost:
                    self._nlog(f"claim collision (peer state): dropping "
                               f"{st.task_id} to {st.robot_id}")
                    self._drop_task()
        elif kind == "task_announce":
            self._sync_auction(t)
            self.auction.on_announce(_task_from_dict(payload), now=now)
        elif kind == "task_bid":
            if payload.get("robot_id") != self.id:
                self.auction.on_bid(dict(payload))
        elif kind == "task_award":
            self._on_task_award(dict(payload), t)
        elif kind == "task_complete":
            tid = payload["task_id"]
            self.auction.on_complete(tid)
            if (self.coord.current_task is not None
                    and self.coord.current_task.task_id == tid):
                self._drop_task()
        elif kind == "task_release":
            if payload.get("robot_id") == self.id:
                return
            self._sync_auction(t)
            if not self.auction.on_release(_task_from_dict(payload),
                                           payload["robot_id"], now=now):
                return
            if (self.coord.current_task is not None
                    and self.coord.current_task.task_id == payload["task_id"]
                    and self.auction.my_task is None):
                self._drop_task()
        elif kind == "task_cancel":
            self._on_task_cancel(dict(payload), t)
        elif kind == "intent":
            rid, seq, stamp, it = payload
            if rid == self.id:
                return
            local = it.to_receiver_clock(stamp, now)   # intent_from_ros
            self.coord.registry.update_intent(rid, local, int(seq), now=now,
                                              stamp=stamp)
        elif kind == "zone_request":
            if payload.get("robot_id") == self.id:
                return
            self.coord.on_zone_request(dict(payload))
        elif kind == "zone_grant":
            if payload.get("granter_id") == self.id:
                return
            self.coord.on_zone_grant(dict(payload))

    # ---------------------------------------------------------- task glue
    def _sync_auction(self, t: float) -> None:
        """fleet_agent_node._sync_auction_inputs."""
        au = self.auction
        au.current_cell = self.grid.world_to_cell(self.coord.state.pose.x,
                                                  self.coord.state.pose.y)
        au.battery_pct = self.coord.state.battery_pct
        au.nominal_speed = self.v_nominal
        au.state_status = self.coord.state.status
        au.charge_hold = bool(self.dock is not None
                              and getattr(self.dock, "charge_hold", False))
        now = self.now(t)
        au.remaining_dwell_s = (max(0.0, self._dwell_until - now)
                                if self._dwell_until is not None else 0.0)
        self.coord.task_phase = (self._task_phase
                                 if au.my_task is not None else None)

    def award(self, task: Task, t: float) -> None:
        """fleet_agent_node.cb_task_award, winner branch (scripted/direct)."""
        now = self.now(t)
        self.auction.my_task = task
        self.auction.first_heard.setdefault(task.task_id, now)
        self.auction.assignments[task.task_id] = self.id
        self.coord.current_task = task
        self.coord.state.task_id = task.task_id
        self.coord.state.task_priority = task.priority
        self._task_phase = "PICKUP"
        self.coord.task_phase = "PICKUP"
        self.coord.task_t0 = now
        if self.dock is not None:
            self.dock.reset()
        if self._dwell_until is not None:
            self._dwell_resume = (task.task_id, task.pickup)
        else:
            self.coord.set_goal(task.pickup, now=now)
        self.sim.on_award(self, task, t)
        self._nlog(f"awarded {task.task_id} pickup={task.pickup} "
                   f"dropoff={task.dropoff}")

    def _drop_task(self) -> None:
        self.auction.my_task = None
        self.coord.current_task = None
        self.coord.state.task_id = ""
        self.coord.state.task_priority = 0
        self.coord.clear_goal()
        self.coord.arbiter.release_all()
        self._task_phase = None
        self.coord.task_phase = None
        self.coord.task_t0 = None

    def _advance_task_lifecycle(self, now: float, t: float) -> None:
        if self._dwell_until is not None:
            if now < self._dwell_until:
                return
            self._end_dwell(now)
        task = self.auction.my_task
        if task is None:
            self._task_phase = None
            self.coord.task_phase = None
            self._route_failure_since = None
            self._advance_dock(now)
            return
        if self.coord.goal_cell is not None and not self.coord.path:
            if self._route_failure_since is None:
                self._route_failure_since = now
            if now - self._route_failure_since >= ROUTE_FAILURE_TIMEOUT_S:
                self._nlog(f"task {task.task_id} route unavailable; released")
                if self.sim.sc.distributed_auction:
                    self.auction.release_current(
                        now=now, reason="route unavailable")
                self.sim.on_task_released(self, task, t)
                self._drop_task()
                self._route_failure_since = None
            return
        self._route_failure_since = None

        target = task.pickup if self._task_phase in (None, "PICKUP") else task.dropoff
        pose = self.coord.state.pose
        tx, ty = self.grid.cell_to_world(target)
        dist = math.hypot(pose.x - tx, pose.y - ty)
        threshold = max(0.25, 0.5 * self.grid.resolution)
        settled = abs(self.coord.state.v) < 0.02 and abs(self.coord.state.w) < 0.05
        arrived = dist <= threshold and (settled or dist <= 0.1)
        if self._task_phase in (None, "PICKUP") and arrived:
            self._task_phase = "DROPOFF"       # LOADED: cancel refused now
            self.coord.task_phase = "DROPOFF"
            self._start_dwell(now, (task.task_id, task.dropoff))
            self.pickups += 1
            self.sim.on_pickup(self, task, t)
            return
        if self._task_phase == "DROPOFF" and arrived:
            if self.sim.sc.distributed_auction:
                self._send_task("task_complete",
                                dict(task_id=task.task_id, robot_id=self.id))
            self.auction.complete_current(now)
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._task_phase = None
            self.coord.task_phase = None
            self.coord.task_t0 = None
            self._start_dwell(now, None)
            self.deliveries += 1
            self.sim.on_delivery(self, task, t)

    def _start_dwell(self, now: float, resume) -> None:
        self._dwell_until = now + self.sim.sc.dwell_s
        self._dwell_resume = resume
        if self._retreat_until is not None:
            self._retreat_cell = None
            self._retreat_until = None
            self._retreat_cooldown_until = now + 10.0

    def _end_dwell(self, now: float) -> None:
        self._dwell_until = None
        resume, self._dwell_resume = self._dwell_resume, None
        task = self.auction.my_task
        if resume is not None and task is not None and task.task_id == resume[0]:
            self.coord.set_goal(resume[1], now=now)

    def _advance_dock(self, now: float) -> None:
        if self.dock is None:
            return
        pose = self.coord.state.pose
        hx, hy = self.grid.cell_to_world(self.dock.home_cell)
        settled = abs(self.coord.state.v) < 0.02 and abs(self.coord.state.w) < 0.05
        action = self.dock.step(
            now, has_task=self.auction.my_task is not None,
            transient=(self._dwell_until is not None
                       or self._retreat_until is not None
                       or getattr(self.coord, "_retreat", None) is not None
                       or self.coord.state.status == YIELDING),
            dist_home_m=math.hypot(pose.x - hx, pose.y - hy),
            settled=settled, has_goal=self.coord.goal_cell is not None,
            battery_pct=self.coord.state.battery_pct)
        if action == docking.GO_HOME:
            self.coord.set_goal(self.dock.home_cell, now=now)
            self._nlog("no tasks left: returning to dock")
        elif action == docking.DOCK:
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._nlog("docked at home")

    def _update_battery(self, now: float) -> None:
        travelled = self._distance_m - self._battery_seen
        self._battery_seen = self._distance_m
        pct = self.coord.state.battery_pct - BATTERY_PER_METRE * max(0.0, travelled)
        if (self._battery_t is not None and self.dock is not None
                and self.dock.phase == docking.DOCKED):
            pct += CHARGE_RATE * max(0.0, now - self._battery_t)
        self._battery_t = now
        self.coord.state.battery_pct = min(100.0, max(0.0, pct))

    # --------------------------------------------- node YIELD manoeuvre
    def _build_static_margin(self, margin: int) -> set:
        near = set()
        g = self.grid
        for r in range(g.height):
            row = g.static[r]
            for c in range(g.width):
                if row[c]:
                    for dr in range(-margin, margin + 1):
                        for dc in range(-margin, margin + 1):
                            near.add((r + dr, c + dc))
        return near

    def _maybe_retreat(self, permit: Permit, now: float) -> Permit:
        if self._retreat_until is not None:
            pose = self.coord.state.pose
            tx, ty = self.grid.cell_to_world(self._retreat_cell)
            done = math.hypot(pose.x - tx, pose.y - ty) <= 0.2
            if done or now >= self._retreat_until:
                self._retreat_cell = None
                self._retreat_until = None
                self._retreat_cooldown_until = now + 10.0
                task = self.auction.my_task
                if task is not None:
                    resume = (task.pickup if self._task_phase in (None, "PICKUP")
                              else task.dropoff)
                    self.coord.set_goal(resume, now=now)
                    self._nlog("retreat finished; resuming task route")
                else:
                    self.coord.clear_goal()
            return permit
        if permit.action != "YIELD" or now < self._retreat_cooldown_until:
            return permit
        cell = self._pick_retreat_cell()
        if cell is None:
            return permit
        self._retreat_cell = cell
        self._retreat_until = now + 8.0
        self.coord.set_goal(cell, now=now)
        self._nlog(f"YIELD: retreating to {cell} to clear "
                   f"{permit.blocking_robot or 'the deadlock'}")
        self.sim.on_node_yield(self, cell)
        try:
            return self.coord.tick(now)
        except Exception:
            return permit

    def _pick_retreat_cell(self):
        pose = self.coord.state.pose
        peers = [(p.pose.x, p.pose.y)
                 for p in self.coord.registry.alive().values()]
        res = self.grid.resolution
        cur = self.grid.world_to_cell(pose.x, pose.y)
        span = int(1.8 / res)
        best, best_clear = None, 0.0
        for dr in range(-span, span + 1):
            for dc in range(-span, span + 1):
                d = math.hypot(dr, dc) * res
                if not 0.8 <= d <= 1.8:
                    continue
                cell = (cur[0] + dr, cur[1] + dc)
                if not self.grid.is_static_free(cell) or cell in self._near_static:
                    continue
                x, y = self.grid.cell_to_world(cell)
                clear = min((math.hypot(x - px, y - py) for px, py in peers),
                            default=9.9)
                if clear < 0.7:
                    continue
                if clear > best_clear:
                    best, best_clear = cell, clear
        return best

    # ---------------------------------------------------------- the tick
    def node_tick(self, t: float) -> None:
        """fleet_agent_node.tick (healthy, localized branch)."""
        now = self.now(t)
        st = self.coord.state
        st.pose = Pose2D(*self.belief())
        st.v, st.w = self.v, self.w
        self._update_battery(now)
        self._advance_task_lifecycle(now, t)
        permit = self.coord.tick(now)
        if (self.dock is not None and self.dock.phase == docking.DOCKED
                and st.status == IDLE):
            st.status = CHARGING
        if self._dwell_until is not None:
            permit = Permit(action="STOP", speed_scale=0.0, reason="arrival dwell")
        else:
            permit = self._maybe_retreat(permit, now)
        self.permit = permit
        self.permit_t = t
        self.permit_counts[permit.action] = self.permit_counts.get(permit.action, 0) + 1
        if "no refuge reachable" in permit.reason:
            self.no_refuge_ticks += 1
        if self.sim.sc.distributed_auction:
            # fleet_agent_node.tick: resolve auctions after the coord tick.
            self._sync_auction(t)
            self.auction.tick(now, self.coord.registry.alive_ids(),
                              self.coord.registry.last_seen)
        self._maybe_publish_intent(now, t)

    def _maybe_publish_intent(self, now: float, t: float) -> None:
        cur = self.coord.current_intent(now)
        h = (tuple(self.coord.intent.cells), tuple(self.coord.intent.zones),
             tuple(cur.zones))
        changed = h != self._last_intent_hash
        if changed or (now - self._last_intent_pub) > INTENT_KEEPALIVE_S:
            if changed:
                self.intent_seq += 1
            self.sim.bus.send(t, self.id, "intent",
                              (self.id, self.intent_seq, now,
                               _intent_wire_copy(cur)), self.sim.agent_ids)
            self._last_intent_hash = h
            self._last_intent_pub = now

    def publish_state(self, t: float) -> None:
        now = self.now(t)
        self.state_seq += 1
        st = self.coord.state
        st.seq = self.state_seq
        st.stamp = now
        st.intent_seq = self.intent_seq
        st.loc_health = 0
        st.loc_sigma_lat = (self.sim.sc.force_sigma_lat
                            if self.sim.sc.force_sigma_lat is not None
                            else self.sim.sc.loc_noise_m)
        self.sim.bus.send(t, self.id, "state", _state_wire_copy(st),
                          self.sim.agent_ids)

    # ---------------------------------------------------------- the body
    def _control(self) -> Tuple[float, float]:
        """RPP-like follower of coordinator.path on the BELIEF pose."""
        path = self.coord.path
        sig = tuple(path)
        bx, by, bth = self.belief()
        if sig != self._path_sig:
            self._path_sig = sig
            self._pts = [self.grid.cell_to_world(c) for c in path]
            self._prog = (min(range(len(self._pts)),
                              key=lambda i: (self._pts[i][0] - bx) ** 2
                                            + (self._pts[i][1] - by) ** 2)
                          if self._pts else 0)
        pts = self._pts
        if not pts:
            return 0.0, 0.0
        n = len(pts)

        def dist(i):
            return math.hypot(pts[i][0] - bx, pts[i][1] - by)

        # Forward-only progress (Nav2-style pruning, bounded by path length
        # so an out-and-back spike to an acquisition point is driven, not
        # skipped), then drop points already reached.
        best, best_d, arc, i = self._prog, dist(self._prog), 0.0, self._prog
        while i + 1 < n and arc <= PRUNE_ARC_M:
            arc += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            i += 1
            if dist(i) < best_d - 1e-9:
                best, best_d = i, dist(i)
        self._prog = best
        while self._prog < n - 1 and dist(self._prog) < REACH_M:
            self._prog += 1
        gx, gy = pts[-1]
        d_goal = math.hypot(gx - bx, gy - by)
        if d_goal <= GOAL_TOL_M and self._prog >= n - 3:
            self._rotating = False
            return 0.0, 0.0
        look = min(LOOKAHEAD_MAX_M, max(LOOKAHEAD_MIN_M,
                                        abs(self.v) * LOOKAHEAD_TIME_S))
        # Carrot: first point at least `look` ahead ALONG the path.
        k, arc = n - 1, dist(self._prog)
        for i in range(self._prog, n):
            if i > self._prog:
                arc += math.hypot(pts[i][0] - pts[i - 1][0],
                                  pts[i][1] - pts[i - 1][1])
            if arc >= look:
                k = i
                break
        # Nav2's costmap keeps RPP off rack corners; a bare pure pursuit cuts
        # them. Pull the carrot back until the chord to it is drivable.
        while k > self._prog + 1 and not self._chord_clear(bx, by, *pts[k]):
            k -= 1
        cx, cy = pts[k]
        ld = max(1e-3, math.hypot(cx - bx, cy - by))
        alpha = _wrap(math.atan2(cy - by, cx - bx) - bth)
        # Rotate-to-heading with hysteresis (no flip-flop at the threshold).
        if abs(alpha) > ROTATE_TO_HEADING_RAD or (
                self._rotating and abs(alpha) > ROTATE_DONE_RAD):
            self._rotating = True
            return 0.0, math.copysign(ROTATE_W, alpha)
        self._rotating = False
        curv = 2.0 * math.sin(alpha) / ld
        v = self.v_nominal
        if abs(curv) > 1e-6 and 1.0 / abs(curv) < REGULATED_MIN_RADIUS_M:
            v = max(REGULATED_MIN_SPEED,
                    v * (1.0 / abs(curv)) / REGULATED_MIN_RADIUS_M)
        if d_goal < APPROACH_DIST_M:
            v = max(APPROACH_MIN_SPEED, v * d_goal / APPROACH_DIST_M)
        return v, v * curv

    def _chord_clear(self, x0, y0, x1, y1) -> bool:
        clr = self.grid.clearance_m()
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
        for s in range(1, n + 1):
            f = s / n
            r, c = self.grid.world_to_cell(x0 + f * (x1 - x0), y0 + f * (y1 - y0))
            if not self.grid.is_static_free((r, c)) or clr[r][c] < CHORD_CLEARANCE_M:
                return False
        return True

    def physics_step(self, dt: float, rng: random.Random,
                     t: Optional[float] = None) -> None:
        v_raw, w_raw = self._control()
        p = self.permit
        fresh = t is None or t - self.permit_t <= PERMIT_TIMEOUT_S + 1e-9
        if fresh and p.action in ("GO", "SLOW"):
            s = max(0.0, min(1.0, float(p.speed_scale)))
            v_cmd = max(-GATE_MAX_V, min(GATE_MAX_V, v_raw * s))
            w_cmd = max(-GATE_MAX_W, min(GATE_MAX_W, w_raw * s))
        else:
            v_cmd = w_cmd = 0.0
        dv = max(-MAX_ACCEL * dt, min(MAX_ACCEL * dt, v_cmd - self.v))
        dw = max(-MAX_ANG_ACCEL * dt, min(MAX_ANG_ACCEL * dt, w_cmd - self.w))
        self.v += dv
        self.w += dw
        x0, y0 = self.x, self.y
        thm = self.th + 0.5 * self.w * dt
        self.x += self.v * math.cos(thm) * dt
        self.y += self.v * math.sin(thm) * dt
        self.th = _wrap(self.th + self.w * dt)
        self._distance_m += math.hypot(self.x - x0, self.y - y0)
        sig = self.sim.sc.loc_noise_m
        if sig > 0.0:
            tau = self.sim.sc.loc_noise_tau_s
            k = sig * math.sqrt(2.0 * dt / tau)
            self.bx += -self.bx * dt / tau + k * rng.gauss(0.0, 1.0)
            self.by += -self.by * dt / tau + k * rng.gauss(0.0, 1.0)
        sc = self.sim.sc
        if sc.loc_jitter_m > 0.0:
            self.jx = rng.gauss(0.0, sc.loc_jitter_m)
            self.jy = rng.gauss(0.0, sc.loc_jitter_m)
        if sc.loc_heading_noise_rad > 0.0:
            self.jth = rng.gauss(0.0, sc.loc_heading_noise_rad)
        err = math.hypot(self.bx + self.jx, self.by + self.jy)
        self.loc_err_sum += err
        self.loc_err_n += 1
        self.loc_err_max = max(self.loc_err_max, err)

    def has_goal(self) -> bool:
        return self.coord.goal_cell is not None or bool(self.coord.path)


# ==================================================================== sim ==
_RE_REFUGE = re.compile(r"retreating to refuge \((\d+), (\d+)\)")


class FleetSim:
    def __init__(self, sc: Scenario):
        self.sc = sc
        self.rng = random.Random(sc.seed)
        self.t = 0.0
        self.bus = Bus(sc, random.Random(sc.seed + 1))
        self.logs: List[Tuple[float, str, str]] = []
        self.agents: List[SimAgent] = []
        self.agent_ids: List[str] = [r.rid for r in sc.robots]
        for spec in sc.robots:
            self.agents.append(SimAgent(self, spec))
        self.by_id = {a.id: a for a in self.agents}
        self._noise_rng = random.Random(sc.seed + 2)
        # tasks
        self.scripted = sorted(
            [(t0, a.id, task) for a in self.agents for t0, task in a.spec.tasks],
            key=lambda e: e[0])
        self.stream = sc.stream
        self._task_rng = random.Random(sc.seed + 3)
        self._next_emit = sc.stream.interval_s if sc.stream else float("inf")
        self.n_announced = 0
        self.live_tasks: Dict[str, Task] = {}
        self.pending: List[list] = []            # [task, t_ready] (central mode)
        self._announce_q = sorted(sc.announced, key=lambda e: e[0])
        self._cancel_q = sorted(sc.cancels, key=lambda e: e[0])
        self.events: List[dict] = []
        # per-task cycle times: first announce (or direct award) -> delivery
        self.task_announce_t: Dict[str, float] = {}
        self.task_cycles: List[dict] = []        # {task, robot, t, cycle_s}
        # metrics
        self.min_sep = float("inf")
        self.min_sep_at = None
        # Wheel-inclusive ground-truth RECT gap (core/hitbox contract): a
        # contact is gap <= 0 on TRUE poses; a near miss is gap < 0.05 m.
        # (The old 0.42 m centre-distance CONTACT_M over-counted rectangles
        # passing laterally and under-counted nose-to-nose.)
        self.min_rect_gap = float("inf")
        self.min_rect_gap_at = None
        self.contact_ticks = 0
        self.contact_events = 0
        self._in_contact = set()
        self.near_miss_events = 0
        self._in_near_miss = set()
        # duplicate task ownership (ground truth from auction.my_task)
        self.dup_owner_ticks = 0
        self.dup_owner_max_run = 0.0
        self._dup_run: Dict[str, float] = {}
        # the user's 3-stuck case: every robot has a goal yet none translates
        self.three_stuck_ticks = 0
        self._three_stuck_run = 0.0
        self.three_stuck_max_run = 0.0
        # mutual yields: both report waiting_for each other (rising edge)
        self.mutual_yield_events = 0
        self._in_mutual = set()
        # lane passes: opposite headings close together inside the lane band
        self.lane_passes = 0
        self._in_lane_pass = set()
        self.spine_head_on = 0
        self._head_on_pairs = set()
        self.spine_both_ticks = 0
        self.zone_rects = {zid: z for zid, z in self.agents[0].grid.zones.items()
                           if z.capacity == 1}
        self.zone_occ = {zid: {"ticks": 0, "events": 0} for zid in self.zone_rects}
        self._zone_dbl = set()
        self.plan_occ = {zid: {"ticks": 0, "events": 0} for zid in PLAN_ZONES}
        self._plan_dbl = set()
        self.double_hold = {}
        self.peer_dead_ticks: Dict[str, int] = {}
        self.vis_num = self.vis_den = 0
        self.rack_scrape_ticks = {a.id: 0 for a in self.agents}
        self.node_yields: List[dict] = []
        self.trace: List[dict] = []
        self._clear = self.agents[0].grid.clearance_m()
        # LANES trajectory audit (see LaneAudit)
        self.lane_audit = LaneAudit(self.agents[0].grid, sc.grid_yaml)
        self.wrong_lane_ticks = 0
        self.wrong_lane_events = 0
        self.wrong_lane_by_robot = {a.id: 0 for a in self.agents}
        self.wrong_lane_samples: List[dict] = []
        self._in_wrong_lane = set()
        self.gated_uturn_count = 0
        self._uturn_prev = {a.id: False for a in self.agents}
        self.uturn_wait_ticks = 0
        # coordinator.reverse_active (own-lane reverse retreat): exempt from
        # the wrong-lane audit ONLY inside the half the manoeuvre started in
        # (latched at the rising edge) or off-road - the flag can never
        # whitewash true wrong-way driving in the opposing half.
        self._rev_prev = {a.id: False for a in self.agents}
        self._rev_own = {a.id: None for a in self.agents}
        self.reverse_count = 0
        self.reverse_ticks = 0
        # coordinator.overtake_active (gated street-pass of a stationary
        # blocker): exempt from the wrong-lane audit ONLY inside the band the
        # manoeuvre engaged in (latched at the rising edge) - the borrowed
        # opposing half is legal there by the overtake gate; a flagged robot
        # travelling wrong-way in any OTHER band stays a violation.
        self._ot_prev = {a.id: False for a in self.agents}
        self._ot_band = {a.id: None for a in self.agents}
        self.overtake_count = 0
        self.overtake_ticks = 0

    # ------------------------------------------------------------ logging
    def log_line(self, rid: str, msg: str) -> None:
        self.logs.append((round(self.t, 2), rid, msg))

    def on_pickup(self, agent, task, t):
        self.events.append(dict(t=round(t, 1), robot=agent.id, kind="pickup",
                                task=task.task_id, cell=task.pickup))

    def on_award(self, agent, task, t):
        self.events.append(dict(t=round(t, 1), robot=agent.id, kind="award",
                                task=task.task_id, cell=task.pickup))
        self.task_announce_t.setdefault(task.task_id, t)

    def on_delivery(self, agent, task, t):
        self.events.append(dict(t=round(t, 1), robot=agent.id, kind="delivery",
                                task=task.task_id, cell=task.dropoff))
        self.live_tasks.pop(task.task_id, None)
        t0 = self.task_announce_t.get(task.task_id)
        if t0 is not None:
            self.task_cycles.append(dict(task=task.task_id, robot=agent.id,
                                         t=round(t, 1),
                                         cycle_s=round(t - t0, 1)))

    def on_task_released(self, agent, task, t):
        self.events.append(dict(t=round(t, 1), robot=agent.id, kind="released",
                                task=task.task_id))
        if self.stream is not None:
            self.pending.append([task, t + self.stream.bid_window_s])

    def on_node_yield(self, agent, cell):
        x, y = agent.grid.cell_to_world(cell)
        self.node_yields.append(dict(t=round(self.t, 1), robot=agent.id,
                                     cell=cell, xy=(round(x, 2), round(y, 2))))

    # ------------------------------------------------------------- tasks
    def _announce_on_bus(self, task: Task, t: float) -> None:
        """task_generator_node.emit: announce on the fleet bus; the robots'
        own distributed auctions do the rest."""
        self.task_announce_t.setdefault(task.task_id, t)
        self.live_tasks[task.task_id] = task
        self.events.append(dict(t=round(t, 1), robot="", kind="announce",
                                task=task.task_id, cell=task.pickup))
        self.bus.send(t, "task_gen", "task_announce", _task_to_dict(task),
                      self.agent_ids)

    def cancel_task(self, task_id: str, t: Optional[float] = None,
                    requester: str = "webapp",
                    reason: str = "user abort") -> None:
        """R6: publish a TaskCancel on the bus (what sih-57's webapp does).
        The generator mirror frees the pickup dedup entry at once."""
        t = self.t if t is None else t
        self.bus.send(t, requester, "task_cancel",
                      dict(task_id=task_id, requester_id=requester,
                           reason=reason), self.agent_ids)
        self.live_tasks.pop(task_id, None)
        self.events.append(dict(t=round(t, 1), robot=requester, kind="cancel",
                                task=task_id))

    def _tick_tasks(self, t: float) -> None:
        while self.scripted and self.scripted[0][0] <= t + 1e-9:
            _, rid, task = self.scripted.pop(0)
            self.live_tasks[task.task_id] = task
            self.by_id[rid].award(task, t)
        while self._announce_q and self._announce_q[0][0] <= t + 1e-9:
            _, task = self._announce_q.pop(0)
            self.n_announced += 1
            self._announce_on_bus(task, t)
        while self._cancel_q and self._cancel_q[0][0] <= t + 1e-9:
            _, tid = self._cancel_q.pop(0)
            self.cancel_task(tid, t)
        s = self.stream
        if s is None:
            return
        if t + 1e-9 >= self._next_emit:
            self._next_emit += s.interval_s
            if self.n_announced < s.num_tasks:
                busy = {tk.pickup for tk in self.live_tasks.values()}
                free = [p for p in s.pickups if p not in busy]
                if free:
                    self.n_announced += 1
                    pk = tuple(self._task_rng.choice(free))
                    dp = tuple(self._task_rng.choice(s.dropoffs))
                    prio = self._task_rng.choices([0, 1, 2, 3],
                                                  weights=s.priority_weights, k=1)[0]
                    task = Task(task_id=f"T-{self.n_announced:04d}", pickup=pk,
                                dropoff=dp, priority=prio, created_at=T0 + t,
                                deadline=T0 + t + _TASK_BUDGET_S[prio])
                    if self.sc.distributed_auction:
                        self._announce_on_bus(task, t)
                    else:
                        self.live_tasks[task.task_id] = task
                        self.task_announce_t.setdefault(task.task_id, t)
                        self.pending.append([task, t + s.bid_window_s])
                        self.events.append(dict(t=round(t, 1), robot="",
                                                kind="announce",
                                                task=task.task_id, cell=pk))
        if self.sc.distributed_auction:
            return                 # the per-robot auctions resolve the rest
        for entry in list(self.pending):
            task, t_ready = entry
            if t + 1e-9 < t_ready:
                continue
            bids = {}
            for a in self.agents:
                if a._stalls and a.stalled(t):
                    continue                      # a frozen node cannot bid
                au = a.auction
                au.current_cell = a.grid.world_to_cell(a.coord.state.pose.x,
                                                       a.coord.state.pose.y)
                au.battery_pct = a.coord.state.battery_pct
                au.nominal_speed = a.v_nominal
                au.state_status = a.coord.state.status
                au.charge_hold = bool(a.dock is not None
                                      and getattr(a.dock, "charge_hold", False))
                b = au.compute_bid(task)
                if b is not None:
                    bids[a.id] = b
            if not bids:
                entry[1] = t + s.no_bid_retry_s
                continue
            winner = min(bids, key=lambda r: (-bids[r], r))
            self.pending.remove(entry)
            self.by_id[winner].award(task, t)

    # ----------------------------------------------------------- metrics
    def _sample(self, t: float) -> None:
        ags = self.agents
        cells = {a.id: a.grid.world_to_cell(a.x, a.y) for a in ags}
        for i in range(len(ags)):
            a = ags[i]
            c = cells[a.id]
            if (a.grid.is_static_free(c)
                    and self._clear[c[0]][c[1]] < CHASSIS_HALF_WIDTH_M):
                self.rack_scrape_ticks[a.id] += 1
            elif not a.grid.is_static_free(c):
                self.rack_scrape_ticks[a.id] += 1
            for j in range(i + 1, len(ags)):
                b = ags[j]
                key = (a.id, b.id)
                d = math.hypot(a.x - b.x, a.y - b.y)
                if d < self.min_sep:
                    self.min_sep, self.min_sep_at = d, dict(
                        t=round(t, 1), pair=key,
                        a=(round(a.x, 2), round(a.y, 2)),
                        b=(round(b.x, 2), round(b.y, 2)))
                # Wheel-inclusive TRUE rect gap (replaces the 0.42 m
                # centre-distance contact test for pair metrics).
                if d < 2.0 * HB_R_CIRC + 0.8:
                    gap = rect_gap((a.x, a.y, a.th), (b.x, b.y, b.th))
                else:
                    gap = d - 2.0 * HB_R_CIRC    # safe lower bound when far
                if gap < self.min_rect_gap:
                    self.min_rect_gap, self.min_rect_gap_at = gap, dict(
                        t=round(t, 1), pair=key,
                        a=(round(a.x, 2), round(a.y, 2), round(a.th, 2)),
                        b=(round(b.x, 2), round(b.y, 2), round(b.th, 2)))
                if gap <= 0.0:
                    self.contact_ticks += 1
                    if key not in self._in_contact:
                        self.contact_events += 1
                        self._in_contact.add(key)
                else:
                    self._in_contact.discard(key)
                if gap < NEAR_MISS_M:
                    if key not in self._in_near_miss:
                        self.near_miss_events += 1
                        self._in_near_miss.add(key)
                else:
                    self._in_near_miss.discard(key)
                # mutual yield: both claim to wait for each other (rising edge)
                mutual = (a.coord.state.waiting_for == b.id
                          and b.coord.state.waiting_for == a.id)
                if mutual and key not in self._in_mutual:
                    self.mutual_yield_events += 1
                    self._in_mutual.add(key)
                elif not mutual:
                    self._in_mutual.discard(key)
                ca, cb = cells[a.id], cells[b.id]
                # lane pass: opposite headings close together inside the band
                # d < 1.3: a measured opposite-lane pass (centres +-0.4 m
                # lateral) runs at 1.15-1.26 m centre distance with the row
                # stagger - 1.2 missed a real pass (integrator smoke).
                passing = (_in_rect(ca, LANE_BAND) and _in_rect(cb, LANE_BAND)
                           and d < 1.3
                           and math.cos(a.th - b.th) < -0.3
                           and abs(a.v) > 0.05 and abs(b.v) > 0.05)
                if passing and key not in self._in_lane_pass:
                    self.lane_passes += 1
                    self._in_lane_pass.add(key)
                elif not passing and d > 1.5:
                    self._in_lane_pass.discard(key)
                both = _in_rect(ca, SPINE_RECT) and _in_rect(cb, SPINE_RECT)
                if both:
                    self.spine_both_ticks += 1
                head_on = False
                if both and d < 1.5 and a.coord.goal_cell and b.coord.goal_cell:
                    da = a.coord.goal_cell[0] - ca[0]
                    db = b.coord.goal_cell[0] - cb[0]
                    head_on = da * db < 0
                if head_on and key not in self._head_on_pairs:
                    self.spine_head_on += 1
                    self._head_on_pairs.add(key)
                elif not head_on and d > 2.0:
                    self._head_on_pairs.discard(key)
        # duplicate task ownership (R4(4) ground truth from my_task)
        owners: Dict[str, int] = {}
        for a in ags:
            tk = a.auction.my_task
            if tk is not None:
                owners[tk.task_id] = owners.get(tk.task_id, 0) + 1
        dup = {tid for tid, n in owners.items() if n >= 2}
        self.dup_owner_ticks += len(dup)
        for tid in dup:
            self._dup_run[tid] = self._dup_run.get(tid, 0.0) + DT
            self.dup_owner_max_run = max(self.dup_owner_max_run,
                                         self._dup_run[tid])
        for tid in list(self._dup_run):
            if tid not in dup:
                del self._dup_run[tid]
        # the user's 3-stuck case: every robot is tasked/goaled, none moving
        if len(ags) >= 3:
            engaged = all(a.has_goal() and a._dwell_until is None for a in ags)
            frozen = all(abs(a.v) < 0.02 for a in ags)
            if engaged and frozen:
                self.three_stuck_ticks += 1
                self._three_stuck_run += DT
                self.three_stuck_max_run = max(self.three_stuck_max_run,
                                               self._three_stuck_run)
            else:
                self._three_stuck_run = 0.0
        # zone double occupancy (ground truth) - yaml zones
        for zid, z in self.zone_rects.items():
            n = sum(1 for c in cells.values() if c in z.cells)
            if n >= 2:
                self.zone_occ[zid]["ticks"] += 1
                if zid not in self._zone_dbl:
                    self.zone_occ[zid]["events"] += 1
                    self._zone_dbl.add(zid)
            else:
                self._zone_dbl.discard(zid)
        for zid, rect in PLAN_ZONES.items():
            n = sum(1 for c in cells.values() if _in_rect(c, rect))
            if n >= 2:
                self.plan_occ[zid]["ticks"] += 1
                if zid not in self._plan_dbl:
                    self.plan_occ[zid]["events"] += 1
                    self._plan_dbl.add(zid)
            else:
                self._plan_dbl.discard(zid)
        # protocol-level: two arbiters HELD on one zone
        held: Dict[str, int] = {}
        for a in ags:
            for zid in a.coord.arbiter.held_zones():
                held[zid] = held.get(zid, 0) + 1
        for zid, n in held.items():
            if n >= 2:
                self.double_hold[zid] = self.double_hold.get(zid, 0) + 1
        # liveness as each robot's registry sees its peers
        for a in ags:
            for rid in a.coord.registry.dead_ids():
                k = f"{a.id}<-{rid}"
                self.peer_dead_ticks[k] = self.peer_dead_ticks.get(k, 0) + 1
        # LANES trajectory audit: wrong-lane occupancy + U-turn gate flags
        for a in ags:
            uturn = bool(getattr(a.coord, "uturn_active", False))
            if uturn and not self._uturn_prev[a.id]:
                self.gated_uturn_count += 1
            self._uturn_prev[a.id] = uturn
            if (getattr(a.coord, "uturn_waiting", False)
                    or getattr(a.coord, "uturn_pending", False)
                    or getattr(a.coord, "_uturn_block_t0", None) is not None):
                self.uturn_wait_ticks += 1
            rev = bool(getattr(a.coord, "reverse_active", False))
            if rev:
                self.reverse_ticks += 1
                if not self._rev_prev[a.id]:
                    self.reverse_count += 1
                    self._rev_own[a.id] = None     # re-latch per manoeuvre
                if self._rev_own[a.id] is None:
                    self._rev_own[a.id] = self.lane_audit.half_at(cells[a.id])
            self._rev_prev[a.id] = rev
            ota = bool(getattr(a.coord, "overtake_active", False))
            if ota:
                self.overtake_ticks += 1
                if not self._ot_prev[a.id]:
                    self.overtake_count += 1
                    self._ot_band[a.id] = None     # re-latch per manoeuvre
                if self._ot_band[a.id] is None:
                    half = self.lane_audit.half_at(cells[a.id])
                    if half is not None:
                        self._ot_band[a.id] = half[0]
            self._ot_prev[a.id] = ota
            viol = (not uturn
                    and self.lane_audit.violation(cells[a.id], a.v, a.th))
            if viol and ota:
                half = self.lane_audit.half_at(cells[a.id])
                if half is None or half[0] == self._ot_band[a.id]:
                    viol = False
            if viol and rev:
                half = self.lane_audit.half_at(cells[a.id])
                # Own-lane (the latched half) or off-road: a legal reverse
                # retreat. A different half = wrong-way in the opposing
                # lane - reverse_active does NOT whitewash it.
                if half is None or half == self._rev_own[a.id]:
                    viol = False
            if viol:
                self.wrong_lane_ticks += 1
                self.wrong_lane_by_robot[a.id] += 1
                if a.id not in self._in_wrong_lane:
                    self.wrong_lane_events += 1
                    self._in_wrong_lane.add(a.id)
                    if len(self.wrong_lane_samples) < LANE_AUDIT_SAMPLES:
                        self.wrong_lane_samples.append(dict(
                            t=round(t, 1), robot=a.id, cell=cells[a.id],
                            v=round(a.v, 2),
                            th_deg=round(math.degrees(_wrap(a.th)), 0)))
            else:
                self._in_wrong_lane.discard(a.id)
        # waits
        for a in ags:
            busy = a.has_goal() and a._dwell_until is None
            blocked = busy and a.permit.action not in ("GO", "SLOW")
            a.wait_run = a.wait_run + DT if blocked else 0.0
            a.max_wait = max(a.max_wait, a.wait_run)
            stalled = busy and abs(a.v) < 0.02 and abs(a.w) < 0.05
            a.stall_run = a.stall_run + DT if stalled else 0.0
            a.max_stall = max(a.max_stall, a.stall_run)
            # Permitted to move but not translating (controller spin, or a
            # real follower failure): should stay a few seconds at most.
            go_stall = (busy and a.permit.action in ("GO", "SLOW")
                        and abs(a.v) < 0.02)
            a.go_stall_run = a.go_stall_run + DT if go_stall else 0.0
            a.max_go_stall = max(a.max_go_stall, a.go_stall_run)
            # peer-intent visibility: observers of a peer with a goal
            for b in ags:
                if b is a or not b.coord.path:
                    continue
                self.vis_den += 1
                p = a.coord.registry.peers.get(b.id)
                if p is not None and p.intent.cells:
                    self.vis_num += 1
        if self.sc.trace_every_s > 0 and \
                abs((t / self.sc.trace_every_s) - round(t / self.sc.trace_every_s)) < 1e-6:
            self.trace.append(dict(t=round(t, 1), robots={
                a.id: (round(a.x, 2), round(a.y, 2), a.permit.action)
                for a in ags}))

    # --------------------------------------------------------------- run
    def step(self) -> None:
        t = self.t
        for _, _, dst, kind, payload, _ in self.bus.due(t):
            a = self.by_id[dst]
            if a._stalls and a.stalled(t):
                a.enqueue(kind, payload, t)
            else:
                a.receive(kind, payload, t)
        self._tick_tasks(t)
        for a in self.agents:
            if a._stalls and a.stalled(t):
                a.stalled_ticks += 1
                continue
            if a.inbox:
                a.drain_inbox(t)
            a.node_tick(t)
            a.publish_state(t)
        for a in self.agents:
            a.physics_step(DT, self._noise_rng, t)
        self.t = round(t + DT, 6)
        self._sample(self.t)

    def run(self, duration_s: float) -> dict:
        n = int(round(duration_s / DT))
        for _ in range(n):
            self.step()
        return self.metrics(duration_s)

    def _cycle_stats(self) -> dict:
        """Per-task cycle times (announce -> delivery), in delivery order.
        Halves compare early vs late throughput: a growing second half means
        the backlog is winning (R5 acceptance: 2nd <= 1.15 x 1st)."""
        cs = [c["cycle_s"] for c in self.task_cycles]
        if not cs:
            return dict(n=0, mean_s=None, max_s=None, first_half_mean_s=None,
                        second_half_mean_s=None, per_task=[])
        h = (len(cs) + 1) // 2
        first, second = cs[:h], cs[h:]
        return dict(
            n=len(cs),
            mean_s=round(sum(cs) / len(cs), 1),
            max_s=round(max(cs), 1),
            first_half_mean_s=round(sum(first) / len(first), 1),
            second_half_mean_s=(round(sum(second) / len(second), 1)
                                if second else None),
            per_task=self.task_cycles)

    def metrics(self, duration_s: float) -> dict:
        def count(pat):
            return sum(1 for _, _, m in self.logs if pat in m)

        refuges = []
        for t, rid, m in self.logs:
            mm = _RE_REFUGE.search(m)
            if mm:
                cell = (int(mm.group(1)), int(mm.group(2)))
                x, y = self.by_id[rid].grid.cell_to_world(cell)
                refuges.append(dict(t=t, robot=rid, cell=cell,
                                    xy=(round(x, 2), round(y, 2)),
                                    in_spine=_in_rect(cell, SPINE_RECT),
                                    in_spine_gap=in_spine_gap(cell)))
        dl = {a.id: a.coord.deadlock.metrics() for a in self.agents}
        dl_events = [dict(robot=a.id, t=round(e["t_detect"] - T0, 1),
                          cycle=e["cycle"], trigger=e["trigger"],
                          resolved=e["resolved"],
                          recovery_s=(round(e["recovery_time"], 1)
                                      if e["recovery_time"] is not None else None))
                     for a in self.agents for e in a.coord.deadlock.events]
        out = dict(
            scenario=self.sc.name, duration_s=duration_s, seed=self.sc.seed,
            n_robots=len(self.agents),
            deliveries=sum(a.deliveries for a in self.agents),
            pickups=sum(a.pickups for a in self.agents),
            deliveries_per_robot={a.id: a.deliveries for a in self.agents},
            tasks_announced=self.n_announced,
            tasks_pending=len(self.pending),
            min_separation_m=round(self.min_sep, 3) if self.agents[1:] else None,
            min_separation_at=self.min_sep_at,
            contacts=self.contact_events,
            contact_ticks=self.contact_ticks,
            min_rect_gap_m=(round(self.min_rect_gap, 3)
                            if self.agents[1:] else None),
            min_rect_gap_at=self.min_rect_gap_at,
            near_misses=self.near_miss_events,
            duplicate_owner_s=round(self.dup_owner_ticks * DT, 1),
            duplicate_owner_max_s=round(self.dup_owner_max_run, 1),
            three_stuck_s=round(self.three_stuck_ticks * DT, 1),
            three_stuck_max_s=round(self.three_stuck_max_run, 1),
            mutual_yield_events=self.mutual_yield_events,
            lane_passes=self.lane_passes,
            task_cycle_s=self._cycle_stats(),
            makeway_started=count("make-way"),
            makeway_failed=count("make-way: no target")
                           + count("make-way failed"),
            max_wait_s={a.id: round(a.max_wait, 1) for a in self.agents},
            max_stall_s={a.id: round(a.max_stall, 1) for a in self.agents},
            max_go_stall_s={a.id: round(a.max_go_stall, 1) for a in self.agents},
            deadlocks_detected=sum(m["deadlocks_detected"] for m in dl.values()),
            deadlocks_resolved=sum(m["deadlocks_resolved"] for m in dl.values()),
            deadlock_max_recovery_s=round(max(
                [m["max_recovery_s"] for m in dl.values()] or [0.0]), 1),
            deadlock_events=dl_events,
            reroutes=count("deadlock victim: rerouted clear of"),
            retreats_started=count("deadlock victim: retreating to refuge"),
            retreats_completed=count("retreat: done"),
            no_refuge_yield_s=round(sum(a.no_refuge_ticks for a in self.agents)
                                    * DT, 1),
            node_yield_retreats=len(self.node_yields),
            zone_timeouts=sum(a.coord.arbiter.stats["timeouts"] for a in self.agents),
            hold_lease_releases=count("hold-lease release"),
            replan_failed=count("REPLAN FAILED"),
            deadlock_log_lines=count("DEADLOCK cycle") + count("DEADLOCK timeout"),
            refuges=refuges,
            refuges_in_spine=sum(1 for r in refuges if r["in_spine"]),
            refuges_in_spine_gap=sum(1 for r in refuges if r["in_spine_gap"]),
            node_yields=self.node_yields,
            spine_head_on_encounters=self.spine_head_on,
            spine_shared_s=round(self.spine_both_ticks * DT, 1),
            # LANES trajectory audit (LaneAudit; crossing cells and gated
            # U-turns excluded, reversing/lateral motion never counted)
            wrong_lane_ticks=self.wrong_lane_ticks,
            wrong_lane_s=round(self.wrong_lane_ticks * DT, 1),
            wrong_lane_events=self.wrong_lane_events,
            wrong_lane_by_robot={k: round(v * DT, 1)
                                 for k, v in self.wrong_lane_by_robot.items()},
            wrong_lane_samples=self.wrong_lane_samples,
            gated_uturn_count=self.gated_uturn_count,
            uturn_wait_ticks=self.uturn_wait_ticks,
            uturn_wait_s=round(self.uturn_wait_ticks * DT, 1),
            reverse_count=self.reverse_count,
            reverse_s=round(self.reverse_ticks * DT, 1),
            overtake_count=self.overtake_count,
            overtake_s=round(self.overtake_ticks * DT, 1),
            zone_double_occupancy={k: v for k, v in self.zone_occ.items()},
            plan_zone_double_occupancy={k: v for k, v in self.plan_occ.items()},
            arbiter_double_hold_ticks=dict(self.double_hold),
            permit_actions={a.id: dict(a.permit_counts) for a in self.agents},
            intent_visibility=(round(self.vis_num / self.vis_den, 4)
                               if self.vis_den else None),
            distance_m={a.id: round(a._distance_m, 1) for a in self.agents},
            rack_scrape_s={k: round(v * DT, 1)
                           for k, v in self.rack_scrape_ticks.items()},
            bus_sent=dict(self.bus.sent), bus_dropped=dict(self.bus.dropped),
            bus_fault_dropped=dict(self.bus.fault_dropped),
            bus_replayed=dict(self.bus.replayed),
            state_expired=self.bus.expired,
            faults=[dict(rid=f.rid, t0=f.t0, t1=f.t1, kind=f.kind, peer=f.peer)
                    for f in self.sc.faults],
            node_stalled_s={a.id: round(a.stalled_ticks * DT, 1)
                            for a in self.agents},
            peer_dead_s={k: round(v * DT, 1)
                         for k, v in sorted(self.peer_dead_ticks.items())},
            peer_failures_detected=sum(
                a.coord.registry.stats.get("failures_detected", 0)
                for a in self.agents),
            loc_error_mean_m=round(
                sum(a.loc_err_sum for a in self.agents)
                / max(1, sum(a.loc_err_n for a in self.agents)), 3),
            loc_error_max_m=round(max(a.loc_err_max for a in self.agents), 3),
            events=self.events,
        )
        if self.sc.keep_log:
            out["log"] = self.logs
        if self.trace:
            out["trace"] = self.trace
        return out


def run(scenario: Scenario, duration_s: float) -> dict:
    """Run a scenario for duration_s simulated seconds; return metrics."""
    return FleetSim(scenario).run(duration_s)


# ============================================================== scenarios ==
def _cell_xy(cell):
    return (cell[1] + 0.5) * 0.1 - 6.0, (cell[0] + 0.5) * 0.1 - 5.0


def _arc_to_row(grid, start_xy, goal_cell, row):
    """Path length from start to the first cell of the A* route on `row`."""
    from amr_fleet.core import astar
    path = astar.astar(grid, grid.world_to_cell(*start_xy), goal_cell)
    d = 0.0
    for a, b in zip(path, path[1:]):
        if a[0] == row:
            return d
        d += grid.cell_distance_m(a, b)
    return d


def head_on(seed: int = 0, meet_row: int = 64, warmup_s: float = 0.0,
            **kw) -> Scenario:
    """Increment-1 head-on script with real tasks.

    A (robot_1): spawn (-3,-4) -> pickup L1 (88,25) -> dropoff D1.
    B (robot_2): starts ON L1 -> 'pickup' L1 (2 s load dwell) -> dropoff D2.
    The later-arriving robot starts first so both reach spine row `meet_row`
    (default 64: inside the gap through rack row 2, the worst case) together.
    warmup_s delays BOTH tasks (steady-state runs: the LANES gate needs
    LANES_HOLD_S of good sigma before it opens, so a task released inside
    the first 3 s meets the SPINE boot fallback, not the lanes).
    """
    grid = GridMap.from_yaml(str(kw.get("grid_yaml", GRID_YAML)))
    a_xy, b_xy = (-3.0, -4.0), _cell_xy(L1)
    v = kw.get("v_nominal", 0.6) * 0.8          # turns/accel make it slower
    t_a = _arc_to_row(grid, a_xy, L1, meet_row) / v
    t_b = kw.get("dwell_s", 2.0) + _arc_to_row(grid, b_xy, D2, meet_row) / v
    delay_a, delay_b = max(0.0, t_b - t_a), max(0.0, t_a - t_b)
    delay_a, delay_b = delay_a + warmup_s, delay_b + warmup_s
    robots = [
        RobotSpec("robot_1", a_xy[0], a_xy[1], math.pi / 2, dock=False,
                  tasks=[(delay_a, Task("H-A", pickup=L1, dropoff=D1))]),
        RobotSpec("robot_2", b_xy[0], b_xy[1], 0.0, dock=False,
                  tasks=[(delay_b, Task("H-B", pickup=L1, dropoff=D2))]),
    ]
    return Scenario(name="head_on", robots=robots, seed=seed, **kw)


def random_stream(seed: int = 7, n_robots: int = 3, interval_s: float = 15.0,
                  num_tasks: int = 20, **kw) -> Scenario:
    """Webots-like run: robots at their spawns/docks, task_generator stream."""
    robots = [RobotSpec(rid, x, y, math.pi / 2)
              for rid, x, y in SPAWNS[:n_robots]]
    return Scenario(name=f"random_seed{seed}", robots=robots, seed=seed,
                    stream=StreamSpec(interval_s=interval_s, num_tasks=num_tasks),
                    **kw)


def single_robot(pickup=L1, dropoff=D2, **kw) -> Scenario:
    robots = [RobotSpec("robot_1", -3.0, -4.0, math.pi / 2, dock=False,
                        tasks=[(0.0, Task("S-1", pickup=pickup, dropoff=dropoff))])]
    return Scenario(name="single", robots=robots, **kw)


def _mk_task(tid, pickup, dropoff, t, priority=1):
    return Task(tid, pickup=pickup, dropoff=dropoff, priority=priority,
                created_at=T0 + t,
                deadline=T0 + t + _TASK_BUDGET_S[priority])


def converge(seed: int = 0, **kw) -> Scenario:
    """The user's 3-stuck case: three robots parked ON the dropoff row,
    three tasks announced at the same instant with CROSSING routes. The
    batch assignment (T2) plus make-way must get all three picked up."""
    robots = [RobotSpec("robot_1", *_cell_xy(D1), theta=math.pi / 2, dock=False),
              RobotSpec("robot_2", *_cell_xy(D2), theta=math.pi / 2, dock=False),
              RobotSpec("robot_3", *_cell_xy(D3), theta=math.pi / 2, dock=False)]
    announced = [(1.0, _mk_task("C-1", L1, D2, 1.0)),
                 (1.0, _mk_task("C-2", L2, D1, 1.0)),
                 (1.0, _mk_task("C-3", L3, D3, 1.0))]
    return Scenario(name="converge", robots=robots, announced=announced,
                    seed=seed, **kw)


def three_at_once(seed: int = 0, **kw) -> Scenario:
    """R4(1): 3 robots at their spawns, 3 tasks at t=3 s announced together.
    The joint min-sum-cost assignment must be cost-optimal: the west robot
    is never sent to the east pickup while an east robot idles."""
    robots = [RobotSpec(rid, x, y, math.pi / 2) for rid, x, y in SPAWNS]
    announced = [(3.0, _mk_task("A-1", L1, D1, 3.0)),
                 (3.0, _mk_task("A-2", L2, D2, 3.0)),
                 (3.0, _mk_task("A-3", L3, D3, 3.0))]
    return Scenario(name="three_at_once", robots=robots, announced=announced,
                    seed=seed, **kw)


def three_way(seed: int = 0, variant: int = 0, **kw) -> Scenario:
    """Three robots forced through crossing routes (8 variants: all 6 pickup
    permutations, plus 2 with rotated dropoffs). Direct awards, so the
    crossing is guaranteed regardless of assignment quality."""
    perms = [(L1, L2, L3), (L1, L3, L2), (L2, L1, L3), (L2, L3, L1),
             (L3, L1, L2), (L3, L2, L1)]
    pick = perms[variant % 6]
    drops = (D1, D2, D3) if variant < 6 else (D3, D1, D2)
    robots = []
    for i, (rid, x, y) in enumerate(SPAWNS):
        t0 = 0.5 + 0.1 * i
        robots.append(RobotSpec(
            rid, x, y, math.pi / 2, dock=False,
            tasks=[(t0, _mk_task(f"W-{i + 1}", pick[i], drops[i], t0))]))
    return Scenario(name=f"three_way_v{variant}", robots=robots, seed=seed,
                    **kw)


def stress(seed: int = 7, n_robots: int = 3, loss: float = 0.4,
           fault_kind: str = "heartbeat", drop_s=(10.0, 16.0),
           gap_s=(30.0, 90.0), horizon_s: float = 900.0,
           latency_s: float = 0.05, jitter_s: float = 0.15, **kw) -> Scenario:
    """sih-57 double-hold stress case on the random stream: `loss` of
    robot_state AND intents dropped, every robot's heartbeat silent for
    U(drop_s) s every U(gap_s) s (peers declare it DEAD after 5 s), plus
    latency/jitter. Zone messages stay reliable (zone_loss=0)."""
    rids = [rid for rid, _, _ in SPAWNS[:n_robots]]
    kw.setdefault("state_loss", loss)
    kw.setdefault("intent_loss", loss)
    kw.setdefault("faults", random_faults(seed, rids, horizon_s, fault_kind,
                                          dur_s=drop_s, gap_s=gap_s))
    sc = random_stream(seed=seed, n_robots=n_robots, latency_s=latency_s,
                       jitter_s=jitter_s, **kw)
    sc.name = f"stress_{fault_kind}_seed{seed}"
    return sc


SCENARIOS = {"head_on": head_on, "random": random_stream, "single": single_robot,
             "stress": stress, "converge": converge,
             "three_at_once": three_at_once, "three_way": three_way}


def summary(m: dict) -> dict:
    """The headline numbers, without the long lists."""
    drop = {"events", "refuges", "node_yields", "deadlock_events", "log",
            "trace", "permit_actions", "zone_double_occupancy",
            "plan_zone_double_occupancy", "bus_sent", "bus_dropped",
            "bus_fault_dropped", "bus_replayed", "faults",
            "wrong_lane_samples"}
    out = {k: v for k, v in m.items() if k not in drop}
    if isinstance(out.get("task_cycle_s"), dict):
        out["task_cycle_s"] = {k: v for k, v in out["task_cycle_s"].items()
                               if k != "per_task"}
    out["zone_double_occupancy_events"] = {
        k: v["events"] for k, v in m["zone_double_occupancy"].items() if v["events"]}
    out["plan_zone_double_occupancy_events"] = {
        k: v["events"] for k, v in m["plan_zone_double_occupancy"].items()
        if v["events"]}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("scenario", choices=sorted(SCENARIOS))
    ap.add_argument("duration", type=float, nargs="?", default=300.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--loss", type=float, default=0.0)
    ap.add_argument("--latency", type=float, default=0.0)
    ap.add_argument("--loc-noise", type=float, default=0.0,
                    help="OU localization bias, 1-sigma m")
    ap.add_argument("--loc-jitter", type=float, default=0.0,
                    help="white localization jitter, 1-sigma m")
    ap.add_argument("--faults", choices=FAULT_KINDS, default=None,
                    help="random 10-16 s fault windows of this kind")
    ap.add_argument("--meet-row", type=int, default=None,
                    help="head_on: spine row where the robots meet")
    ap.add_argument("--variant", type=int, default=0,
                    help="three_way: crossing variant 0-7")
    ap.add_argument("--central", action="store_true",
                    help="A/B: the old centralised auction resolver")
    ap.add_argument("--force-sigma", type=float, default=None,
                    help="broadcast loc_sigma_lat override (LANES gate)")
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--log", action="store_true")
    ap.add_argument("--legacy", action="store_true",
                    help="run on test/fixtures/warehouse_grid_legacy.yaml")
    a = ap.parse_args(argv)
    kw = dict(latency_s=a.latency, loc_noise_m=a.loc_noise,
              loc_jitter_m=a.loc_jitter, keep_log=a.log,
              grid_yaml=str(LEGACY_YAML if a.legacy else GRID_YAML))
    if a.scenario == "stress":
        kw.pop("latency_s")
        if a.loss:
            kw["loss"] = a.loss
        if a.faults:
            kw["fault_kind"] = a.faults
    else:
        kw.update(state_loss=a.loss, intent_loss=a.loss)
        if a.faults:
            rids = [r for r, _, _ in SPAWNS]
            kw["faults"] = random_faults(a.seed, rids, a.duration, a.faults)
    if a.scenario == "head_on" and a.meet_row is not None:
        kw["meet_row"] = a.meet_row
    if a.scenario == "three_way":
        kw["variant"] = a.variant
    if a.central:
        kw["distributed_auction"] = False
    if a.force_sigma is not None:
        kw["force_sigma_lat"] = a.force_sigma
    sc = SCENARIOS[a.scenario](seed=a.seed, **kw)
    m = run(sc, a.duration)
    if a.log:
        for t, rid, msg in m["log"]:
            if not msg.startswith("ZONE REQ"):
                print(f"{t:7.1f} {rid} {msg}")
    print(json.dumps(m if a.full else summary(m), indent=1, default=str))


if __name__ == "__main__":
    main()
