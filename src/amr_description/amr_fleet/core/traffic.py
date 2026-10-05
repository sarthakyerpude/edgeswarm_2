"""
Topological traffic layer (L2 of wiki/Plan-P2P-Coordination.md, increment 2).

Pure functions over the GridMap's RANKED zones (zones with a 'rank' in
warehouse_grid.yaml). No ROS, no clocks: every caller passes 'now' through.

THE RULES (SPINE mode)
  1. Ranked zones are capacity-1 Ricart-Agrawala mutexes (core/zone.py,
     unchanged). Ranks: SP_* (0) < SPINE (1) < J_N (2) < J_AW (3); ties are
     broken by zone id, so the acquisition order is a strict total order.
  2. A robot stops at its ACQUISITION POINT before its path enters any ranked
     zone it is not already inside: a wait bay (WB_W / WB_E) when entering
     from the south, otherwise ACQ_STANDOFF_M short of the zone boundary (the
     spur mouth when leaving a spur).
  3. There it requests the zones its remaining path needs strictly in
     ascending rank, requesting the next one only once the previous one is
     HELD and physically empty, and enters only when it holds all of them.
  4. A zone is released when the robot has passed it (the coordinator's
     _maybe_release_passed_zones) and never while its pose is still inside.
  5. OCCUPANCY: an alive peer whose pose lies inside zone Z is a required
     granter AND a blocker for Z, even if its intent does not list Z
     (idle robots, stale intents, robots grandfathered across a release).

Why it cannot deadlock: every robot takes zones in the same total order and
never waits for a lower-ranked zone while holding a higher one (Havender's
resource ordering), so the wait-for relation over zones has no cycle. Every
request in the coordinator goes through request() below, which refuses an
out-of-order request instead of sending it.
"""
import math
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .models import Cell

SPINE_MODE, LANES_MODE = "SPINE", "LANES"

# LANES gate (increment 4, R3/R7): LANES is EFFECTIVE only while every robot
# near the band localizes well. Robots already inside the spine are
# grandfathered by the occupancy rule (a physical occupant blocks the mutex).
#
# Hysteresis (webots_final4 fix): the old single threshold (0.08, drop AT
# ONCE) flapped LANES<->SPINE on real AMCL, whose broadcast loc_sigma_lat
# was measured at 0.051-0.058 steady with excursions to ~0.155 between
# corrections - the fleet sat in the capacity-1 SPINE fallback almost the
# whole run. The gate is now a band: ENTER when everyone near the band is
# at or below LANE_SIGMA_MAX for LANES_HOLD_S; LEAVE only when someone is
# at or above LANE_SIGMA_EXIT (or LOC_DEGRADED/LOST) for LANES_EXIT_HOLD_S.
# Between the two thresholds the mode is sticky. Safe because the mode gate
# only selects GEOMETRY (keep-right lanes vs single-file mutex): the pass
# itself is judged pairwise every tick by the safety envelope with the LIVE
# sigmas (safety.envelope_permit lane-pass rule needs ~<=0.095 each), so a
# robot whose sigma spikes above the pass threshold is stopped/slowed by
# the envelope without collapsing the whole fleet's traffic mode. A ghost
# (SUSPECT/SILENT peer) near the band or a LOC_LOST robot still drops the
# mode immediately: that is a missing-position hazard, not a noisy sigma.
LANE_SIGMA_MAX = 0.18       # ENTER ceiling for every robot near the band
LANE_SIGMA_EXIT = 0.30      # LEAVE only at/above this, sustained
LANES_HOLD_S = 3.0          # good conditions must hold this long to enter
LANES_EXIT_HOLD_S = 5.0     # bad conditions must hold this long to leave
LANES_RADIUS_M = 6.0        # peers within this of the band are gate inputs

# Hitbox-cell occupancy inflation: r = K_SIG * sigma (contract K_SIG=2.0).
K_SIG = 2.0

_HB = None
_HB_TRIED = False


def _hitbox_mod():
    """core/hitbox.py is the HITBOX builder's module; it may land after this
    one. Fall back to pose-centre occupancy until it is importable."""
    global _HB, _HB_TRIED
    if _HB is None and not _HB_TRIED:
        try:
            from . import hitbox as _h
            _HB = _h
        except Exception:
            pass
        _HB_TRIED = True
    return _HB

# Stop this far (path length) before the first cell of a zone still to be
# acquired. 0.26 m footprint radius + ~0.15 m stopping/localization margin.
ACQ_STANDOFF_M = 0.4

FREE, REQUESTING, HELD = "FREE", "REQUESTING", "HELD"


# ------------------------------------------------------------------ ranks
def is_ranked(grid, zone_id: str) -> bool:
    z = grid.zones.get(zone_id)
    return z is not None and getattr(z, "rank", None) is not None


def rank_key(grid, zone_id: str) -> Tuple[int, str]:
    """Total acquisition order: (rank, zone_id)."""
    return (int(grid.zones[zone_id].rank), zone_id)


def ranked_zone_ids(grid) -> List[str]:
    return sorted((z for z in grid.zones if is_ranked(grid, z)),
                  key=lambda z: rank_key(grid, z))


def has_ranked_zones(grid) -> bool:
    return any(is_ranked(grid, z) for z in grid.zones)


def in_mode(zone, mode: str) -> bool:
    modes = getattr(zone, "modes", None)
    return modes is None or mode in modes


def rank_sorted(grid, zone_ids: Iterable[str]) -> List[str]:
    """Ranked zones in acquisition order, then unranked ones in given order."""
    ids = list(dict.fromkeys(zone_ids))
    ranked = sorted((z for z in ids if is_ranked(grid, z)),
                    key=lambda z: rank_key(grid, z))
    return ranked + [z for z in ids if not is_ranked(grid, z)]


# ------------------------------------------------------------ path queries
def zones_at(grid, cell: Cell, mode: Optional[str] = None) -> List[str]:
    """Ranked zones containing a cell (zones may overlap: J_AW is inside
    SPINE), in rank order. mode=None ignores per-mode zones filtering."""
    out = [zid for zid, z in grid.zones.items()
           if getattr(z, "rank", None) is not None and cell in z.cells
           and (mode is None or in_mode(z, mode))]
    return sorted(out, key=lambda z: rank_key(grid, z))


def required_zones(grid, path: Sequence[Cell],
                   mode: str = SPINE_MODE) -> List[str]:
    """Ranked zones the path enters that the mode requires, sorted by rank
    (the order in which they must be acquired)."""
    if not path:
        return []
    zs = [(zid, z) for zid, z in grid.zones.items()
          if getattr(z, "rank", None) is not None and in_mode(z, mode)]
    out: Set[str] = set()
    for cell in path:
        for zid, z in zs:
            if zid not in out and cell in z.cells:
                out.add(zid)
        if len(out) == len(zs):
            break
    return sorted(out, key=lambda z: rank_key(grid, z))


def zone_window(grid, path: Sequence[Cell], mode: str = SPINE_MODE,
                skip: Iterable[str] = (), guard_m: float = 0.5) -> List[str]:
    """R10 acquisition window: the ranked zones the NEXT stretch of path
    needs, in rank order - NOT the whole route's zone set.

    The window starts at the first pending (not in `skip`) zone the path
    enters, then grows to a fixpoint with every pending zone that
      - ranks at or below the window's maximum (it could never be requested
        later: requests must ascend in rank), or
      - is entered within guard_m of path of a window zone's first entry
        (no room to stop and acquire it separately).
    Zones outside the window are deferred to a later acquisition round. By
    construction every later round ranks strictly above everything still
    held from earlier rounds, so the Havender order is preserved while a
    corridor split into segments (SPINE_S/M/N) is acquired one segment at a
    time in the ascending-rank direction and pipelined between robots. In
    the descending-rank direction the window closes over the whole run, as
    rank order requires."""
    req = required_zones(grid, path, mode)
    pending = [z for z in req if z not in set(skip)]
    if not pending:
        return []
    entry: Dict[str, int] = {}
    for i, cell in enumerate(path):
        for zid in pending:
            if zid not in entry and cell in grid.zones[zid].cells:
                entry[zid] = i
    first = min(pending, key=lambda z: (entry[z], rank_key(grid, z)))
    win = {first}
    changed = True
    while changed:
        changed = False
        maxk = max(rank_key(grid, z) for z in win)
        lim = min(entry[z] for z in win)
        for z in pending:
            if z in win:
                continue
            if (rank_key(grid, z) <= maxk
                    or path_distance_m(grid, path, lim, entry[z]) <= guard_m):
                win.add(z)
                changed = True
    return sorted(win, key=lambda z: rank_key(grid, z))


def first_new_zone_index(grid, path: Sequence[Cell], start: int = 0,
                         inside: Optional[Iterable[str]] = None,
                         mode: str = SPINE_MODE) -> Optional[int]:
    """First index >= start whose cell lies in a mode-required ranked zone
    that the robot is not already inside (inside defaults to the zones of
    path[start])."""
    if not path or start >= len(path):
        return None
    inside = set(zones_at(grid, path[start], mode) if inside is None
                 else inside)
    zs = [(zid, z) for zid, z in grid.zones.items()
          if getattr(z, "rank", None) is not None and in_mode(z, mode)
          and zid not in inside]
    if not zs:
        return None
    for i in range(start, len(path)):
        cell = path[i]
        for _zid, z in zs:
            if cell in z.cells:
                return i
    return None


def _cell_dist(grid, a: Cell, b: Cell) -> float:
    return grid.cell_distance_m(a, b)


def acquisition_point(grid, path: Sequence[Cell], start: int = 0,
                      inside: Optional[Iterable[str]] = None,
                      mode: str = SPINE_MODE,
                      standoff_m: float = ACQ_STANDOFF_M) -> Optional[int]:
    """Index of the path cell where the robot must stop and acquire.

    None if the remaining path enters no new ranked zone. If a wait bay lies
    on the path before the first new-zone cell, the (last such) bay is the
    acquisition point; otherwise it is the last cell at least standoff_m of
    path length before that zone (the spur mouth when leaving a spur), never
    earlier than `start` (a robot already past it stops where it is).
    """
    k = first_new_zone_index(grid, path, start, inside, mode)
    if k is None:
        return None
    bays = set(getattr(getattr(grid, "traffic", None), "wait_bays", {})
               .values())
    for i in range(k - 1, start - 1, -1):
        if path[i] in bays:
            return i
    travelled = 0.0
    i = k
    while i > start:
        travelled += _cell_dist(grid, path[i - 1], path[i])
        i -= 1
        if travelled >= standoff_m:
            return i
    return start


def path_distance_m(grid, path: Sequence[Cell], i0: int, i1: int) -> float:
    if i1 <= i0:
        return 0.0
    return sum(_cell_dist(grid, path[i], path[i + 1]) for i in range(i0, i1))


# --------------------------------------------------------------- occupancy
def occupant_cells(grid, info: tuple) -> Set[Cell]:
    """Cells a peer's body occupies. info = (x, y) -> pose-centre cell
    (legacy); (x, y, theta, sigma) -> rasterised hitbox grown by K_SIG*sigma
    (core/hitbox.py), falling back to the centre cell until hitbox lands."""
    hb = _hitbox_mod()
    if len(info) >= 4 and hb is not None:
        try:
            return hb.cells(grid, info[0], info[1], info[2],
                            K_SIG * max(0.0, float(info[3])))
        except Exception:
            pass
    return {grid.world_to_cell(info[0], info[1])}


def zone_occupants(grid, zone_id: str,
                   peer_xy: Dict[str, tuple]) -> Set[str]:
    """Peers whose body (hitbox cells, or pose centre for 2-tuples) overlaps
    the zone."""
    z = grid.zones.get(zone_id)
    if z is None:
        return set()
    return {rid for rid, info in peer_xy.items()
            if occupant_cells(grid, info) & z.cells}


def occupants_by_zone(grid, peer_xy: Dict[str, tuple]
                      ) -> Dict[str, Set[str]]:
    """Ranked zone -> peers physically inside it (hitbox cells)."""
    out: Dict[str, Set[str]] = {}
    ranked = [(zid, grid.zones[zid]) for zid in grid.zones
              if getattr(grid.zones[zid], "rank", None) is not None]
    for rid, info in peer_xy.items():
        body = occupant_cells(grid, info)
        for zid, z in ranked:
            if body & z.cells:
                out.setdefault(zid, set()).add(rid)
    return out


def required_granters(peers_needing: Set[str], occupants: Set[str]) -> Set[str]:
    """Occupancy rule: an occupant must grant too, whatever its intent says."""
    return set(peers_needing) | set(occupants)


# ------------------------------------------------------------- rank guard
def out_of_order(grid, zone_id: str, arbiter_state: Dict[str, str],
                 ignore: Iterable[str] = ()) -> List[str]:
    """Ranked zones I hold or am requesting that rank ABOVE zone_id (minus
    `ignore`). Any entry means requesting zone_id now would be hold-and-wait
    out of order."""
    if not is_ranked(grid, zone_id):
        return []
    key = rank_key(grid, zone_id)
    ignore = set(ignore)
    return sorted(z for z, st in arbiter_state.items()
                  if st != FREE and z != zone_id and z not in ignore
                  and is_ranked(grid, z) and rank_key(grid, z) > key)


def request(grid, arbiter, zone_id: str, score: float, t_enter: float,
            t_exit: float, now: float,
            allow_above: Iterable[str] = ()) -> bool:
    """The ONLY way the coordinator sends a zone request.

    Ranked zones are requested only if no higher-ranked zone is held or
    requested (the caller backs those off first). `allow_above` names held
    zones the robot is physically inside: it cannot give those back, so they
    are the one tolerated exception (a route that changed mid-zone). Unranked
    legacy zones pass straight through. Returns True if a request is (or
    already was) in flight.
    """
    if arbiter.state.get(zone_id) in (REQUESTING, HELD):
        return True
    if out_of_order(grid, zone_id, arbiter.state, ignore=allow_above):
        return False
    try:
        arbiter.request(zone_id, score, t_enter, t_exit, now=now,
                        ranked=is_ranked(grid, zone_id))
    except TypeError:       # a stubbed/legacy arbiter without `ranked`
        arbiter.request(zone_id, score, t_enter, t_exit, now=now)
    return True


def rank_safe(grid, required: Iterable[str], held: Iterable[str],
              inside: Iterable[str]) -> bool:
    """True if a path needing `required` can be acquired in rank order from
    the current holdings: every still-unacquired zone must outrank every
    zone I hold AND am physically inside (those cannot be backed off;
    held-but-not-entered higher zones can be released first)."""
    held, inside = set(held), set(inside)
    committed = [z for z in held & inside if is_ranked(grid, z)]
    if not committed:
        return True
    top = max(rank_key(grid, z) for z in committed)
    for z in required:
        if z in held or z in inside or not is_ranked(grid, z):
            continue
        if rank_key(grid, z) < top:
            return False
    return True


# ------------------------------------------------------- designated cells
def designated_refuge_cells(grid, clearance=None,
                            min_clearance_m: float = 0.30
                            ) -> Tuple[List[Cell], List[Cell]]:
    """(tier 1, tier 2) deadlock-retreat targets from the yaml:
    tier 1 = retreat cells and wait bays; tier 2 = every refuge cell with at
    least min_clearance_m of static clearance (the refuge centre lines)."""
    t = getattr(grid, "traffic", None)
    if t is None:
        return [], []
    tier1 = (list(t.retreat_cells) + list(t.wait_bays.values())
             + [tuple(c) for c in pockets(grid)])
    tier2: List[Cell] = []
    if clearance is not None:
        for cells, _cost in t.refuges.values():
            for (r, c) in cells:
                if (grid.is_static_free((r, c))
                        and clearance[r][c] >= min_clearance_m):
                    tier2.append((r, c))
    return tier1, sorted(tier2)


# ------------------------------------------------------------ pockets/lanes
def pockets(grid) -> List[Cell]:
    """Designated pull-over pockets (R3: the 'third robot' capacity of the
    two-abreast lanes). Parsed by gridmap into grid.pockets; until that
    lands, read them off the TrafficConfig if present. Tier-1 make-way and
    refuge targets."""
    p = getattr(grid, "pockets", None)
    if p is None:
        p = getattr(getattr(grid, "traffic", None), "pockets", None)
    return [(int(c[0]), int(c[1])) for c in (p or [])]


def lane_band(grid):
    """The LaneBand (gridmap.LaneBand, HITBOX builder) or None. Duck-typed:
    anything with .rect/.in_band works, so tests can stub it."""
    return getattr(grid, "lanes", None)


def band_distance_m(grid, band, x: float, y: float) -> float:
    """World distance from (x, y) to the lane band's rectangle."""
    r0, c0, r1, c1 = band.rect
    x0, y0 = grid.cell_to_world((r0, c0))
    x1, y1 = grid.cell_to_world((r1, c1))
    xmin, xmax = min(x0, x1), max(x0, x1)
    ymin, ymax = min(y0, y1), max(y0, y1)
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


def select_mode(grid, me, peers: Dict[str, object], ghosts_xy,
                now: float, gate: Dict[str, float],
                lanes_supported: bool = True) -> str:
    """EFFECTIVE traffic mode: LANES only while it is safe, else SPINE.

    me:        my RobotState (loc_health / loc_sigma_lat are mine).
    peers:     alive peers' RobotStates (SUSPECT included).
    ghosts_xy: (x, y) of every SUSPECT/SILENT/ghost peer (stale data).
    gate:      mutable hysteresis state, owned by the caller
               ({'ok_since': t or None}; 'bad_since' and 'mode' are managed
               here); time only from `now`.
    lanes_supported: the planner/band plumbing is in place (callers pass
               False while gridmap/astar lane support has not landed).

    ENTER LANES only after, continuously for LANES_HOLD_S:
      - the yaml requests LANES and the grid has a lane band;
      - my loc_health is OK and sigma <= LANE_SIGMA_MAX;
      - every alive peer within LANES_RADIUS_M of the band reports
        loc_health OK and sigma <= LANE_SIGMA_MAX;
      - no SUSPECT/SILENT/ghost peer within LANES_RADIUS_M of the band.
    LEAVE LANES (hysteresis - see the gate comment above) only when
      - a ghost is near the band or a relevant robot reports LOC_LOST
        (drops AT ONCE, gate reset), or
      - a relevant robot reports loc_health DEGRADED or
        sigma >= LANE_SIGMA_EXIT continuously for LANES_EXIT_HOLD_S.
    Robots physically inside the spine when the mode drops are
    grandfathered by the occupancy rule.
    """
    band = lane_band(grid)
    requested = getattr(getattr(grid, "traffic", None), "mode", SPINE_MODE)
    if requested != LANES_MODE or band is None or not lanes_supported:
        gate["ok_since"] = gate["bad_since"] = None
        gate["mode"] = SPINE_MODE
        return SPINE_MODE

    in_lanes = gate.get("mode", SPINE_MODE) == LANES_MODE
    hard_bad = False            # ghost / LOC_LOST: no hysteresis
    soft_bad = False            # DEGRADED or sigma >= exit: sustained exit
    good = True                 # everyone at/below the ENTER ceiling

    def _classify(st) -> None:
        nonlocal hard_bad, soft_bad, good
        health = getattr(st, "loc_health", 0)
        sigma = float(getattr(st, "loc_sigma_lat", 0.0))
        if health >= 2:
            hard_bad = True
        if health != 0 or sigma >= LANE_SIGMA_EXIT:
            soft_bad = True
        if health != 0 or sigma > LANE_SIGMA_MAX:
            good = False

    _classify(me)
    for st in peers.values():
        pose = getattr(st, "pose", None)
        if pose is None:
            continue
        if band_distance_m(grid, band, pose.x, pose.y) > LANES_RADIUS_M:
            continue
        _classify(st)
    # ROAD MODEL (reservations off): a SUSPECT peer is usually just a late
    # state under CPU load (2.0-2.1 s gaps live) and the safety envelope
    # already bounds it (stale_reach). Dropping the lanes AT ONCE flipped
    # the whole fleet's geometry ~30x per run (userrestart7), each flip a
    # re-plan between lanes and centre lines. There it is a SUSTAINED exit
    # reason (LANES_EXIT_HOLD_S) like a high sigma; LOC_LOST stays hard.
    road_model = not getattr(getattr(grid, "traffic", None),
                             "reservations", True)
    for (gx, gy) in ghosts_xy:
        if band_distance_m(grid, band, gx, gy) <= LANES_RADIUS_M:
            if road_model:
                soft_bad = True
                good = False
            else:
                hard_bad = True
            break

    if hard_bad:
        gate["ok_since"] = gate["bad_since"] = None
        gate["mode"] = SPINE_MODE
        return SPINE_MODE

    if in_lanes:
        if soft_bad:
            t0 = gate.get("bad_since")
            if t0 is None:
                gate["bad_since"] = t0 = now
            if now - t0 >= LANES_EXIT_HOLD_S:
                gate["ok_since"] = gate["bad_since"] = None
                gate["mode"] = SPINE_MODE
                return SPINE_MODE
        else:
            gate["bad_since"] = None
        return LANES_MODE

    gate["bad_since"] = None
    if not good:
        gate["ok_since"] = None
        return SPINE_MODE
    t0 = gate.get("ok_since")
    if t0 is None:
        gate["ok_since"] = t0 = now
    if now - t0 >= LANES_HOLD_S:
        gate["mode"] = LANES_MODE
        return LANES_MODE
    return SPINE_MODE


# ================================================== strict directional lanes
# (R-lanes increment.) THE RULES: robots use only the lane built for their
# direction; move forward whenever there is clearance; a U-turn may cut
# across into the opposite lane ANYWHERE but only after checking the space;
# if the check fails, wait in your own lane or drive on to where the turn
# fits. The planner's gridmap exposes
#     grid.lane_band_at(row, col) -> Optional[(lane_id, direction)]
#     grid.in_crossing_zone(row, col) -> bool
# (direction NB/SB/EB/WB; None off every band and inside junction boxes,
# where in_crossing_zone is True instead). Everything below is duck-typed
# against those two names and degrades safely until they land: band_at
# returns None (no bands = no confinement, gate judges only what it can
# see), in_crossing falls back to the ranked 'intersection' zones.

DIR_VECS = {"NB": (0.0, 1.0), "SB": (0.0, -1.0),
            "EB": (1.0, 0.0), "WB": (-1.0, 0.0)}
OPPOSITE_DIR = {"NB": "SB", "SB": "NB", "EB": "WB", "WB": "EB"}

# U-turn gate sizing. Duration bounds the whole manoeuvre (rotate ~pi at the
# 0.3-0.9 rad/s the controller actually drives, plus crossing the ~0.9 m
# divider at reduced speed); REACT covers one decision+actuation latency of
# the approaching peer; BODY is two circumscribed radii (2*0.286) plus grid
# slack; FOLLOWER_GAP is the standoff a same-direction follower needs so my
# rotation never sweeps into it.
UTURN_DURATION_S = 4.0
UTURN_REACT_S = 0.3
UTURN_BODY_M = 0.6
UTURN_FOLLOWER_GAP_M = 0.8
UTURN_ACCEL_MPS2 = 2.0          # fleet accel cap (hitbox.A_MAX_MPS2 if there)
HB_R_CIRC_FALLBACK = 0.286      # circumscribed safety-rect radius
# Static-fit drivability bound for the crossing cells: wheel-inclusive
# half-width 0.205 m + 2 cm, the same lethal band the planner uses
# (gridmap.CLEARANCE_LETHAL_M). The full swept-disc + sigma requirement
# applies at the PIVOT (the crossing's widest cell), where the rotation
# physically happens.
UTURN_DRIVE_CLEAR_M = 0.225


def heading_dir(theta: float) -> str:
    """Dominant compass direction (NB/SB/EB/WB) of a map-frame heading."""
    dx, dy = math.cos(theta), math.sin(theta)
    if abs(dy) >= abs(dx):
        return "NB" if dy >= 0.0 else "SB"
    return "EB" if dx >= 0.0 else "WB"


def band_at(grid, cell) -> Optional[Tuple[str, str]]:
    """grid.lane_band_at at a (row, col) cell, or None when the planner's
    band map has not landed (duck-typed, stubbable)."""
    fn = getattr(grid, "lane_band_at", None)
    if fn is None:
        return None
    try:
        hit = fn(int(cell[0]), int(cell[1]))
    except Exception:
        return None
    return None if hit is None else (hit[0], hit[1])


def in_crossing(grid, cell) -> bool:
    """grid.in_crossing_zone at a cell; before it lands, the ranked
    'intersection' zones are the crossings (J_N / J_AW)."""
    fn = getattr(grid, "in_crossing_zone", None)
    if fn is not None:
        try:
            return bool(fn(int(cell[0]), int(cell[1])))
        except Exception:
            return False
    cell = (int(cell[0]), int(cell[1]))
    return any(getattr(z, "kind", "") == "intersection" and cell in z.cells
               for z in grid.zones.values())


def cell_lane_legal(grid, cell, my_dir: Optional[str]) -> bool:
    """Lane confinement for the ground RECOVERY driving may use: a cell is
    drivable iff it is off-road (south open floor), inside a crossing zone
    (junction box / turnaround - crossings are for moving through), in a
    band of my own or a PERPENDICULAR direction (turning into a cross
    corridor is ordinary driving; its keep-right is the planner's law) -
    never a cell of the OPPOSING lane. The own-lane yield shape rests on
    this: reversing stays inside the own band, which is always legal."""
    hit = band_at(grid, cell)
    if hit is None:
        return True
    if in_crossing(grid, cell):
        return True
    return my_dir is None or hit[1] != OPPOSITE_DIR.get(my_dir)


def cell_lane_parkable(grid, cell, my_dir: Optional[str],
                       designated=()) -> bool:
    """Lane confinement for PARK spots (refuges, make-way targets, retreat
    holds): crossing zones are for moving THROUGH, never for parking (a body
    held in a junction box blocks the cross flow for everyone - the measured
    (65,65) park inside the aisle_a x spine box); otherwise a park spot must
    be off-road, a DESIGNATED tier-1 cell (the engineered per-direction
    pockets / wait bays / retreat mouths), or a band cell that is NOT the
    opposing lane. A PERPENDICULAR band cell is reached by turning into that
    corridor, so the robot's travel there matches the band - parking in the
    cross corridor's correct-side half is how a spine robot clears the spine
    (aisle refuge lines). Only the exact opposing lane is barred."""
    cell = (int(cell[0]), int(cell[1]))
    if in_crossing(grid, cell):
        return False
    if cell in designated:
        return True
    hit = band_at(grid, cell)
    if hit is None:
        return True
    return my_dir is None or hit[1] != OPPOSITE_DIR.get(my_dir)


def confined_no_go(grid, my_dir: Optional[str]) -> Set[Cell]:
    """Every free cell cell_lane_legal forbids for my_dir (the opposing
    lane's cells). Empty until the planner's lane_band_at lands. Static
    geometry - cache per direction."""
    if getattr(grid, "lane_band_at", None) is None:
        return set()
    out: Set[Cell] = set()
    for r in range(grid.height):
        for c in range(grid.width):
            if grid.static[r][c]:
                continue
            if not cell_lane_legal(grid, (r, c), my_dir):
                out.add((r, c))
    return out


def lane_strip_cells(grid, lane_id: str, direction: str) -> Set[Cell]:
    """Every cell of ONE directional half-lane, across the band's full rect -
    junction boxes included, because the lane's traffic drives through them.
    This is the ground a yielder must leave free for the robot it yields to:
    the blocker WILL drive its lane, however far its broadcast intent
    currently reaches. Resolved from the grid's LaneBand objects (rect /
    axis / divider / centres); on band-map stubs without them, every cell
    lane_band_at maps to (lane_id, direction) is the fallback strip."""
    bands = list(getattr(grid, "lane_bands", None) or ())
    for i, b in enumerate(bands):
        name = getattr(b, "name", None) or f"band_{i}"
        if name != lane_id:
            continue
        r0, c0, r1, c1 = b.rect
        pos_dir, neg_dir = (("NB", "SB") if b.axis == "v" else ("EB", "WB"))
        out: Set[Cell] = set()
        for r in range(int(r0), int(r1) + 1):
            for c in range(int(c0), int(c1) + 1):
                pos = float(c if b.axis == "v" else r)
                d = pos - b.divider
                if d * (b.centre_pos - b.divider) > 0:
                    dd = pos_dir
                elif d * (b.centre_neg - b.divider) > 0:
                    dd = neg_dir
                else:
                    dd = (pos_dir if abs(pos - b.centre_pos)
                          <= abs(pos - b.centre_neg) else neg_dir)
                if dd == direction:
                    out.add((r, c))
        return out
    if getattr(grid, "lane_band_at", None) is None:
        return set()
    out = set()
    for r in range(grid.height):
        for c in range(grid.width):
            hit = band_at(grid, (r, c))
            if hit is not None and hit[0] == lane_id and hit[1] == direction:
                out.add((r, c))
    return out


def _xyts(p) -> Tuple[float, float, float, float]:
    """(x, y, theta, sigma) from a RobotState, a Pose2D-like or a tuple
    (x, y[, theta[, sigma]])."""
    if hasattr(p, "pose"):                  # RobotState-shaped
        q = p.pose
        return (q.x, q.y, getattr(q, "theta", 0.0),
                max(0.0, float(getattr(p, "loc_sigma_lat", 0.0) or 0.0)))
    if hasattr(p, "x"):                     # Pose2D-shaped
        return p.x, p.y, getattr(p, "theta", 0.0), 0.0
    t = tuple(p)
    return (float(t[0]), float(t[1]),
            float(t[2]) if len(t) > 2 else 0.0,
            max(0.0, float(t[3])) if len(t) > 3 else 0.0)


def _uturn_stop_dist(v: float, accel: float) -> float:
    """Stopping distance of an approaching peer: braking at the fleet accel
    cap plus one reaction interval at its speed."""
    v = abs(v)
    return v * v / (2.0 * max(0.1, accel)) + UTURN_REACT_S * v


def uturn_gate(now: float, my_pose, my_band, target_band, turn_cells,
               peers, grid) -> Tuple[bool, str]:
    """THE one U-turn predicate - planning and execution both use it.

    my_pose:     (x, y, theta[, sigma]) or Pose2D/RobotState (duck-typed).
    my_band:     (lane_id, direction) I am travelling in.
    target_band: (lane_id, direction) the turn lands in (the opposite lane).
    turn_cells:  the cells the turn cuts across (between the lane centres).
    peers:       broadcast RobotStates, dict rid->state or iterable.

    All checks are deterministic from the broadcast peer state/intents:
      (a) no robot approaching in the TARGET band within its stopping
          distance + its speed x my turn duration (+ a body margin);
      (b) no robot already inside the turn's crossing cells (hitbox
          occupancy, the same K_SIG*sigma rule as zone occupancy);
      (c) no same-direction follower close behind me in MY band;
      (d) static fit: a conservative swept DISC of HB_R_CIRC plus the sigma
          margin (hitbox.per_robot_inflation) must fit the grid's static
          clearance at every swept cell. (A tighter swept-arc primitive is
          the safety owner's; the disc is deliberately conservative.)
    Returns (ok, reason); reason is 'clear' or names the failed check and -
    when a robot caused it - that robot's id.
    """
    mx, my_y, _mth, msig = _xyts(my_pose)
    cells_list = [(int(r), int(c)) for r, c in (turn_cells or [])]
    if not cells_list:
        return False, "no turn cells"
    cell_set = set(cells_list)
    hb = _hitbox_mod()

    # (d) static fit first: no amount of waiting fixes geometry, and the
    # caller's fallback for it is 'drive on to where the turn fits'. Two
    # parts: every crossing cell must be DRIVABLE (the planner's lethal
    # band, UTURN_DRIVE_CLEAR_M - crossing laterally is ordinary driving),
    # and the PIVOT - the crossing's widest cell, where the rotation
    # physically happens - must fit the conservative swept DISC of
    # HB_R_CIRC plus my K_SIG/sqrt(2) sigma share (hitbox's per-robot sigma
    # term, without the pairwise base margin: walls are not another
    # uncertain robot). At the sigma floor the pivot needs 0.357 m; at the
    # forced-fallback sigma 0.3 it needs 0.71 m, which only the aisle
    # middles, junction boxes and turnarounds offer (~0.75 m) - a
    # degraded robot still turns at the engineered spots (far-side pickups
    # stay reachable; starving them was the measured idle-starvation
    # collapse) but can no longer swing beside a rack face.
    r_circ = getattr(hb, "HB_R_CIRC", HB_R_CIRC_FALLBACK) \
        if hb is not None else HB_R_CIRC_FALLBACK
    clean = getattr(hb, "clean_sigma", None) if hb is not None else None
    k_sig = getattr(hb, "K_SIG", K_SIG) if hb is not None else K_SIG
    sig = clean(msig) if callable(clean) else max(0.05, msig)
    need = r_circ + (k_sig / 1.4142136) * sig
    clearance = grid.clearance_m()
    widest, pivot = 0.0, cells_list[0]
    for (r, c) in cells_list:
        if not grid.is_static_free((r, c)):
            return False, f"static fit: ({r},{c}) is not free"
        clr = clearance[r][c]
        if clr < UTURN_DRIVE_CLEAR_M:
            return (False, f"static fit: clearance {clr:.2f}m < "
                           f"{UTURN_DRIVE_CLEAR_M:.2f}m at ({r},{c})")
        if clr > widest:
            widest, pivot = clr, (r, c)
    sweep = getattr(hb, "uturn_swept_cells", None) if hb is not None else None
    if callable(sweep):
        # SWEPT-RECT static fit (replaces the pivot-disc approximation):
        # the manoeuvre is modelled as the planner's rectilinear turn -
        # translate IN LANE to the pivot (the crossing's widest cell),
        # rotate IN PLACE there, translate out in the target lane - and
        # every rasterised cell the grown rect sweeps must be statically
        # free. extra_m = (sigma share)/sqrt(2) grows each half-extent so
        # the rect's rotated corner reach equals the old disc + margin
        # (hypot(hl+e, hw+e) ~= r_circ + sqrt(2)*e), i.e. the same sigma
        # law, but now exact about WHERE the body actually is during each
        # phase instead of one disc at the widest cell.
        th0 = math.atan2(*reversed(DIR_VECS[my_band[1]])) \
            if my_band is not None and my_band[1] in DIR_VECS else _mth
        th1 = math.atan2(*reversed(DIR_VECS[target_band[1]])) \
            if target_band is not None and target_band[1] in DIR_VECS \
            else th0 + math.pi
        extra = (need - r_circ) / 1.4142136
        sx, sy = grid.cell_to_world(cells_list[0])
        px_, py_ = grid.cell_to_world(pivot)
        ex, ey = grid.cell_to_world(cells_list[-1])
        swept = (sweep(grid, sx, sy, th0, px_, py_, th0, extra)
                 | sweep(grid, px_, py_, th0, px_, py_, th1, extra)
                 | sweep(grid, px_, py_, th1, ex, ey, th1, extra))
        hit_cell = next((c for c in sorted(swept)
                         if not grid.is_static_free(c)), None)
        if hit_cell is not None:
            return (False, f"static fit: swept rect (pivot {pivot}, "
                           f"clearance {widest:.2f}m) hits a wall/rack "
                           f"at {hit_cell}")
    elif widest < need:
        return (False, f"static fit: pivot clearance {widest:.2f}m < "
                       f"{need:.2f}m across the turn cells")

    tgt = (target_band[0], target_band[1]) if target_band is not None else None
    mine = (my_band[0], my_band[1]) if my_band is not None else None
    accel = getattr(hb, "A_MAX_MPS2", UTURN_ACCEL_MPS2) \
        if hb is not None else UTURN_ACCEL_MPS2
    pts = [grid.cell_to_world(c) for c in cells_list]

    states = peers.values() if hasattr(peers, "values") else peers
    for st in sorted(states, key=lambda s: getattr(s, "robot_id", "")):
        if not getattr(st, "alive", True):
            continue
        rid = getattr(st, "robot_id", "?")
        px, py, pth, psig = _xyts(st)
        v = abs(float(getattr(st, "v", 0.0) or 0.0))

        # (b) a robot already inside the turn's crossing cells.
        if occupant_cells(grid, (px, py, pth, psig)) & cell_set:
            return False, f"occupied: {rid} inside the turn cells"

        hit = band_at(grid, grid.world_to_cell(px, py))
        if hit is None:
            continue

        # (a) approaching traffic in the TARGET band.
        if tgt is not None and hit == tgt:
            dvx, dvy = DIR_VECS.get(hit[1], (0.0, 0.0))
            s_min = min((tx - px) * dvx + (ty - py) * dvy for tx, ty in pts)
            threat = (_uturn_stop_dist(v, accel) + v * UTURN_DURATION_S
                      + UTURN_BODY_M)
            if -UTURN_BODY_M < s_min <= threat:
                return (False, f"approach: {rid} {max(0.0, s_min):.2f}m from "
                               f"the turn in the target band "
                               f"(threat {threat:.2f}m)")
            if min(math.hypot(px - tx, py - ty) for tx, ty in pts) \
                    <= UTURN_BODY_M:
                return False, f"approach: {rid} beside the turn cells"

        # (c) same-direction follower close behind me in MY band.
        if mine is not None and hit == mine:
            dvx, dvy = DIR_VECS.get(hit[1], (0.0, 0.0))
            gap = (mx - px) * dvx + (my_y - py) * dvy
            if 0.0 < gap <= _uturn_stop_dist(v, accel) + UTURN_FOLLOWER_GAP_M:
                return (False,
                        f"follower: {rid} {gap:.2f}m behind me in my lane")
    return True, "clear"


# ====================================================== overtake gate (street)
# Street-style pass of a STATIONARY blocker (user report: "due to 1 robot
# other robots are stuck"): the robot directly behind a leader that has been
# immobile past the coordinator's T_OVERTAKE_S may BORROW the opposing lane
# alongside and past it - like passing a parked car - but only through THIS
# gate, evaluated every tick until the manoeuvre commits. Same deterministic
# broadcast inputs and duck-typing as uturn_gate.
OVERTAKE_DURATION_S = 6.0       # borrow-stretch transit bound (~2.5 m slow)
OVERTAKE_BODY_M = 0.6           # two circumscribed radii + grid slack


def overtake_gate(now: float, my_pose, my_band, borrow_cells, peers, grid,
                  blocker_id: str = "") -> Tuple[bool, str]:
    """May I borrow the opposing lane over `borrow_cells` right now?

    my_band:      (lane_id, direction) I travel in; the borrow cells are the
                  OPPOSING half of the same band alongside+past the blocker.
    borrow_cells: the opposing-half cells of the overtake window (from just
                  behind me to past the blocker + the merge-back run).
    blocker_id:   the stationary robot being passed - exempt from the
                  occupancy check (the planner routes around its inflated
                  body as a hard obstacle; the live envelope keeps the gap).

    Checks, most deterministic first:
      (s) static: every borrow cell drivable (UTURN_DRIVE_CLEAR_M) and the
          swept rect along the borrow stretch at my travel heading, grown by
          my sigma share (hitbox.uturn_swept_cells), hits nothing static;
      (b) nobody but the blocker already inside the borrow cells (hitbox
          occupancy, the zone-occupancy rule);
      (a) no robot APPROACHING in the opposing direction of my band within
          its stopping distance + its speed x OVERTAKE_DURATION_S + a body
          margin of the borrow stretch - the whole manoeuvre must fit before
          the earliest possible oncoming arrival, because stopping in the
          borrowed lane is forbidden (complete or retreat back).
    Returns (ok, reason) like uturn_gate."""
    mx, my_y, mth, msig = _xyts(my_pose)
    cells_list = [(int(r), int(c)) for r, c in (borrow_cells or [])]
    if not cells_list:
        return False, "no borrow cells"
    if my_band is None or my_band[1] not in DIR_VECS:
        return False, "no lane band under me"
    cell_set = set(cells_list)
    hb = _hitbox_mod()

    # (s) static fit.
    clearance = grid.clearance_m()
    for (r, c) in cells_list:
        if not grid.is_static_free((r, c)):
            return False, f"static fit: ({r},{c}) is not free"
        if clearance[r][c] < UTURN_DRIVE_CLEAR_M:
            return (False, f"static fit: clearance {clearance[r][c]:.2f}m < "
                           f"{UTURN_DRIVE_CLEAR_M:.2f}m at ({r},{c})")
    dvx, dvy = DIR_VECS[my_band[1]]
    sweep = getattr(hb, "uturn_swept_cells", None) if hb is not None else None
    if callable(sweep):
        clean = getattr(hb, "clean_sigma", None)
        k_sig = getattr(hb, "K_SIG", K_SIG)
        sig = clean(msig) if callable(clean) else max(0.05, msig)
        extra = (k_sig / 1.4142136) * sig / 1.4142136
        th = math.atan2(dvy, dvx)
        # Sweep along the borrow strip's LATERAL MIDLINE between its along-
        # extent endpoints (raw extreme cells can sit at the strip's edge
        # and swing the sweep line toward the racks).
        nvx, nvy = -dvy, dvx
        wpts = [grid.cell_to_world(c) for c in cells_list]
        alongs = [x * dvx + y * dvy for x, y in wpts]
        lat_mid = sum(x * nvx + y * nvy for x, y in wpts) / len(wpts)
        a0, a1 = min(alongs), max(alongs)
        swept = sweep(grid, a0 * dvx + lat_mid * nvx, a0 * dvy + lat_mid * nvy,
                      th, a1 * dvx + lat_mid * nvx, a1 * dvy + lat_mid * nvy,
                      th, extra)
        hit_cell = next((c for c in sorted(swept)
                         if not grid.is_static_free(c)), None)
        if hit_cell is not None:
            return (False, f"static fit: swept rect hits a wall/rack "
                           f"at {hit_cell}")

    opp = OPPOSITE_DIR[my_band[1]]
    ovx, ovy = DIR_VECS[opp]                    # the oncoming travel vector
    accel = getattr(hb, "A_MAX_MPS2", UTURN_ACCEL_MPS2) \
        if hb is not None else UTURN_ACCEL_MPS2
    pts = [grid.cell_to_world(c) for c in cells_list]
    states = peers.values() if hasattr(peers, "values") else peers
    for st in sorted(states, key=lambda s: getattr(s, "robot_id", "")):
        if not getattr(st, "alive", True):
            continue
        rid = getattr(st, "robot_id", "?")
        if rid == blocker_id:
            continue
        px, py, pth, psig = _xyts(st)
        v = abs(float(getattr(st, "v", 0.0) or 0.0))

        # (b) a body already inside the borrow cells.
        if occupant_cells(grid, (px, py, pth, psig)) & cell_set:
            return False, f"occupied: {rid} inside the borrow cells"

        hit = band_at(grid, grid.world_to_cell(px, py))
        # (a) oncoming traffic in the opposing half of MY band.
        if hit == (my_band[0], opp):
            s_min = min((tx - px) * ovx + (ty - py) * ovy for tx, ty in pts)
            threat = (_uturn_stop_dist(v, accel) + v * OVERTAKE_DURATION_S
                      + OVERTAKE_BODY_M)
            if -OVERTAKE_BODY_M < s_min <= threat:
                return (False, f"oncoming: {rid} {max(0.0, s_min):.2f}m from "
                               f"the borrow stretch (threat {threat:.2f}m)")
            if min(math.hypot(px - tx, py - ty) for tx, ty in pts) \
                    <= OVERTAKE_BODY_M:
                return False, f"oncoming: {rid} beside the borrow cells"
    return True, "clear"
