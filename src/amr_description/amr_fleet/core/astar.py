"""
A* on the warehouse occupancy grid, plus space-time intent generation.

SCOPE - read this before assuming this file plans the robot's motion.

This planner exists so the coordination layer can produce a SPACE-TIME INTENT
to broadcast, and so the deadlock handler can evaluate "is there an alternative
route?". It does NOT command the robot. Motion control belongs to
amr_navigation / Nav2, which owns cmd_vel. We publish a MotionPermit and, if
the navigation team wants it, a suggested path on <ns>/coordination_path.

Keeping these separate means this package never fights Nav2 for the actuator.

WHY PLAIN A* AND NOT D* LITE
On the aligned 12m x 10m grid at 0.1m resolution, A* operates over roughly 8k free cells;
which is about 2-6 ms on a laptop and 4-8 ms on a Jetson Nano. Replans happen a
few times per minute. D* Lite's incremental advantage only pays off on much
larger maps replanned at high rate, and it costs ~250 lines of subtle code.
Measure before optimising.

AISLE CENTRING
Nav2 follows this path verbatim, so plain shortest-path A* used to hug rack
corners and walls. Every step now also pays GridMap.clearance_cost() (a cached
per-cell array, no per-call distance transform): zero on the local aisle
centre line, rising toward racks/walls, and prohibitive where the chassis does
not fit. extra_cost (congestion, zone penalties) is added on top unchanged.
"""
import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from .gridmap import CLEARANCE_LETHAL_COST, NEAR_WALL_KEEP_M, lane_step_cost
from .hitbox import HB_CHASSIS_HALF_W
from .models import Cell, Intent, Pose2D

# HARD feasibility: a cell whose centre is closer to a wall/rack than the
# bare chassis half-width can never be driven, whatever the soft costs say.
# This closes the measured leak where blocking the spine made 26/55 routes
# thread the 0.1 m rack-end strip at cost 1000/cell.
# The gate is applied with half a cell of tolerance (clearance is sampled at
# the CELL CENTRE: a 0.15 m-centre cell on a 0.1 m grid still contains
# feasible ground up to 0.20 m out, and rejecting it strands goals one cell
# off a face on the legacy map). The sub-0.11 m strip cells stay hard-walled.
FREE_MIN_CLEARANCE_M = HB_CHASSIS_HALF_W      # 0.16 m

# Merge AFTER moving off: within this many cells of the START, being OFF
# CENTRE in my own lane is free (the off-centre pull fades in) while
# CHANGING lanes costs up to (1 + LANE_START_LATERAL_BOOST)x. A robot
# replanned off its lane first keeps driving the way it faces, then merges
# on the move. Without it the lane pull demanded an immediate diagonal
# swerve from standstill - a turn whose swept corners a robot queued 0.1 m
# behind vetoed, so neither robot could ever move (_repro_pin.spine_wedge).
#
# STRICT LANES: the taper NEVER applies to the wrong-way term
# (gridmap.LaneBand.step_cost charges LANE_WRONG_SIDE_HARD at full strength
# from the very first step, crossing zones excepted). The old fully-tapered
# start let a robot roll up to 8 cells AGAINST the oncoming lane before its
# U-turn; now it turns in place / merges laterally instead - it can start
# off-lane, but never against the flow.
LANE_START_TAPER_CELLS = 8.0
LANE_START_LATERAL_BOOST = 3.0

# 8-connected with true diagonal cost.
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, 1.4142136), (-1, 1, 1.4142136),
              (1, -1, 1.4142136), (1, 1, 1.4142136)]

# ------------------------------------------------- turn shaping (road model)
# Corner & rotation fixes (2026-10-04, webots_strictlanes1): RPP arcs and
# pivots around every planned corner, so a corner PLANNED on a lane line
# 0.35 m from a wall/rack face physically swings the body into it (robot_1
# wedged nose-to-wall after the NB->WB left turn at J_N; robot_2's front
# corner 4 cm from rack 4 while merging). Two soft terms, applied only in
# the ROAD MODEL (reservations off - legacy fixtures pin their old routes):
#
# 1. TURN SHAPING: a step that changes direction by >= 90 deg pays
#    TURN_SHAPE_COST * severity * (1 - clearance/TURN_CLEAR_M) at the corner
#    cell. The 0.286 m rotation disc + the sigma share + RPP overshoot needs
#    ~0.45-0.55 m, so corners move off the 0.35 m lane lines into junction-
#    box middles (~0.75 m clear) - the path through a junction becomes
#    lane -> box-centre pivot -> lane. 45 deg steps pay HALF severity (two
#    45s one cell apart are the same physical swing as one 90), so the
#    square corner cannot dodge the cost by decomposing into diagonals.
# 2. FORWARD HAZARD: a step whose heading points at a static obstacle within
#    FWD_HAZARD_M (0.45 m collision-monitor StopZone + 0.20 m nose) pays
#    FWD_HAZARD_COST * (1 - free/FWD_HAZARD_M): the collision monitor WILL
#    stop any forward command there, so the planner must not command it.
#    This is what keeps NB travel off the last rows before the north wall.
#
# Both terms fade with the goal taper (station/dock goals legitimately sit
# 0.4-0.45 m from faces) and stay far below LANE_WRONG_SIDE_HARD, so lane
# direction always dominates route choice.
# >= 90 deg corners (the rectilinear model's in-place SPINS): sized at the
# fleet's measured-worst localization design point sigma = 0.30 (live AMCL
# error reached 0.28 m): spin disc 0.286 + sqrt(2)*0.30 = 0.71. Junction-box
# middles offer ~0.75 m, so full spins land there; dead-end turnaround
# pivots (~0.55-0.65) keep a small permanent deficit - an unavoidable
# constant per U-turn trip, while the runtime uturn_gate still scales with
# the LIVE sigma (floor need 0.357) and relocates the turn when degraded.
TURN_CLEAR_M = 0.71
# A single 45 deg step only swings the nose by the spin disc + sigma
# (~0.36 m): it needs less room than a full corner, and charging it more
# than the ENGINEERED 0.35 m lane-line clearance made every merge onto a
# lane line more expensive than riding one row off centre (measured: the
# dock1->L1 WB run sat on row 93 instead of the lane line; the L1->dock1 EB
# run on row 88 instead of 87). 0.35 = the fleet-wide minimum face
# clearance: 45-deg merges there are the baseline design, driven slowly by
# the rectilinear controller profile; only genuinely tighter ground pays.
TURN_CLEAR_45_M = 0.35
TURN_SHAPE_COST = 400.0
FWD_HAZARD_M = 0.65
FWD_HAZARD_COST = 300.0


def _turn_severity(pdr: int, pdc: int, dr: int, dc: int) -> float:
    """0 straight, 0.5 per 45 deg step, 1.0 at 90, 1.5 at 135, 2.0 at 180."""
    num = pdr * dr + pdc * dc
    den = math.sqrt((pdr * pdr + pdc * pdc) * (dr * dr + dc * dc))
    cos = num / den if den else 1.0
    if cos >= 0.9:
        return 0.0
    if cos >= 0.5:
        return 0.5
    if cos >= -0.5:
        return 1.0
    if cos >= -0.95:
        return 1.5
    return 2.0


def octile(a: Cell, b: Cell) -> float:
    """Admissible heuristic for 8-connected grids with sqrt(2) diagonals."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (1.4142136 - 2.0) * min(dx, dy)


def astar(gridmap, start: Cell, goal: Cell,
          blocked: Optional[Set[Cell]] = None,
          extra_cost: Optional[Dict[Cell, float]] = None,
          max_expansions: int = 20000,
          clearance: bool = True,
          goal_taper_cells: float = 5.0,
          mode: Optional[str] = None,
          crossing_extra: Optional[Set[Cell]] = None) -> List[Cell]:
    """Cheapest path start -> goal, or [] if none exists.

    blocked    : hard dynamic obstacles (from MapUpdate merging; peer-robot
                 obstacles should come from hitbox.obstacle_cells, the
                 rasterised wheel-inclusive rect, not a pose-centred disc)
    extra_cost : SOFT penalties per cell. This is where congestion awareness
                 enters: adding ~0.4 per cell a peer plans to occupy soon makes
                 robots spread across aisles BEFORE any explicit negotiation is
                 needed. A robot that never enters the busy aisle never has to
                 argue about it, and that is a large part of the throughput
                 gain over stop-and-wait.
    max_expansions : safety valve so a pathological map cannot hang the tick.
    clearance  : add the static aisle-centring cost (see module docstring).
    goal_taper_cells : the centring cost fades linearly to 0 within this many
                 cells of the goal. Pickup/drop-off markers sit right beside
                 racks; without the taper the path would wander around the
                 goal before committing to the final approach.
    mode       : legacy grids (traffic.reservations True): pass 'LANES' to
                 apply gridmap.lanes.step_cost inside the spine band; any
                 other value leaves routing as before. ROAD MODEL grids
                 (reservations False, the shipped config): every lane band
                 applies in EVERY mode - the SPINE fallback stays in lane
                 too (strict directional lanes; no mode may route into the
                 opposite lane outside gridmap.in_crossing_zone cells).
                 Inside a band the centring pull is suppressed (the lane
                 centre replaces it) but the LETHAL term and the near-wall
                 (< NEAR_WALL_KEEP_M) term are KEPT - without them the
                 cheap lane corner cuts the rack (measured).

    Cells with static clearance below FREE_MIN_CLEARANCE_M (chassis
    half-width) are rejected as WALLS, not merely expensive.
    """
    blocked = blocked or set()
    extra_cost = extra_cost or {}
    ccost = None
    if clearance and hasattr(gridmap, "clearance_cost"):
        ccost = gridmap.clearance_cost()
    clr = gridmap.clearance_m() if hasattr(gridmap, "clearance_m") else None
    clr_gate = FREE_MIN_CLEARANCE_M - 0.5 * gridmap.resolution
    # Keep-right lanes. Legacy (reservations on): the spine band only, and
    # only in effective LANES mode. ROAD MODEL (reservations off, the
    # shipped config): EVERY band, in EVERY mode - opposite flows get
    # separate in/out paths through every aisle and the spine and only meet
    # at a junction box or turnaround (user order: "use the dedicated lanes
    # for the direction it is designed and not the other lane"). The SPINE
    # fallback (sigma >= 0.30 sustained) used to drop to the shared centre
    # line = BOTH lanes; it now STAYS IN LANE at reduced effectiveness - at
    # that sigma the envelope needs ~0.97 m, more than the 0.9 m lane
    # spacing, so an opposing pass stops on the envelope and the right-of-
    # way machinery resolves it, but NO mode may ever route a robot into
    # the opposite lane. mode None (estimates, bay legs, webapp checks)
    # plans lanes too.
    traffic_cfg = getattr(gridmap, "traffic", None)
    road_model = (traffic_cfg is not None
                  and not getattr(traffic_cfg, "reservations", True))
    spine_band = getattr(gridmap, "lanes", None)
    if road_model:
        bands = list(getattr(gridmap, "lane_bands", None) or
                     ([spine_band] if spine_band is not None else []))
    elif mode == "LANES" and spine_band is not None:
        bands = [spine_band]
    else:
        bands = []
    lanes = bands or None
    # Crossing zones (junction boxes + yaml turnarounds): the only cells
    # where a wrong-way lane step is plannable (at the soft cost). Hand-
    # built grids without the accessor have none.
    crossing_fn = getattr(gridmap, "crossing_cells", None)
    crossing: Set[Cell] = crossing_fn() if (lanes is not None
                                            and callable(crossing_fn)) \
        else set()
    # Overtake borrow window (coordinator._maybe_overtake): cells where the
    # wrong-way term is soft FOR THIS PLAN ONLY, so a gated overtake can
    # route through the opposing lane past a stationary blocker. Everywhere
    # else the strict-lanes law is unchanged.
    if crossing_extra:
        crossing = crossing | {(int(r), int(c)) for r, c in crossing_extra}
    # Turn shaping + forward hazard (road model only; legacy fixture grids
    # pin their old route shapes). See the constants' comment block.
    shape = (road_model and clr is not None
             and callable(getattr(gridmap, "forward_clearance", None)))
    fwd_maps: Dict[Tuple[int, int], List[List[float]]] = {}
    # Traffic refuges (aisle B, aisle A east) are closed to through-routing:
    # every refuge cell costs its yaml 'cost' unless the goal lies in that
    # same refuge. Leaving a refuge one starts in pays the same toll on every
    # exit, so it simply takes the shortest way out.
    traffic = getattr(gridmap, "traffic", None)
    rcost = traffic.refuge_cost if traffic is not None else None
    goal_refuge = traffic.refuge_of(goal) if rcost else None

    def free(c: Cell) -> bool:
        """Traversable. The exact start and goal are exempt from the
        clearance gate: a robot already in (or sent to) a tight cell must
        still be able to plan - what the gate forbids is routing THROUGH
        cells the chassis does not fit (the rack-end strip leak)."""
        if not gridmap.is_static_free(c) or c in blocked:
            return False
        if c == start or c == goal:
            return True
        return clr is None or clr[c[0]][c[1]] >= clr_gate

    if not free(start) or not free(goal):
        return []
    if start == goal:
        return [start]

    open_heap: List[Tuple[float, float, Cell]] = [(octile(start, goal), 0.0, start)]
    came: Dict[Cell, Cell] = {}
    g: Dict[Cell, float] = {start: 0.0}
    closed: Set[Cell] = set()
    expansions = 0

    while open_heap:
        _, gc, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expansions += 1
        if expansions > max_expansions:
            return []

        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]

        for dr, dc, step in NEIGHBOURS:
            nxt = (cur[0] + dr, cur[1] + dc)
            if nxt in closed or not free(nxt):
                continue
            # Forbid corner cutting. Without this the path slips diagonally
            # between two rack corners and the real robot clips a rack leg.
            if dr and dc:
                if not free((cur[0] + dr, cur[1])) or not free((cur[0], cur[1] + dc)):
                    continue
            h = octile(nxt, goal)
            ng = gc + step + extra_cost.get(nxt, 0.0)
            if rcost:
                rp = rcost.get(nxt)
                if rp is not None and rp[0] != goal_refuge:
                    ng += rp[1]
            if ccost is not None:
                cc = ccost[nxt[0]][nxt[1]]
                if (lanes is not None and cc
                        and cc < CLEARANCE_LETHAL_COST
                        and any(b.in_band(nxt[0], nxt[1]) for b in lanes)
                        and (clr is None
                             or clr[nxt[0]][nxt[1]] >= NEAR_WALL_KEEP_M)):
                    cc = 0.0        # the lane centre replaces aisle centring
                if cc and h < goal_taper_cells:
                    cc *= h / goal_taper_cells
                ng += cc
            if shape:
                # Both terms fade with the goal taper, like the centring
                # cost: stations/docks legitimately sit near faces.
                taper = (h / goal_taper_cells if h < goal_taper_cells
                         else 1.0)
                # STRAIGHT steps only: a sustained drive AT a nearby face is
                # always a straight run (F1: NB at the north wall), while a
                # diagonal step is the planner's one-cell merge idiom whose
                # real heading the follower smooths - charging it priced
                # every merge onto a rack-side lane line at ~39 and pushed
                # whole runs one row off the engineered lanes (measured:
                # L1->dock1 rode row 88 over the 87 lane line).
                if not (dr and dc):
                    fm = fwd_maps.get((dr, dc))
                    if fm is None:
                        fm = gridmap.forward_clearance(dr, dc)
                        fwd_maps[(dr, dc)] = fm
                    free_ahead = fm[nxt[0]][nxt[1]]
                    if free_ahead < FWD_HAZARD_M:
                        ng += (FWD_HAZARD_COST * taper
                               * (1.0 - free_ahead / FWD_HAZARD_M))
                prev = came.get(cur)
                if prev is not None:
                    sev = _turn_severity(cur[0] - prev[0], cur[1] - prev[1],
                                         dr, dc)
                    if sev:
                        thr = TURN_CLEAR_M if sev >= 1.0 else TURN_CLEAR_45_M
                        cm = clr[cur[0]][cur[1]]
                        if cm < thr:
                            ng += (TURN_SHAPE_COST * sev * taper
                                   * (1.0 - cm / thr))
            if lanes is not None:
                frac = min(1.0, octile(start, nxt) / LANE_START_TAPER_CELLS)
                ng += lane_step_cost(
                    lanes, cur[0], cur[1], nxt[0], nxt[1], along_scale=frac,
                    lateral_scale=1.0 + LANE_START_LATERAL_BOOST * (1.0 - frac),
                    crossing=nxt in crossing)
            if ng < g.get(nxt, float("inf")):
                g[nxt] = ng
                came[nxt] = cur
                heapq.heappush(open_heap, (ng + h, ng, nxt))
    return []


def path_length_m(gridmap, path: List[Cell]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += gridmap.cell_distance_m(a, b)
    return total


def congestion_costs(peer_intents, now: float, horizon: float = 10.0,
                     penalty: float = 0.4) -> Dict[Cell, float]:
    """Soft cost from peers' broadcast plans. Feeds astar(extra_cost=...).

    A cell counts only while its [t_enter, t_exit] window overlaps
    [now, now + horizon]; cells the peer has already passed cost nothing.
    Peer windows must already be on the local clock (conversions.py does
    this on receipt).
    """
    out: Dict[Cell, float] = {}
    for intent in peer_intents:
        for cell, t_in, t_out in intent.triples():
            if t_out >= now and t_in <= now + horizon:
                out[cell] = out.get(cell, 0.0) + penalty
    return out


def build_intent(gridmap, path: List[Cell], v_nominal: float, now: float,
                 turn_penalty_s: float = 0.5,
                 safety_margin_s: float = 0.5,
                 goal: Optional[Pose2D] = None) -> Intent:
    """Convert a geometric path into a time-stamped occupancy claim.

    safety_margin_s widens every cell window at BOTH ends. It absorbs clock
    skew between robots, control lag, and prediction error. Start at 0.5 s.
    If you observe near-misses, raise it; if throughput suffers badly, lower it
    and measure. Tune it empirically and REPORT the tuning - that is exactly
    the kind of detail that distinguishes a serious project.
    """
    intent = Intent(goal=goal or Pose2D())
    if not path:
        return intent

    t = now
    cell_time = gridmap.resolution / max(0.05, v_nominal)

    for i, cell in enumerate(path):
        travel = cell_time
        # Turning costs real time and is the main source of ETA error.
        if 0 < i < len(path) - 1:
            a, b, c = path[i - 1], path[i], path[i + 1]
            if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                travel += turn_penalty_s
        intent.cells.append(cell)
        intent.t_enter.append(t - safety_margin_s)
        intent.t_exit.append(t + travel + safety_margin_s)
        t += travel

    intent.zones = gridmap.zones_on_path(path)
    intent.active_zone_index = 0 if intent.zones else -1
    return intent
