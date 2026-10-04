"""Strict directional lanes: behavioural acceptance scenarios (VALIDATION).

Written by the validation agent in parallel with the LANES planner
(gridmap.lane_band_at / in_crossing_zone, lethal wrong-way A* costs,
yaml traffic.turnarounds) and the coordinator (traffic.uturn_gate,
coordinator.uturn_active, own-lane retreats, no wait-edges for
opposing-band peers). Every test gates on the APIs it needs and SKIPS
(never fails) until they land.

Ground truth only: all lane-discipline acceptance is judged on TRUE poses
via the fleet_sim LaneAudit metrics (wrong_lane_ticks / wrong_lane_events /
gated_uturn_count / uturn_wait_ticks) and direct pose recording, never on
believed poses.
"""
import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fleet_sim as fs  # noqa: E402  (also puts the package root on sys.path)

from amr_fleet.core.models import Task  # noqa: E402
from amr_fleet.core import gridmap as gridmap_mod  # noqa: E402
from amr_fleet.core import traffic as traffic_mod  # noqa: E402
from amr_fleet.core.gridmap import GridMap  # noqa: E402
from amr_fleet.core.astar import astar  # noqa: E402


def _grid():
    try:
        return GridMap.from_yaml(str(fs.GRID_YAML))
    except Exception:
        return None


GRID = _grid()
PLANNER_API = (GRID is not None
               and hasattr(GRID, "lane_band_at")
               and hasattr(GRID, "in_crossing_zone")
               and bool(getattr(GRID, "turnaround_rects", []))
               and hasattr(gridmap_mod, "LANE_WRONG_SIDE_HARD"))
COORD_API = hasattr(traffic_mod, "uturn_gate")

needs_planner = pytest.mark.skipif(
    not PLANNER_API,
    reason="gridmap lane_band_at/in_crossing_zone/turnarounds/hard wrong-way"
           " cost not landed yet (LANES planner)")
needs_coord = pytest.mark.skipif(
    not COORD_API,
    reason="traffic.uturn_gate not landed yet (LANES coordinator)")

SPINE = (25, 52, 83, 67)                 # lane band rect incl. south throat
DIVIDER = 59.5                           # spine NB/SB divider column


def _run(sim, duration_s, tick=None, until=None):
    """Step the sim with an optional per-tick callback and early exit."""
    n = int(round(duration_s / fs.DT))
    for _ in range(n):
        sim.step()
        if tick is not None:
            tick(sim)
        if until is not None and until(sim):
            break
    return sim.metrics(sim.t)


def _skip_unless_uturn_flag(sim):
    if not all(hasattr(a.coord, "uturn_active") for a in sim.agents):
        pytest.skip("coordinator.uturn_active flag not landed yet"
                    " (LANES coordinator)")


def _in_spine(cell):
    r0, c0, r1, c1 = SPINE
    return r0 <= cell[0] <= r1 and c0 <= cell[1] <= c1


def _wrong_way_steps(grid, path):
    """Planning-level audit: steps of `path` that travel WITH the flow of
    the opposite lane outside a crossing zone (must be impossible once the
    wrong-way cost is lethal)."""
    bad = []
    for a, b in zip(path, path[1:]):
        if grid.in_crossing_zone(*b):
            continue
        res = grid.lane_band_at(*b)
        if not res:
            continue
        _, d = res
        along = (b[0] - a[0]) if d in ("NB", "SB") else (b[1] - a[1])
        if along == 0:
            continue                      # lateral merge step: legal
        if (along > 0) != (d in ("NB", "EB")):
            bad.append((a, b, d))
    return bad


# ----------------------------------------------------------- planning ------
@needs_planner
def test_station_routes_never_run_against_the_flow():
    """Lethal wrong-way costs: no station-to-station A* route ever travels
    along the opposing lane outside a junction box / turnaround."""
    places = [fs.L1, fs.L2, fs.L3, fs.D1, fs.D2, fs.D3,
              GRID.world_to_cell(-3.0, -4.0), GRID.world_to_cell(3.0, -4.0)]
    for s in places:
        for g in places:
            if s == g:
                continue
            p = astar(GRID, s, g)
            assert p, (s, g, "must plan")
            bad = _wrong_way_steps(GRID, p)
            assert not bad, (s, g, bad[:3])


@needs_planner
def test_fallback_replan_stays_in_lane_when_own_lane_blocked():
    """In-lane fallback: a northbound plan from deep inside the NB lane
    stays off the SB flow even when the direct in-lane line is unattractive
    (the planner may cross at boxes/turnarounds, never run the gauntlet)."""
    for start, goal in [((26, 64), (83, 64)), ((83, 55), (26, 55)),
                        ((58, 64), fs.D1), ((58, 55), fs.L2)]:
        p = astar(GRID, start, goal)
        assert p, (start, goal)
        assert not _wrong_way_steps(GRID, p), (start, goal)


# ----------------------------------------------------- head-on spine pass --
@needs_planner
def test_head_on_spine_pass_zero_wrong_lane():
    """Two robots meet head-on mid-spine: they pass in their own lanes with
    zero wrong-lane occupancy and no contact."""
    sim = fs.FleetSim(fs.head_on(warmup_s=5.0))
    m = _run(sim, 240.0,
             until=lambda s: sum(a.deliveries for a in s.agents) >= 2)
    assert m["deliveries"] == 2, m["deliveries_per_robot"]
    assert m["contacts"] == 0
    assert m["wrong_lane_ticks"] == 0, m["wrong_lane_samples"][:5]
    assert m["lane_passes"] >= 1


@needs_planner
def test_single_trip_zero_wrong_lane():
    m = fs.run(fs.single_robot(), 200.0)
    assert m["deliveries"] == 1
    assert m["wrong_lane_ticks"] == 0, m["wrong_lane_samples"][:5]


# ------------------------------------------------------- far-side pickup ---
@needs_planner
def test_far_side_pickup_delivers_with_zero_wrong_lane():
    """A west-side robot with an east/north pickup must cross the spine via
    a legal crossing (junction box or turnaround), never against the flow."""
    x, y = fs._cell_xy(fs.D1)
    robots = [fs.RobotSpec("robot_1", x, y, 0.0, dock=False,
                           tasks=[(0.5, Task("FS-1", pickup=fs.L2,
                                             dropoff=fs.D1))])]
    sim = fs.FleetSim(fs.Scenario(name="far_side", robots=robots, seed=0))
    m = _run(sim, 300.0, until=lambda s: s.agents[0].deliveries >= 1)
    assert m["deliveries"] == 1
    assert m["wrong_lane_ticks"] == 0, m["wrong_lane_samples"][:5]
    assert m["contacts"] == 0


# ------------------------------------------------------------ U-turn gate --
@needs_planner
@needs_coord
def test_uturn_waits_for_oncoming_then_turns():
    """A NB robot that must reverse direction holds its lane while an
    oncoming SB robot is in the target lane, and only crosses the divider
    once that robot is clear (no early line-cross; the turn completes)."""
    x1, y1 = fs._cell_xy((60, 64))       # NB lane, south of the aisle-A box
    x2, y2 = fs._cell_xy((82, 55))       # SB lane, north of it, inbound
    robots = [
        fs.RobotSpec("robot_1", x1, y1, math.pi / 2, dock=False,
                     tasks=[(0.0, Task("UT-1", pickup=fs.D1,
                                       dropoff=fs.L1))]),
        fs.RobotSpec("robot_2", x2, y2, -math.pi / 2, dock=False,
                     tasks=[(0.0, Task("UT-2", pickup=fs.D2,
                                       dropoff=fs.D3))]),
    ]
    sim = fs.FleetSim(fs.Scenario(name="uturn_gate", robots=robots, seed=0))
    _skip_unless_uturn_flag(sim)
    grid = sim.agents[0].grid
    rec = []

    def tick(s):
        a, b = s.by_id["robot_1"], s.by_id["robot_2"]
        rec.append((s.t, grid.world_to_cell(a.x, a.y), a.y, a.th, a.v,
                    grid.world_to_cell(b.x, b.y), b.y))

    m = _run(sim, 240.0, tick=tick,
             until=lambda s: s.by_id["robot_1"].pickups >= 1)
    assert sim.by_id["robot_1"].pickups >= 1, "robot_1 never got south"
    assert m["contacts"] == 0
    assert m["wrong_lane_ticks"] == 0, m["wrong_lane_samples"][:5]
    early, first_sb = [], None
    for t, ca, ay, ath, av, cb, by in rec:
        a_sb = _in_spine(ca) and ca[1] <= DIVIDER
        if a_sb and first_sb is None:
            first_sb = (t, ca)
        oncoming = (_in_spine(cb) and cb[1] <= DIVIDER
                    and by > ay and (by - ay) < 2.0)
        if a_sb and oncoming and not grid.in_crossing_zone(*ca):
            early.append((t, ca, cb))
    assert not early, early[:5]
    assert first_sb is not None, "the turn never completed"
    done = [t for t, ca, ay, ath, av, cb, by in rec
            if _in_spine(ca) and ca[1] <= DIVIDER
            and math.sin(ath) < -0.5 and av > 0.05]
    assert done, "robot_1 never travelled south in the SB lane"
    assert (m["gated_uturn_count"] >= 1
            or grid.in_crossing_zone(*first_sb[1])), \
        ("divider crossed outside a crossing zone with no gated U-turn",
         first_sb)


# ------------------------------------------- own-lane retreat (sigma 0.3) --
_RETREAT_START = ("retreating to", "make-way: clearing")
_RETREAT_END = ("retreat finished", "make-way: done", "retreat: done",
                "make-way failed", "make-way: no target")


def _retreat_windows(logs, pad_s=2.0):
    """{rid: [(t0, t1)]} retreat/make-way episodes from the sim log."""
    open_t, out = {}, {}
    for t, rid, msg in logs:
        if any(k in msg for k in _RETREAT_START) and rid not in open_t:
            open_t[rid] = t
        elif any(k in msg for k in _RETREAT_END) and rid in open_t:
            out.setdefault(rid, []).append((open_t.pop(rid), t + pad_s))
    for rid, t0 in open_t.items():
        out.setdefault(rid, []).append((t0, float("inf")))
    return out


@needs_planner
@needs_coord
def test_own_lane_retreat_keeps_out_of_opposing_band():
    """Degraded sigma head-on: the give-way robot backs up IN ITS OWN LANE.
    While travelling/reversing along the band during a retreat episode it
    never occupies the opposing half (lateral pull-overs are exempt)."""
    sc = fs.head_on(warmup_s=5.0, force_sigma_lat=0.3, keep_log=True)
    sim = fs.FleetSim(sc)
    _skip_unless_uturn_flag(sim)
    grid = sim.agents[0].grid
    rec = []

    def tick(s):
        rec.append((s.t, {a.id: (grid.world_to_cell(a.x, a.y), a.th,
                                 bool(getattr(a.coord, "reverse_active",
                                              False)))
                          for a in s.agents}))

    m = _run(sim, 300.0, tick=tick,
             until=lambda s: sum(a.deliveries for a in s.agents) >= 2)
    assert m["contacts"] == 0
    windows = _retreat_windows(sim.logs)
    if not windows:
        pytest.skip("no retreat/make-way episode occurred in this run")
    bad = []
    for rid, spans in windows.items():
        for t, cells in rec:
            if not any(t0 <= t <= t1 for t0, t1 in spans):
                continue
            cell, th, rev = cells[rid]
            if not _in_spine(cell) or grid.in_crossing_zone(*cell):
                continue
            s = math.sin(th)
            if abs(s) < 0.5:
                continue                  # lateral pull-over: exempt
            own_pos = s > 0               # facing north -> NB (east) half
            in_pos = cell[1] > DIVIDER
            if own_pos != in_pos:
                # Rule 6: REVERSING along the own lane is legal, and the
                # coordinator flags it (reverse_active, the audit contract).
                # Facing against the half's direction WITH the flag set is
                # exactly that own-lane reverse (a retreat driving back down
                # its own half; the sim follower executes it as turn-and-
                # drive); without the flag it is wrong-way occupancy.
                if rev:
                    continue
                bad.append((rid, round(t, 1), cell))
    assert not bad, bad[:8]


@needs_planner
@needs_coord
def test_degraded_sigma_fallback_stays_in_lane():
    """force_sigma_lat 0.3 (the old SPINE single-file fallback input): the
    robots still hold lane discipline - zero wrong-lane occupancy."""
    m = fs.run(fs.head_on(warmup_s=5.0, force_sigma_lat=0.3), 300.0)
    assert m["deliveries"] == 2, m["deliveries_per_robot"]
    assert m["contacts"] == 0
    assert m["wrong_lane_ticks"] == 0, m["wrong_lane_samples"][:5]


# ------------------------------------------- reverse_active exemption ------
@needs_planner
def test_reverse_active_exempts_only_the_own_half():
    """coordinator.reverse_active (own-lane reverse retreat) exempts the
    wrong-lane audit ONLY inside the half the manoeuvre started in (or
    off-road): it can never whitewash true wrong-way driving in the
    opposing half."""
    sim = fs.FleetSim(fs.single_robot())
    a = sim.agents[0]
    g = a.grid
    own, opposing = (44, 20), (51, 20)   # aisle B: EB (south) / WB (north)
    assert g.lane_band_at(*own) and g.lane_band_at(*own)[1] == "EB"
    assert g.lane_band_at(*opposing) and g.lane_band_at(*opposing)[1] == "WB"

    def place(cell, th, v=0.3):
        a.x, a.y = g.cell_to_world(cell)
        a.th, a.v = th, v

    def sampled():
        before = sim.wrong_lane_ticks
        sim._sample(sim.t)
        return sim.wrong_lane_ticks - before

    # control: forward westbound in the EB half is a violation un-flagged
    a.coord.reverse_active = False
    place(own, math.pi)
    assert sampled() == 1
    # rising edge inside the own half latches it: exempt there
    a.coord.reverse_active = True
    place(own, math.pi)
    assert sampled() == 0
    # still flagged but in the OPPOSING half driving against its flow:
    # reverse_active must NOT whitewash it
    place(opposing, 0.0)
    assert sampled() == 1
    # back in the latched own half: exempt again
    place(own, math.pi)
    assert sampled() == 0
    # flag dropped: the own-half wrong-way tick counts once more
    a.coord.reverse_active = False
    place(own, math.pi)
    assert sampled() == 1


# ------------------------------------- stationary opposing-band peer -------
@needs_planner
@needs_coord
def test_stationary_opposing_peer_is_not_waited_on():
    """A robot parked in the OPPOSING lane is not traffic: passing it must
    create no waiting_for edge and no deadlock participation."""
    px, py = fs._cell_xy((58, 55))       # SB half, strict mid segment
    sx, sy = fs._cell_xy((30, 64))       # NB lane, south throat
    robots = [
        fs.RobotSpec("robot_1", sx, sy, math.pi / 2, dock=False,
                     tasks=[(0.5, Task("OP-1", pickup=fs.L2,
                                       dropoff=fs.D3))]),
        fs.RobotSpec("robot_2", px, py, -math.pi / 2, dock=False),
    ]
    sim = fs.FleetSim(fs.Scenario(name="opposing_parked", robots=robots,
                                  seed=0))
    waited = []

    def tick(s):
        if s.by_id["robot_1"].coord.state.waiting_for == "robot_2":
            waited.append(round(s.t, 1))

    m = _run(sim, 240.0, tick=tick,
             until=lambda s: s.by_id["robot_1"].pickups >= 1)
    assert sim.by_id["robot_1"].pickups >= 1, \
        "robot_1 never passed the parked opposing-band peer"
    assert not waited, waited[:10]
    assert not m["deadlock_events"], m["deadlock_events"]
    assert m["contacts"] == 0
    assert m["wrong_lane_ticks"] == 0, m["wrong_lane_samples"][:5]
