"""Topological traffic layer, SPINE mode (Plan-P2P-Coordination increment 2).

Pins down wiki/Traffic-Lanes.md: the ranked zones of config/warehouse_grid.yaml,
the cells that must stay off every station route, the occupancy rule, the
acquisition point (wait bays) and that no request path can take zones out of
rank order (Havender ordering = no hold-and-wait cycle).
"""
import itertools
import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))   # fleet_sim

from amr_fleet.core import astar, traffic                       # noqa: E402
from amr_fleet.core import coordinator                            # noqa: E402
from amr_fleet.core.coordinator import FleetCoordinator          # noqa: E402
from amr_fleet.core.gridmap import GridMap                       # noqa: E402
from amr_fleet.core.models import Pose2D, RobotState             # noqa: E402
from amr_fleet.core.zone import T_ZONE_S, ZoneArbiter            # noqa: E402

GRID_PATH = (pathlib.Path(__file__).resolve().parents[1]
             / "config" / "warehouse_grid.yaml")
GRID = GridMap.from_yaml(str(GRID_PATH))
# The shipped road model runs with reservations OFF (yaml traffic.
# reservations: false). These tests pin the reservation machinery that
# still exists behind that flag, so enable it on this module's grid copy.
GRID.traffic.reservations = True
NOW = 100.0
L1, L2, L3 = (88, 25), (88, 94), (74, 25)     # slots 1R10, 3R11, 2R10
SPAWNS = {"S1": (10, 30), "S2": (10, 60), "S3": (10, 90)}
# R10: the whole-corridor SPINE mutex is re-cut into 3 junction-bounded
# segments; J_N/J_AW keep ranks ABOVE the segments.
EXPECTED_RANKS = {"SP_AW": 0, "SP_NW": 0, "SP_NE": 0,
                  "SPINE_S": 1, "SPINE_M": 2, "SPINE_N": 3,
                  "J_N": 4, "J_AW": 5}
SPINE_SEGMENTS = ("SPINE_S", "SPINE_M", "SPINE_N")


def _endpoints():
    pts = dict(SPAWNS)
    pts.update(GRID.stations)
    return pts


@pytest.fixture(scope="module")
def station_routes():
    pts = _endpoints()
    routes = {}
    for a, b in itertools.permutations(sorted(pts), 2):
        if pts[a] == pts[b] or (pts[b], pts[a]) in routes:
            continue
        path = astar.astar(GRID, pts[a], pts[b])
        assert path and path[-1] == pts[b], f"{a}->{b}"
        routes[(pts[a], pts[b])] = (a, b, path)
    return routes


def _rank_sorted(zones):
    keys = [traffic.rank_key(GRID, z) for z in zones]
    return keys == sorted(keys)


# ------------------------------------------------------------------ config
def test_zone_set_and_ranks_match_the_design():
    assert {z: GRID.zones[z].rank for z in GRID.zones} == EXPECTED_RANKS
    for old in ("choke_CP1", "aisle_A1", "aisle_A2", "inter_X1"):
        assert old not in GRID.zones
    assert all(z.capacity == 1 for z in GRID.zones.values())
    for seg in SPINE_SEGMENTS:
        assert GRID.zones[seg].modes == ("SPINE",)
    # The three segments tile the old SPINE rect [28,52,83,67] exactly and
    # disjointly, with boundaries at the aisle-B (row 40) and aisle-A
    # (row 62) crossings - each segment ends where a junction begins.
    assert not (GRID.zones["SPINE_S"].cells & GRID.zones["SPINE_M"].cells)
    assert not (GRID.zones["SPINE_M"].cells & GRID.zones["SPINE_N"].cells)
    assert GRID.zones["SPINE_S"].contains((39, 60))
    assert GRID.zones["SPINE_M"].contains((40, 60))   # aisle-B crossing
    assert GRID.zones["SPINE_M"].contains((61, 60))
    assert GRID.zones["SPINE_N"].contains((62, 60))   # aisle-A crossing
    assert GRID.zones["SPINE_N"].contains((83, 60))
    assert GRID.zones["J_N"].contains((84, 60))       # J_N right above
    union = (GRID.zones["SPINE_S"].cells | GRID.zones["SPINE_M"].cells
             | GRID.zones["SPINE_N"].cells)
    rows = {r for r, _ in union}
    cols = {c for _, c in union}
    assert (min(rows), min(cols), max(rows), max(cols)) == (28, 52, 83, 67)
    # R3/R7: LANES is REQUESTED; the SPINE mutex stays the fallback and the
    # effective mode comes from traffic.select_mode (gate tests below).
    assert GRID.traffic.mode == "LANES"
    assert set(GRID.traffic.refuges) == {"R_AE", "R_BW", "R_BE"}
    assert GRID.traffic.wait_bays == {"WB_W": (30, 35), "WB_E": (30, 85)}
    assert set(GRID.traffic.retreat_cells) == {(47, 47), (47, 72), (70, 73)}


def test_every_station_route_zones_come_out_rank_sorted(station_routes):
    for a, b, path in station_routes.values():
        req = traffic.required_zones(GRID, path)
        assert _rank_sorted(req), f"{a}->{b}: {req}"
        # Intent.zones is built from zones_on_path: ranked part in rank order.
        on_path = GRID.zones_on_path(path)
        assert on_path == req, f"{a}->{b}: {on_path}"


def test_coordinator_intent_zones_go_out_in_rank_order():
    for start, goal in (((10, 30), L1), ((22, 60), L2), (L1, (22, 30)),
                        (L3, (22, 90)), (L2, (22, 60))):
        c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
        c.state.pose = Pose2D(*GRID.cell_to_world(start), 0.0)
        assert c.set_goal(goal, now=NOW)
        assert c.intent.zones and _rank_sorted(c.intent.zones), c.intent.zones
        assert _rank_sorted(c.current_intent(NOW).zones)


def test_spawns_dropoffs_and_chg_cells_lie_in_no_ranked_zone():
    cells = dict(SPAWNS)
    cells.update({k: v for k, v in GRID.stations.items()
                  if k in ("P1", "P2", "P3")
                  or k.startswith(("DROPOFF", "CHG"))})
    assert len(cells) == 3 + 3 + 3 + 3
    for name, cell in cells.items():
        assert traffic.zones_at(GRID, cell) == [], name
        assert GRID.traffic.refuge_of(cell) is None, name
    # ...while every pickup is in exactly its spur.
    assert traffic.zones_at(GRID, L1) == ["SP_NW"]
    assert traffic.zones_at(GRID, L2) == ["SP_NE"]
    assert traffic.zones_at(GRID, L3) == ["SP_AW"]


def test_aisle_b_and_aisle_a_east_are_on_no_station_route(station_routes):
    for a, b, path in station_routes.values():
        for cell in path:
            assert GRID.traffic.refuge_of(cell) is None, (a, b, cell)


def test_bays_refuges_and_retreat_cells_are_parkable_and_off_routes(
        station_routes):
    clr = GRID.clearance_m()
    route_xy = [GRID.cell_to_world(c)
                for _a, _b, p in station_routes.values() for c in p]
    bays = set(GRID.traffic.wait_bays.values())
    named = list(bays) + GRID.traffic.retreat_cells
    for cell in named:
        assert GRID.is_static_free(cell), cell
        assert clr[cell[0]][cell[1]] >= 0.30, (cell, clr[cell[0]][cell[1]])
        assert traffic.zones_at(GRID, cell) == [], cell
        x, y = GRID.cell_to_world(cell)
        d = min(math.hypot(x - rx, y - ry) for rx, ry in route_xy)
        # R7: the south area between the bottom racks (y=-1.6) and the
        # dropoff row (y=-2.8) is 1.2 m deep, so a bay 0.35 m off the rack
        # face is 0.70 m from the dropoff-to-dropoff route (0.34 m body gap).
        assert d >= (0.65 if cell in bays else 0.85), (cell, round(d, 2))
    for cells, cost in GRID.traffic.refuges.values():
        assert cost == 1000.0
        assert all(GRID.is_static_free(c) for c in cells)
        assert not any(traffic.zones_at(GRID, c) for c in cells)


def test_refuges_cost_1000_unless_they_are_the_goal():
    # Into a refuge: allowed (goal), straight down its centre line.
    p = astar.astar(GRID, GRID.world_to_cell(-1.5, -0.2), (47, 10))
    assert p and p[-1] == (47, 10)
    # Through a refuge: never, even when it is the geometric shortcut.
    p = astar.astar(GRID, (70, 73), L2)
    assert p and not any(GRID.traffic.refuge_of(c) == "R_BE" for c in p)


# --------------------------------------------------------- traffic helpers
def test_acquisition_point_is_the_wait_bay_or_the_spur_mouth():
    # From the south: routed through the free wait bay, which is the point.
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((10, 30)), math.pi / 2)
    assert c.set_goal(L1, now=NOW)
    acq = traffic.acquisition_point(GRID, c.path)
    assert c.path[acq] == (30, 35)
    assert traffic.required_zones(GRID, c.path[:acq + 1]) == []
    # Leaving a spur: stand-off before the first zone after the spur.
    path = astar.astar(GRID, L1, (22, 30))
    acq = traffic.acquisition_point(GRID, path)
    k = traffic.first_new_zone_index(GRID, path)
    assert GRID.zones["J_N"].contains(path[k])
    assert traffic.zones_at(GRID, path[acq]) == ["SP_NW"]
    assert traffic.path_distance_m(GRID, path, acq, k) >= traffic.ACQ_STANDOFF_M
    # A path that enters no new ranked zone has none.
    assert traffic.acquisition_point(GRID, astar.astar(GRID, (22, 30),
                                                       (22, 90))) is None


def test_occupancy_makes_the_occupant_a_granter():
    x, y = GRID.cell_to_world((60, 60))
    occ = traffic.zone_occupants(GRID, "SPINE_M", {"robot_2": (x, y),
                                                   "robot_3": (-3.0, -4.0)})
    assert occ == {"robot_2"}
    assert traffic.required_granters(set(), occ) == {"robot_2"}


def test_request_guard_refuses_out_of_order():
    a = ZoneArbiter("robot_1", lambda d: None, lambda d: None)
    assert traffic.request(GRID, a, "SPINE_M", 0.5, 0, 1, NOW)
    assert not traffic.request(GRID, a, "SP_NW", 0.5, 0, 1, NOW)
    assert a.state.get("SP_NW") is None
    assert not traffic.request(GRID, a, "SPINE_S", 0.5, 0, 1, NOW)
    assert traffic.request(GRID, a, "SPINE_N", 0.5, 0, 1, NOW)
    assert traffic.request(GRID, a, "J_N", 0.5, 0, 1, NOW)


# ------------------------------------------------------ coordinator, live
def _peer(rid, cell, now, intent=None, status="IDLE", seq=1):
    x, y = GRID.cell_to_world(cell)
    st = RobotState(robot_id=rid, seq=seq, stamp=now, pose=Pose2D(x, y, 0.0),
                    status=status)
    if intent is not None:
        st.intent = intent
    return st


def _at_bay_heading_to_l1(sent):
    c = FleetCoordinator("robot_1", GRID, sent.append, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((30, 35)), 0.0)
    assert c.set_goal(L1, now=NOW)
    return c


def _auto_grant(c, sent, granter="robot_2", lc=None):
    """Answer every captured request with a grant from `granter` (merged
    spec 4: EVERY alive peer is a required granter of a ranked zone, so unit
    tests must answer; an idle peer grants at once in the real protocol)."""
    answered = 0
    for d in list(sent):
        if d.get("robot_id") != c.me:
            continue
        lamport = lc if lc is not None else int(d["lamport_ts"]) + 1
        c.on_zone_grant(dict(granter_id=granter, requester_id=c.me,
                             zone_id=d["zone_id"], req_id=d["req_id"],
                             granted=True, lamport_ts=lamport))
        answered += 1
    return answered


def test_peer_physically_inside_spine_blocks_entry_even_with_every_grant():
    """Occupancy rule: grants are necessary but NOT sufficient - a body
    physically inside the zone blocks entry until it leaves, even if its
    intent is empty and it granted everything (idle robots, stale intents)."""
    sent = []
    c = _at_bay_heading_to_l1(sent)
    t = NOW
    c.on_peer_state(_peer("robot_2", (60, 60), t, seq=1), t)
    assert not c.registry.peers["robot_2"].intent.zones   # no plan at all
    p = c.tick(t)
    assert p.action == "STOP" and p.zone_id == "SP_NW", p  # rank 0 first
    assert c.arbiter.state["SP_NW"] == "REQUESTING"
    # The idle occupant grants everything requested so far...
    _auto_grant(c, sent)
    t += 0.1
    c.on_peer_state(_peer("robot_2", (60, 60), t, seq=2), t)
    p = c.tick(t)
    _auto_grant(c, sent)
    t += 0.1
    c.on_peer_state(_peer("robot_2", (60, 60), t, seq=3), t)
    p = c.tick(t)
    _auto_grant(c, sent)
    t += 0.1
    c.on_peer_state(_peer("robot_2", (60, 60), t, seq=4), t)
    p = c.tick(t)
    # ...the first window (SP_NW + SPINE_S) is acquired - R10: the body
    # inside SPINE_M blocks ONLY that segment, so the robot may hold its
    # segment but can never pass the aisle-B junction while the occupant
    # is inside, however many grants arrive. J_N stays deferred (FREE).
    assert c.arbiter.state["SP_NW"] == "HELD"
    assert c.arbiter.state["SPINE_S"] == "HELD"
    assert p.zone_id == "SPINE_M", p
    assert p.action in ("STOP", "SLOW"), p
    assert c.arbiter.state.get("J_N", "FREE") == "FREE"
    # Occupant drives out of the spine and I drive on into my held segment:
    # the next window is acquired on the move, GO.
    c.state.pose = Pose2D(*GRID.cell_to_world((38, 60)), math.pi / 2)
    for k in (5, 6, 7):
        t += 0.1
        c.on_peer_state(_peer("robot_2", (20, 100), t, seq=k), t)
        p = c.tick(t)
        _auto_grant(c, sent)
    assert p.action in ("GO", "SLOW"), p
    assert {z for z, s in c.arbiter.state.items() if s == "HELD"} >= {
        "SP_NW", "SPINE_S", "SPINE_M"}


def test_robot_holding_zone_never_releases_it_while_inside():
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world(L1), 0.0)
    c.arbiter.state["SP_NW"] = "HELD"
    c.clear_goal()                       # e.g. task released at the pickup
    c.tick(NOW)
    assert c.arbiter.state["SP_NW"] == "HELD"
    assert "SP_NW" in c.current_intent(NOW).zones or not c.path


def test_wait_bay_detour_is_dropped_once_everything_is_acquired():
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((10, 30)), math.pi / 2)
    assert c.set_goal(L1, now=NOW)
    assert (30, 35) in c.path
    # Close to the bay: uncontested zones are acquired on the spot - but
    # only after the boot-race grace (an empty peer registry must not be an
    # instant quorum: measured 12.2 s cold-start double-hold).
    c.state.pose = Pose2D(*GRID.cell_to_world((28, 34)), math.pi / 2)
    p = c.tick(NOW)
    assert p.action == "STOP", "boot grace: no self-grant before 1st heartbeat"
    p = c.tick(NOW + coordinator.BOOT_QUORUM_GRACE_S + 0.1)
    assert p.action in ("GO", "SLOW")
    assert (30, 35) not in c.path, "no bay loop once the zones are held"
    # R10: only the first WINDOW must be held before moving on; segments
    # beyond the next junction are acquired later, at the junction standoff.
    assert all(c.arbiter.state.get(z) == "HELD"
               for z in traffic.zone_window(GRID, c.path))
    assert c.arbiter.state.get("SPINE_N", "FREE") == "FREE", (
        "the far segment must stay deferred, not locked end to end")
    assert c.arbiter.state.get("J_N", "FREE") == "FREE"


def test_path_progress_does_not_flip_onto_the_bay_loops_outbound_leg():
    """The wait-bay detour doubles back beside itself. A global nearest-
    index search flipped onto the outbound leg on 0.1 m of pose noise, which
    moved the acquisition point metres ahead and gave GO into a zone still
    being requested (fleet_sim seed 7, loc noise 0.1). Progress is tracked
    forward-only, like the follower's."""
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((22, 60)), math.pi / 2)
    assert c.set_goal(L1, now=NOW)
    b = c.path.index((30, 35))
    # A return-leg cell j whose 1-cell neighbour is an outbound cell i.
    j, i = next((j, i) for j in range(b + 4, len(c.path))
                for i in range(0, b - 3)
                if max(abs(c.path[i][0] - c.path[j][0]),
                       abs(c.path[i][1] - c.path[j][1])) == 1)
    seen = []
    for k in range(0, j + 1):                 # drive out to the bay and back
        c.state.pose = Pose2D(*GRID.cell_to_world(c.path[k]), 0.0)
        seen.append(c._path_index_near_current())
    assert seen == sorted(seen) and seen[-1] == j
    # Belief 0.1 m off, exactly on the outbound leg (where a global nearest
    # search lands): progress stays on the return leg.
    c.state.pose = Pose2D(*GRID.cell_to_world(c.path[i]), 0.0)
    assert c._path_index_near_current() >= b
    # A real jump (teleport back to the start) is still followed.
    c.state.pose = Pose2D(*GRID.cell_to_world(c.path[2]), 0.0)
    assert c._path_index_near_current() == 2


def test_occupied_wait_bay_is_not_chosen():
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((10, 30)), math.pi / 2)
    c.on_peer_state(_peer("robot_2", (30, 35), NOW, status="WAITING"), NOW)
    assert c.set_goal(L1, now=NOW)
    assert (30, 35) not in c.path


# ------------------------------------------------------- rank-order guard
class _RankWatch:
    """Wraps an arbiter's request(): records any request made while a
    higher-ranked zone is held or requested. The one tolerated exception
    (wiki/Traffic-Lanes.md) is a held zone the robot is physically inside:
    it cannot be given back; those are counted separately."""

    def __init__(self, coord):
        self.coord = coord
        self.violations = []
        self.exceptions = []
        self.requests = []
        inner = coord.arbiter.request

        def checked(zone_id, *a, **kw):
            committed = set(coord.arbiter.held_zones()) & set(
                coord._inside_zones())
            bad = traffic.out_of_order(coord.grid, zone_id,
                                       coord.arbiter.state, ignore=committed)
            self.requests.append(zone_id)
            if bad:
                self.violations.append((zone_id, bad))
            elif traffic.out_of_order(coord.grid, zone_id, coord.arbiter.state):
                self.exceptions.append(zone_id)
            return inner(zone_id, *a, **kw)

        coord.arbiter.request = checked


def test_rank_order_holds_on_every_request_path():
    sent = []
    c = _at_bay_heading_to_l1(sent)
    w = _RankWatch(c)
    t = NOW
    # 1. Contested acquisition: a peer that needs SPINE and never answers.
    #    Ranked zones never time out now - the same stamp is resent at 1 Hz
    #    and the FIFO queue position is kept (increment 3).
    blocker = astar.build_intent(GRID, astar.astar(GRID, (93, 60), (22, 60)),
                                 0.4, t)
    for k in range(int((T_ZONE_S + 3.0) / 0.1)):      # 2. far past old T_ZONE
        t = NOW + 0.1 * k
        c.on_peer_state(_peer("robot_2", (93, 60), t, intent=blocker,
                              status="MOVING", seq=k + 1), t)
        c.tick(t)
    assert c.arbiter.stats["timeouts"] == 0, "ranked zones must not time out"
    assert c.arbiter.stats.get("resends", 0) >= 1
    pending = [z for z, s in c.arbiter.state.items() if s == "REQUESTING"]
    assert pending, "still queued, same request"
    sent_reqs = [d for d in sent if d.get("zone_id") == pending[0]]
    assert len({d["req_id"] for d in sent_reqs}) == 1
    assert len({d["lamport_ts"] for d in sent_reqs}) == 1
    # 3. Path change: holding SPINE + SP_NW, the route now needs SP_AW (rank
    #    0): the higher zones are backed off before SP_AW is requested.
    c.arbiter.state.update(SPINE="HELD", J_N="HELD")
    c.set_goal(L3, now=t)
    for k in range(5):
        t += 0.1
        c.on_peer_state(_peer("robot_2", (10, 90), t, seq=1000 + k), t)
        c.tick(t)
    # 4. Deadlock retreat and resume: the retreat acquires nothing.
    v = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    v.state.pose = Pose2D(0.0, 2.0, math.pi / 2)
    assert v.set_goal(L1, now=NOW)
    for z in traffic.required_zones(GRID, v.path):
        v.arbiter.state[z] = "HELD"
    wv = _RankWatch(v)
    v.on_peer_state(_peer("robot_2", (80, 60), NOW, intent=blocker,
                          status="WAITING"), NOW)
    v._yield_permit({"cycle": ["robot_1", "robot_2"], "escalate": True}, NOW)
    assert v._retreat is not None
    for k in range(1, 60):
        v.tick(NOW + 0.5 * k)
    assert w.requests and not w.violations, w.violations
    assert not w.exceptions and not wv.violations and not wv.exceptions


def test_three_robot_sim_never_breaks_rank_order_or_mutual_exclusion():
    fs = pytest.importorskip("fleet_sim")
    sim = fs.FleetSim(fs.random_stream(seed=7))
    watches = [_RankWatch(a.coord) for a in sim.agents]
    m = sim.run(150.0)
    n_requests = sum(len(w.requests) for w in watches)
    if sim.agents[0].coord._reservations_on:
        # Reservation machinery enabled: requests happen and follow rank.
        assert n_requests > 0
        assert not [v for w in watches for v in w.violations]
    else:
        # Shipped ROAD MODEL (user requirement 2026-10-04): robots are free
        # to move - NO zone/path reservation anywhere, ever.
        assert n_requests == 0, f"road model made {n_requests} zone requests"
    assert m["arbiter_double_hold_ticks"] == {}
    # AC2: the capacity-1 zones stay exclusive in EVERY mode. SPINE is a
    # mutex only in SPINE mode - under LANES two robots in the band is the
    # whole point (two-abreast lanes) - so SPINE is judged only when some
    # robot ran the SPINE fallback.
    lanes_run = all(a.coord.traffic_mode_effective == "LANES"
                    for a in sim.agents)
    for zid, v in m["plan_zone_double_occupancy"].items():
        if zid.startswith("SPINE") and lanes_run:
            continue
        assert not v["events"], (zid, v)
    assert m["contacts"] == 0


# ------------------------------------------------- zone.py grant ordering
def test_peer_that_granted_my_request_is_ordered_after_me():
    """The double hold the simulator found: P grants my current request,
    then re-requests with a higher (aged) score; I must defer it."""
    grants = []
    me = ZoneArbiter("robot_3", lambda d: None, grants.append)
    me.request("SPINE", 0.2, 0.0, 1.0)
    me.on_grant(dict(granter_id="robot_1", requester_id="robot_3",
                     zone_id="SPINE", req_id=me.req_id["SPINE"],
                     granted=True, lamport_ts=5))
    me.on_request(dict(robot_id="robot_1", zone_id="SPINE", lamport_ts=9,
                       priority_score=0.9, req_id=77),
                  i_need_zone=True, relevant_peers={"robot_1", "robot_2"})
    assert not grants, "must defer, not grant back"
    me.release("SPINE")
    assert grants and grants[-1]["requester_id"] == "robot_1"
