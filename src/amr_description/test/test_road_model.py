"""ROAD MODEL regression tests (webots_final4 fixes).

A corridor is a two-way keep-right street, never a lock:
  1. LANE PASS - two lane-keeping robots pass abreast in the 1.6 m spine;
     the envelope judges the pass on the LATERAL rect gap with the
     sigma-only threshold (safety.envelope_permit lane_pass).
  2. CONVOY ENTRY - a ranked corridor segment whose every occupant and
     claimant is a FRESH, MOVING, same-direction robot may be entered while
     my FIFO request is still queued (coordinator._convoy_zones); opposing,
     stopped or stale traffic keeps the strict capacity-1 mutex.
  3. DON'T BLOCK THE BOX - a junction (zone kind 'intersection') is entered
     only when the exit stretch beyond it is clear of stopped robots.
  4. Published road geometry (yaml traffic: road_lanes / junctions) matches
     the LaneBand and the junction zone rects - one source of truth.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import hitbox, safety, traffic              # noqa: E402
from amr_fleet.core.coordinator import FleetCoordinator          # noqa: E402
from amr_fleet.core.gridmap import GridMap                       # noqa: E402
from amr_fleet.core.models import Pose2D, RobotState             # noqa: E402
from amr_fleet.core.safety import (PeerDisc, SafetyMemory,       # noqa: E402
                                   envelope_permit)

YAML = str(pathlib.Path(__file__).resolve().parents[1]
           / "config" / "warehouse_grid.yaml")
GRID = GridMap.from_yaml(YAML)
# The convoy/box unit tests below exercise the RESERVATION machinery, which
# ships flag-disabled (road model): turn it on for this module's GRID copy.
GRID.traffic.reservations = True
NOW = 100.0
L1 = (88, 25)
NB_X, SB_X = 0.45, -0.45        # keep-right lane centres (yaml road_lanes)


def _me(x, y, th, v=1.0, sigma=0.05):
    st = RobotState(robot_id="robot_1", pose=Pose2D(x, y, th), v=v)
    st.loc_sigma_lat = sigma
    return st


def _disc(rid, x, y, th, v=1.0, sigma=0.05, **kw):
    return PeerDisc(robot_id=rid, x=x, y=y, theta=th, v=v,
                    rx_time=NOW, loc_sigma_lat=sigma, **kw)


def _path_north(x, y0, length=2.0, step=0.1):
    n = int(round(length / step))
    return [(x, y0 + i * step) for i in range(1, n + 1)]


# ================================================================ lane pass
def test_lateral_gap_is_the_projection_gap():
    a = (NB_X, 0.0, math.pi / 2)
    b = (SB_X, 1.0, -math.pi / 2)
    assert abs(hitbox.lateral_gap(a, b, math.pi / 2)
               - (0.9 - 2 * hitbox.HB_HALF_W)) < 1e-9
    # misaligned rects project wider: the gap shrinks (conservative)
    b2 = (SB_X, 1.0, -math.pi / 2 + 0.4)
    assert hitbox.lateral_gap(a, b2, math.pi / 2) < hitbox.lateral_gap(
        a, b, math.pi / 2)


def test_head_on_lane_pass_is_envelope_clear_only_with_lane_pass():
    """The measured corridor lock: head-on at the +-0.45 m lanes, closing
    2.0 m/s at cruise. Omnidirectional rule: STOP (v_close margin + sigma
    beats the 0.49 m lane gap). With lane_pass: clear - the lateral gap
    0.49 m beats the sigma-only threshold 0.12 + 2*sqrt(2)*0.05 = 0.26."""
    me = _me(NB_X, 0.0, math.pi / 2, v=1.0)
    peer = _disc("robot_2", SB_X, 2.5, -math.pi / 2, v=1.0)
    path = _path_north(NB_X, 0.0)
    # The envelope now grants the parallel-pass judgement to ANY peer driving
    # anti-parallel within PASS_HEADING_TOL_RAD (not only caller-attested
    # lane keepers), so the pass is clear with or without the hint.
    p = envelope_permit(me, path, [peer], SafetyMemory(), NOW)
    assert p is None, f"anti-parallel lane pass must be clear, got {p}"
    p2 = envelope_permit(me, path, [peer], SafetyMemory(), NOW,
                         lane_pass={"robot_2": math.pi / 2})
    assert p2 is None, f"lane pass must be clear, got {p2}"
    # ...but the SAME geometry with the peer drifting across the lanes
    # (heading 0.6 rad off-axis) is not a pass: the full rule stops it.
    cross = _disc("robot_2", SB_X, 1.4, -math.pi / 2 + 0.6, v=1.0)
    p3 = envelope_permit(me, path, [cross], SafetyMemory(), NOW)
    assert p3 is not None


def test_lane_pass_still_stops_on_high_sigma_or_lateral_drift():
    me = _me(NB_X, 0.0, math.pi / 2, v=1.0, sigma=0.15)
    peer = _disc("robot_2", SB_X, 2.5, -math.pi / 2, v=1.0, sigma=0.15)
    path = _path_north(NB_X, 0.0)
    # sigma 0.15 each: lateral threshold 0.12 + 2*sqrt(2*0.15^2) = 0.54
    # > 0.39 lane gap -> the FULL rule applies and stops the pass.
    p = envelope_permit(me, path, [peer], SafetyMemory(), NOW,
                        lane_pass={"robot_2": math.pi / 2})
    assert p is not None and p.action in ("STOP", "SLOW")
    # a peer drifting INTO my lane (predicted lateral closure) is not passed
    me2 = _me(NB_X, 0.0, math.pi / 2, v=1.0)
    drift = _disc("robot_2", SB_X, 1.4, -math.pi / 2 + 0.6, v=1.0)
    p3 = envelope_permit(me2, path, [drift], SafetyMemory(), NOW,
                         lane_pass={"robot_2": math.pi / 2})
    assert p3 is not None


def test_lane_pass_never_applies_to_stale_or_spinning_robots():
    me = _me(NB_X, 0.0, math.pi / 2, v=1.0)
    path = _path_north(NB_X, 0.0)
    ghost = _disc("robot_2", SB_X, 2.5, -math.pi / 2, v=1.0, silent=True)
    ghost.rx_time = NOW - 2.0
    spin = _disc("robot_3", SB_X, 1.2, -math.pi / 2, v=0.0, w=1.0)
    for peer in (ghost, spin):
        with_exc = envelope_permit(me, path, [peer], SafetyMemory(), NOW,
                                   lane_pass={peer.robot_id: math.pi / 2})
        without = envelope_permit(me, path, [peer], SafetyMemory(), NOW)
        assert (with_exc is None) == (without is None), (
            "lane_pass must be ignored for stale/spinning robots")


def test_lane_pass_eligibility_needs_lanes_mode_band_and_lane_keeping():
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(NB_X, 0.0, math.pi / 2)
    c.state.loc_sigma_lat = 0.05

    def peer(rid, x, y, th):
        st = RobotState(robot_id=rid, seq=1, stamp=NOW,
                        pose=Pose2D(x, y, th), v=1.0)
        st.loc_sigma_lat = 0.05
        c.on_peer_state(st, NOW)

    peer("robot_2", SB_X, 1.5, -math.pi / 2)      # opposite lane, lane-keeping
    peer("robot_3", -0.05, 2.5, -math.pi / 2)     # off its lane centre
    assert c._lane_pass_peers(NOW) is None, "SPINE mode: no lane pass"
    c._eff_mode = traffic.LANES_MODE
    out = c._lane_pass_peers(NOW) or {}
    assert "robot_2" in out and "robot_3" not in out
    # me off-lane: nobody is eligible
    c.state.pose = Pose2D(0.02, 0.0, math.pi / 2)
    assert c._lane_pass_peers(NOW) is None


# ============================================================= convoy entry
_SEQ = [0]


def _peer_state(rid, cell, th, v, zones=(), sigma=0.05):
    _SEQ[0] += 1
    st = RobotState(robot_id=rid, seq=_SEQ[0], stamp=NOW + 0.001 * _SEQ[0],
                    pose=Pose2D(*GRID.cell_to_world(cell), th), v=v)
    st.loc_sigma_lat = sigma
    if zones:
        from amr_fleet.core.models import Intent
        st.intent = Intent(cells=[cell], t_enter=[NOW], t_exit=[NOW + 5.0],
                           zones=list(zones))
        st.intent_seq = _SEQ[0]
    return st


def _northbound(cell=(25, 60)):
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world(cell), math.pi / 2)
    c.state.loc_sigma_lat = 0.05
    c.path = [(r, 60) for r in range(cell[0], 71)]
    c.goal_cell = (70, 60)
    return c


def test_convoy_zone_needs_moving_fresh_same_direction_traffic():
    c = _northbound()
    cur_i = 0
    # leader inside SPINE_S, moving north, claiming SPINE_S: convoy
    c.on_peer_state(_peer_state("robot_2", (35, 60), math.pi / 2, v=1.0,
                                zones=("SPINE_S",)), NOW)
    occ = traffic.occupants_by_zone(GRID, c._peer_positions(NOW))
    assert "robot_2" in occ.get("SPINE_S", set())
    assert "SPINE_S" in c._convoy_zones(c.path, cur_i, occ, NOW)
    # stopped leader: queue, not convoy
    c.on_peer_state(_peer_state("robot_2", (35, 60), math.pi / 2, v=0.0,
                                zones=("SPINE_S",)), NOW)
    assert "SPINE_S" not in c._convoy_zones(
        c.path, cur_i, traffic.occupants_by_zone(
            GRID, c._peer_positions(NOW)), NOW)
    # opposing leader: strict mutex
    c.on_peer_state(_peer_state("robot_2", (35, 60), -math.pi / 2, v=1.0,
                                zones=("SPINE_S",)), NOW)
    assert "SPINE_S" not in c._convoy_zones(
        c.path, cur_i, traffic.occupants_by_zone(
            GRID, c._peer_positions(NOW)), NOW)


def test_convoy_closes_when_an_opposing_claimant_queues():
    """Signal-style handover: one opposing REQUEST (intent claim) closes the
    convoy to new entrants even while the leader inside still moves north."""
    c = _northbound()
    c.on_peer_state(_peer_state("robot_2", (35, 60), math.pi / 2, v=1.0,
                                zones=("SPINE_S",)), NOW)
    c.on_peer_state(_peer_state("robot_3", (50, 60), -math.pi / 2, v=0.1,
                                zones=("SPINE_S", "SPINE_M")), NOW)
    occ = traffic.occupants_by_zone(GRID, c._peer_positions(NOW))
    assert "SPINE_S" not in c._convoy_zones(c.path, 0, occ, NOW)


def test_convoy_counts_as_acquired_only_with_a_queued_request():
    c = _northbound()
    c.on_peer_state(_peer_state("robot_2", (35, 60), math.pi / 2, v=1.0,
                                zones=("SPINE_S",)), NOW)
    occ = traffic.occupants_by_zone(GRID, c._peer_positions(NOW))
    convoy = c._convoy_zones(c.path, 0, occ, NOW)
    assert "SPINE_S" in convoy
    inside = set()
    _req, acq0, _ = c._traffic_status(c.path, 0, inside, occ, convoy, NOW)
    assert "SPINE_S" not in acq0, "no FIFO queue position yet"
    c.arbiter.state["SPINE_S"] = "REQUESTING"
    _req, acq1, _ = c._traffic_status(c.path, 0, inside, occ, convoy, NOW)
    assert "SPINE_S" in acq1, "queued + same-direction leader = follow it in"


def test_follower_gets_go_behind_moving_leader_in_segment():
    """End to end through _traffic_permit: the leader's claim/body in
    SPINE_S no longer pins the follower at the acquisition point."""
    c = _northbound()
    c.on_peer_state(_peer_state("robot_2", (35, 60), math.pi / 2, v=1.0,
                                zones=("SPINE_S",)), NOW)
    c.arbiter.state["SPINE_S"] = "REQUESTING"   # queued behind the leader
    p = c._traffic_permit(NOW)
    assert p is None or p.zone_id != "SPINE_S", (
        f"follower must not wait on the convoy segment: {p and p.reason}")


# ======================================================= don't-block-the-box
def test_junction_entry_waits_while_the_exit_is_blocked():
    """A2: the holder of J_N must NOT enter while a stopped robot blocks its
    exit stretch; it waits at the standoff, leaving the cross-flow free."""
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((80, 60)), math.pi / 2)
    c.state.loc_sigma_lat = 0.05
    # straight path north through J_N (rows 84-98), turning west on row 88
    c.path = ([(r, 60) for r in range(80, 89)]
              + [(88, col) for col in range(59, 24, -1)])
    c.goal_cell = (88, 25)
    blocker = _peer_state("robot_2", (88, 48), 0.0, v=0.0)   # beyond the exit
    c.on_peer_state(blocker, NOW)
    assert c._junction_exit_blocked("J_N", c.path, 0, NOW) == "robot_2"
    # a moving robot there is cross traffic, not a blocked box
    c.on_peer_state(_peer_state("robot_2", (88, 48), 0.0, v=0.8), NOW)
    assert c._junction_exit_blocked("J_N", c.path, 0, NOW) is None
    # non-junction zones are never box-checked
    assert c._junction_exit_blocked("SPINE_S", c.path, 0, NOW) is None


def test_blocked_exit_removes_the_junction_from_acquired():
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((80, 60)), math.pi / 2)
    c.state.loc_sigma_lat = 0.05
    c.path = ([(r, 60) for r in range(80, 89)]
              + [(88, col) for col in range(59, 24, -1)])
    c.goal_cell = (88, 25)
    c.arbiter.state["J_N"] = "HELD"
    c.on_peer_state(_peer_state("robot_2", (88, 48), 0.0, v=0.0), NOW)
    occ = traffic.occupants_by_zone(GRID, c._peer_positions(NOW))
    _req, acq, acq_i = c._traffic_status(c.path, 0, set(), occ, set(), NOW)
    assert "J_N" not in acq and acq_i is not None
    # exit clears -> the held junction is enterable again
    c.on_peer_state(_peer_state("robot_2", (47, 47), 0.0, v=0.0), NOW)
    occ = traffic.occupants_by_zone(GRID, c._peer_positions(NOW))
    _req, acq2, _ = c._traffic_status(c.path, 0, set(), occ, set(), NOW)
    assert "J_N" in acq2


# ========================================================== published roads
def test_road_geometry_matches_lane_band_and_junction_zones():
    t = GridMap.from_yaml(YAML).traffic     # fresh: module GRID flips flags
    lanes = {l["id"]: l for l in t.road_lanes}
    assert {"SPINE_NB", "SPINE_SB"} <= set(lanes)
    res, (ox, oy) = GRID.resolution, GRID.origin
    band = GRID.lanes
    nb_x = ox + (band.centre_pos + 0.5) * res
    sb_x = ox + (band.centre_neg + 0.5) * res
    assert all(abs(x - nb_x) < 1e-6 for x, _ in lanes["SPINE_NB"]["polyline"])
    assert all(abs(x - sb_x) < 1e-6 for x, _ in lanes["SPINE_SB"]["polyline"])
    # NB runs +y, SB runs -y (polylines in travel order)
    nb = lanes["SPINE_NB"]["polyline"]
    sb = lanes["SPINE_SB"]["polyline"]
    assert nb[-1][1] > nb[0][1] and sb[-1][1] < sb[0][1]
    assert lanes["SPINE_NB"]["direction"] == "NB"
    assert lanes["SPINE_SB"]["direction"] == "SB"
    # every lane id is a stable keep-right pair
    for lid in ("AISLE_A_EB", "AISLE_A_WB", "AISLE_B_EB", "AISLE_B_WB",
                "AISLE_N_EB", "AISLE_N_WB"):
        assert lid in lanes, lid
    # BOX junctions: the rectangular crossings of the road bands
    # (schema {id, center, half_size}); centred on the spine axis (x=0).
    juncs = {j["id"]: j for j in t.junctions}
    assert {"J_N", "J_AW"} <= set(juncs)
    for j in juncs.values():
        assert len(j["center"]) == 2 and len(j["half_size"]) == 2
        assert abs(j["center"][0]) < 1e-6, "crossings sit on the spine axis"
        assert 0.3 <= j["half_size"][0] <= 1.0
        assert 0.3 <= j["half_size"][1] <= 1.0
    assert abs(juncs["J_AW"]["center"][1] - 2.0) < 0.05   # aisle A centre
    assert abs(juncs["J_N"]["center"][1] - 4.15) < 0.05   # north aisle
    # no-reservations road model is the shipped default
    assert t.reservations is False
