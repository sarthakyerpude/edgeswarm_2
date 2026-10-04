"""Regressions for the two live Webots stalls (webots_final.log, 2026-10-04).

Incident A - corridor pin, 0 deliveries / 8 min, SPINE fallback (sigma 0.2):
  robot_2 stood mid-corridor 'at acquisition point: acquiring SPINE_N'
  (blocking robot_3) for hundreds of seconds while it actually HELD SPINE_N
  the whole time; robots 1/3 were stuck south with ~10 REPLAN FAILED whose
  failed victim reroutes restored their stale paths (and with them the stale
  SPINE claims). Mechanisms pinned here:
   1. entry-occupancy wedge: at sigma 0.2 the K_SIG*sigma occupancy
      inflation (0.4 m) makes a STOPPED neighbour whose centre is outside a
      segment an 'occupant' of it, vetoing the holder's entry forever
      -> inside-beats-stuck tiebreak (_entry_occupants, ACQ_INSIDE_TIEBREAK_S);
   2. K consecutive replan failures must drop the path AND the ranked claims
      the robot is not inside (replan() streak + _drop_stale_route_claims,
      and _yield_permit no longer restores the stale path past the streak);
   3. a REQUESTING ranked claim had no lease at all, so a pinned claimant
      deferred every later requester forever (request-lease back-off);
   4. hitbox.obstacle_cells grew by 0.10 + 2*sigma uncapped: at sigma 0.2
      that is r = 0.705 m - a 1.6 m aisle becomes unplannable past one
      parked robot (PLAN_SIGMA_INFL_CAP_M).

Incident B - make-way stall (sih-57): the yielder parked 'clear' by its own
  0.45 m pocket margin but inside the beneficiary's stationary-peer safety
  rule; the stale-beneficiary release is excluded when waiting_for == me, so
  all three robots froze until the 20 s MW_HOLD_MAX_S cap
  -> _mw_blocked_retarget (re-target farther within ~1.5 s, or end the hold).
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from amr_fleet.core import astar, hitbox, traffic
from amr_fleet.core.coordinator import (ACQ_INSIDE_TIEBREAK_S,
                                        MW_BLOCKED_RETARGET_S,
                                        REPLAN_FAIL_RELEASE_K,
                                        ZONE_HOLD_LEASE_S, FleetCoordinator)
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Intent, Pose2D, RobotState

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
# The shipped road model runs with reservations OFF (yaml traffic.
# reservations: false). These tests pin the reservation machinery that
# still exists behind that flag, so enable it on this module's grid copy.
GRID.traffic.reservations = True
NOW = 1000.0


def _coord(rid="robot_2", cell=(61, 60), theta=math.pi / 2, sigma=0.2):
    sent = {"requests": [], "grants": []}
    c = FleetCoordinator(rid, GRID,
                         lambda d: sent["requests"].append(d),
                         lambda d: sent["grants"].append(d))
    c.state.pose = Pose2D(*GRID.cell_to_world(cell), theta)
    c.state.loc_sigma_lat = sigma
    c._sent = sent
    return c


def _rs(rid, cell=None, xy=None, theta=math.pi / 2, status="WAITING",
        waiting_for="", v=0.0, sigma=0.2, zones=(), score=0.0):
    x, y = GRID.cell_to_world(cell) if cell is not None else xy
    st = RobotState(robot_id=rid, seq=1, stamp=NOW, pose=Pose2D(x, y, theta))
    st.status = status
    st.waiting_for = waiting_for
    st.priority_score = score
    st.v = v
    st.loc_sigma_lat = sigma
    st.intent = Intent()
    st.intent.zones = list(zones)
    return st


# ------------------------------------------------- 1. entry-occupancy wedge
def test_inside_beats_stuck_unwedges_holder_at_acquisition_point():
    """The holder of SPINE_N, stopped at its acquisition point past the
    grace, must stop counting a STOPPED neighbour whose centre is outside
    SPINE_N (only its sigma-inflated hitbox pokes in) as an occupant."""
    c = _coord("robot_2", cell=(61, 60), sigma=0.2)   # last SPINE_M row
    c.goal_cell = (70, 60)                            # inside SPINE_N
    assert c.replan(now=NOW), "sanity: route north must plan"
    # robot_3 jammed 0.5 m behind: centre in SPINE_M, inflated into SPINE_N.
    peer = _rs("robot_3", cell=(56, 60), v=0.0, sigma=0.25,
               zones=("SPINE_M", "SPINE_N"))
    assert c.registry.update(peer, now=NOW)
    occ = traffic.occupants_by_zone(GRID, c._peer_positions(NOW))
    assert "robot_3" in occ.get("SPINE_N", set()), (
        "sanity: the wedge geometry must rasterise robot_3 into SPINE_N")
    for z in ("SPINE_M", "SPINE_N"):
        c.arbiter.state[z] = "HELD"                   # grants already in
    # Within the grace the veto stands (flowing traffic untouched).
    c._acq_wait_since = NOW - 1.0
    p = c._traffic_permit(NOW)
    assert p is not None and p.action == "STOP"
    assert p.zone_id == "SPINE_N" and p.blocking_robot == "robot_3"
    # Past the grace the inside-beats-stuck tiebreak unwedges the holder.
    c._acq_wait_since = NOW - ACQ_INSIDE_TIEBREAK_S - 0.1
    assert c._traffic_permit(NOW) is None, (
        "holder at the acquisition point must win against a stopped, "
        "centre-outside claimant")
    assert "SPINE_N" in c._tiebreak_zones
    # Sticky: the first GO resets the wait clock; the tiebreak must not
    # re-wedge one tick later.
    c._acq_wait_since = None
    assert c._traffic_permit(NOW + 0.1) is None
    # Control 1: a MOVING occupant that is NOT my same-direction convoy
    # (here: opposing heading) keeps the full inflated veto. (A moving
    # SAME-direction occupant is now a convoy leader - test_road_model.)
    peer_moving = _rs("robot_3", cell=(56, 60), v=0.3, sigma=0.25,
                      theta=-math.pi / 2,
                      zones=("SPINE_M", "SPINE_N"), status="MOVING")
    peer_moving.seq = 2
    assert c.registry.update(peer_moving, now=NOW + 0.2)
    c._acq_wait_since = NOW - ACQ_INSIDE_TIEBREAK_S - 0.1
    c._tiebreak_zones = set()
    p = c._traffic_permit(NOW + 0.2)
    assert p is not None and p.action == "STOP"
    # Control 2: an occupant whose CENTRE is inside the zone keeps the veto.
    peer_in = _rs("robot_3", cell=(64, 60), v=0.0, sigma=0.05,
                  zones=("SPINE_N",))
    peer_in.seq = 3
    assert c.registry.update(peer_in, now=NOW + 0.4)
    c._tiebreak_zones = set()
    p = c._traffic_permit(NOW + 0.4)
    assert p is not None and p.action == "STOP"


# ------------------------------- 2. replan-failure streak drops stale claims
def test_replan_fail_streak_drops_path_and_claims_not_inside():
    """After REPLAN_FAIL_RELEASE_K straight failures the robot keeps the
    empty path and releases every claim it is not physically inside; the
    HELD segment under its body is kept (traffic rule 4)."""
    c = _coord("robot_3", cell=(34, 60))              # inside SPINE_S
    c.arbiter.state["SPINE_S"] = "HELD"               # entered, my body in it
    c.arbiter.state["SPINE_M"] = "REQUESTING"         # stale route claims
    c.arbiter.state["SPINE_N"] = "REQUESTING"
    c.goal_cell = (88, 94)
    # Seal the robot in: a ring of blocked cells beyond SELF_CLEARANCE_M.
    ring = set()
    for dr in range(-9, 10):
        for dc in range(-9, 10):
            if 6 <= max(abs(dr), abs(dc)) <= 9:
                ring.add((34 + dr, 60 + dc))
    for k in range(REPLAN_FAIL_RELEASE_K - 1):
        assert not c.replan(now=NOW, extra_blocked=ring)
        assert c.arbiter.state["SPINE_N"] == "REQUESTING", (
            "claims survive the first K-1 failures")
    assert not c.replan(now=NOW, extra_blocked=ring)   # K-th failure
    assert c.path == []
    assert c.arbiter.state.get("SPINE_M") == "FREE"
    assert c.arbiter.state.get("SPINE_N") == "FREE"
    assert c.arbiter.state.get("SPINE_S") == "HELD", (
        "the segment my hitbox is inside is never released")
    # A later successful replan resets the streak.
    assert c.replan(now=NOW)
    assert c._replan_fail_streak == 0


# ----------------------------------- 3. REQUESTING-claim lease (FIFO unpin)
def test_requesting_claim_lease_flushes_grant_to_inside_needer():
    """A ranked claim stuck REQUESTING past ZONE_HOLD_LEASE_S, while a peer
    that needs it stands physically inside a zone I claim, is backed off -
    the release flushes my deferred queue, handing the inside robot its
    grant (the webots pin: robot_2 deferred behind stuck claimants)."""
    c = _coord("robot_3", cell=(34, 60))              # inside SPINE_S
    c.goal_cell = (88, 94)
    assert c.replan(now=NOW)                          # real route north
    # Stuck claims, requested 13 s ago (ranked: resend forever, no timeout).
    for z in ("SPINE_M", "SPINE_N"):
        c.arbiter.request(z, 0.0, NOW, NOW + 30.0,
                          now=NOW - ZONE_HOLD_LEASE_S - 1.0, ranked=True)
    # robot_2 is INSIDE SPINE_M, stopped, and needs SPINE_N: it asked after
    # me, so my on_request defers it (pure FIFO).
    peer = _rs("robot_2", cell=(50, 60), v=0.0, status="WAITING",
               waiting_for="robot_3", zones=("SPINE_M", "SPINE_N"))
    assert c.registry.update(peer, now=NOW)
    c.on_zone_request(dict(robot_id="robot_2", zone_id="SPINE_N",
                           lamport_ts=9999, req_id=77, priority_score=0.0))
    assert ("robot_2", 77) in c.arbiter.deferred.get("SPINE_N", []), (
        "sanity: FIFO must defer the later requester")
    c.tick(NOW)
    flushed = [g for g in c._sent["grants"]
               if g["zone_id"] == "SPINE_N" and g["requester_id"] == "robot_2"
               and g.get("granted")]
    assert flushed, ("the request lease must back the stuck claim off and "
                     "flush the deferred grant to the robot inside")


# --------------------------------------- 4. planning inflation cap (sigma)
def test_planning_obstacle_inflation_is_capped_and_aisle_stays_plannable():
    """At sigma 0.2 the uncapped 0.10 + 2*sigma rect walled off the whole
    1.6 m spine for A*. Capped, a parked robot leaves a plannable thread."""
    x, y = GRID.cell_to_world((50, 60))               # parked mid-spine
    blocked = hitbox.obstacle_cells(GRID, x, y, math.pi / 2, 0.2)
    # The sigma term saturates at PLAN_SIGMA_INFL_CAP_M.
    assert blocked == hitbox.obstacle_cells(GRID, x, y, math.pi / 2, 1.0)
    half = (hitbox.HB_HALF_W + 0.10 + hitbox.PLAN_SIGMA_INFL_CAP_M
            + GRID.resolution)                        # + half-diag slack
    assert all(abs(cc - 60) * GRID.resolution <= half + 1e-9
               for _, cc in blocked), "lateral planning extent must be capped"
    # The corridor must remain plannable straight past the parked robot.
    path = astar.astar(GRID, (45, 54), (55, 54), blocked=blocked)
    assert path, "a single parked robot must not make the aisle unplannable"


# ----------------------------------------- 5. make-way blocked-hold retarget
def test_makeway_hold_retargets_or_ends_when_beneficiary_blocked_by_me():
    """sih-57 stall: the yielder's HOLD must end (re-target or resume) within
    ~MW_BLOCKED_RETARGET_S once the beneficiary is stationary with
    waiting_for == me - never freeze to the 20 s MW_HOLD_MAX_S cap."""
    cell = (47, 72)                                   # a designated retreat
    c = _coord("robot_3", cell=cell, sigma=0.1)
    # Beneficiary parked 0.5 m away, stopped BY me (its stationary rule).
    bx, by = GRID.cell_to_world(cell)
    ben = _rs("robot_1", xy=(bx, by - 0.5), v=0.0, sigma=0.1,
              status="WAITING", waiting_for="robot_3", score=3000.0)
    assert c.registry.update(ben, now=NOW)
    c._retreat = dict(cell=cell, saved_goal=None, blockers=["robot_1"],
                      phase="HOLD", t0=NOW - 3.0, hold_t0=NOW - 2.0,
                      make_way=True, beneficiary="robot_1", b_still=None,
                      route_zones=[])
    unblocked_at = None
    t = NOW
    while t < NOW + 5.0:
        ben.stamp = ben.rx_time = t
        ben.seq += 1
        c.registry.update(ben, now=t)
        c.tick(t)
        r = c._retreat
        if r is None or r.get("phase") == "DRIVE" or r.get("cell") != cell:
            unblocked_at = t - NOW
            break
        t += 0.5
    assert unblocked_at is not None, (
        "the make-way hold froze for 5 s although the beneficiary was "
        "stationary and waiting on me (pre-fix: 20 s MW_HOLD_MAX_S)")
    assert unblocked_at <= MW_BLOCKED_RETARGET_S + 1.5


# --------------------------------------- 6. full-sim wedge: deliveries > 0
def test_spine_wedge_fleet_delivers_within_240s():
    """The distilled live incident in the real fleet sim: a compressed
    northbound queue at fallback sigma 0.25, leader at the SPINE_N boundary,
    follower rasterised into SPINE_N. Pre-fix this wedged at 0-1 deliveries
    with REPLAN-FAILED pinning; the fleet must now deliver."""
    import fleet_sim as fs
    import _repro_pin as rp
    m = fs.run(rp.spine_wedge(), 240.0)
    assert m["deliveries"] >= 1, (
        f"fleet stayed wedged: {m['deliveries']} deliveries, "
        f"max_wait={m['max_wait_s']}")
    # No robot may sit pinned for the whole run.
    assert max(m["max_wait_s"].values()) < 180.0
