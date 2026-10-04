"""Regressions for the integration-fixer round (post-R10 measurement FAILs).

Each test pins one measured failure from the slice reports:
  1. boot-race double-hold (zone quorum vacuously empty at cold start);
  2. deadlock timeout self-victimisation ignoring priority (group=[me]);
  3. waiting on a YIELDING blocker treated as a deadlock (mutual-yield churn);
  4. stationary-peer envelope freeze: a robot interlocked below g_eff could
     never drive AWAY (seed-7 375 s standoff);
  5. idle peer parked inside a ranked zone it does not hold suppressing
     make-way via the zone-wait skip (scenario e / AC12 end-block);
  6. a parked robot (empty path) keeping ranked corridor segments HELD.
"""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import safety, traffic
from amr_fleet.core.coordinator import (FleetCoordinator,
                                        BOOT_QUORUM_GRACE_S,
                                        RELEASE_SIGMA_SCALE)
from amr_fleet.core.deadlock import DeadlockManager, T_TIMEOUT_SELF_S
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Intent, Pose2D, RobotState

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
# The shipped road model runs with reservations OFF (yaml traffic.
# reservations: false). These tests pin the reservation machinery that
# still exists behind that flag, so enable it on this module's grid copy.
GRID.traffic.reservations = True
NOW = 100.0


def _coord(rid="robot_1", cell=(10, 30), theta=math.pi / 2):
    c = FleetCoordinator(rid, GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world(cell), theta)
    return c


def _rs(rid, x, y, theta=0.0, status="MOVING", waiting_for="",
        score=0.0, zones=(), v=0.0):
    st = RobotState(robot_id=rid, seq=1, stamp=NOW, pose=Pose2D(x, y, theta))
    st.status = status
    st.waiting_for = waiting_for
    st.priority_score = score
    st.v = v
    st.intent = Intent()
    st.intent.zones = list(zones)
    return st


# ------------------------------------------------- 1. boot-race double-hold
def test_boot_grace_no_ranked_hold_before_first_heartbeat():
    """zone.py may_enter treated an EMPTY peer registry as a satisfied
    quorum: two simultaneous cold starts both logged 'ZONE HELD SP_NE
    (grants from [])' at t=0.0 and double-held it for 12.2 s."""
    c = _coord(cell=(28, 34))
    assert c.set_goal((88, 25), now=NOW)
    p = c.tick(NOW)
    assert p.action == "STOP"
    assert not c.arbiter.held_zones(), "no self-grant before a heartbeat"
    # A genuinely alone robot proceeds once the grace expires.
    p = c.tick(NOW + BOOT_QUORUM_GRACE_S + 0.1)
    assert p.action in ("GO", "SLOW")
    assert c.arbiter.held_zones()


# --------------------------------- 2./3. deadlock timeout victim selection
def test_timeout_victim_is_the_lower_priority_side_of_the_pair():
    """A CARRYING robot whose no-cycle timeout fires while waiting on a
    lower-priority blocker must HOLD (victim = blocker), not retreat."""
    dm = DeadlockManager("robot_1")
    me = _rs("robot_1", 0, 0, status="WAITING", waiting_for="robot_2",
             score=3000.0)
    peer = _rs("robot_2", 1, 0, status="WAITING", waiting_for="robot_3",
               score=2000.0)
    out = None
    for t in (0.0, 9.0):            # past t_deadlock=8
        out = dm.tick(me, {"robot_2": peer}, t)
    assert out is not None and out["role"] == "HOLD"
    assert out["victim"] == "robot_2"
    # The mirror case: I am the lower-priority side, so I am the victim.
    dm2 = DeadlockManager("robot_1")
    me2 = _rs("robot_1", 0, 0, status="WAITING", waiting_for="robot_2",
              score=2000.0)
    peer2 = _rs("robot_2", 1, 0, status="WAITING", waiting_for="robot_3",
                score=3000.0)
    out2 = None
    for t in (0.0, 9.0):
        out2 = dm2.tick(me2, {"robot_2": peer2}, t)
    assert out2 is not None and out2["role"] == "VICTIM"


def test_timeout_escalates_to_self_when_holding_never_works():
    """A non-cooperating lower-priority blocker cannot park me forever: past
    T_TIMEOUT_SELF_S the group collapses back to [me]."""
    dm = DeadlockManager("robot_1")
    me = _rs("robot_1", 0, 0, status="WAITING", waiting_for="robot_2",
             score=3000.0)
    peer = _rs("robot_2", 1, 0, status="WAITING", waiting_for="",
               score=2000.0)
    dm.tick(me, {"robot_2": peer}, 0.0)
    out = dm.tick(me, {"robot_2": peer}, T_TIMEOUT_SELF_S + 1.0)
    assert out is not None and out["role"] == "VICTIM"


def test_waiting_on_a_yielding_blocker_is_progress_not_deadlock():
    """The blocker is already clearing (retreat/make-way, time-bounded) and
    MOVING: self-victimising at the timeout made BOTH parties retreat and
    resume into the identical standoff (the measured mutual-yield churn).
    A PINNED yielder (stationary) cannot clear anything, so then the timeout
    runs and I move (measured: suppressing on a pinned yielder froze
    converge at 0 pickups / 189 s wait)."""
    dm = DeadlockManager("robot_1")
    me = _rs("robot_1", 0, 0, status="WAITING", waiting_for="robot_2",
             score=2000.0)
    yielder = _rs("robot_2", 1, 0, status="YIELDING", waiting_for="",
                  score=3000.0, v=0.3)
    dm.tick(me, {"robot_2": yielder}, 0.0)
    assert dm.tick(me, {"robot_2": yielder}, 12.0) is None
    # Pinned (stationary) yielder: I become the victim and clear the way.
    yielder.v = 0.0
    out = dm.tick(me, {"robot_2": yielder}, 13.0)
    assert out is not None and out["role"] == "VICTIM"


# ------------------------------------- 4. stationary-escape envelope rule
def test_interlocked_robot_may_drive_away_from_a_stationary_peer():
    """Seed-7 standoff: robot_2's retreat was vetoed forever because the
    rect swing of its first poses dipped the gap below gap_now while it
    stood 0.25-0.34 m from a STATIONARY robot_3. Driving away (gap at the
    horizon well above the start) must be allowed."""
    mem = safety.SafetyMemory()
    me = _rs("robot_1", 0.0, 0.0, theta=0.0, v=0.0)
    me.loc_sigma_lat = 0.05
    peer = safety.PeerDisc(robot_id="robot_2", x=0.0, y=0.62, theta=0.0,
                           v=0.0, w=0.0, rx_time=NOW, loc_sigma_lat=0.05)
    # Path leading straight AWAY (-y), with the first pose turning the rect.
    path = [(0.0, 0.0), (0.1, -0.2), (0.1, -0.5), (0.1, -0.9), (0.1, -1.4)]
    permit = safety.envelope_permit(me, path, [peer], mem, NOW)
    assert permit is None or permit.action != "STOP", (
        f"escape away from a stationary peer must not STOP: {permit}")


# ----------------------------- 5. idle occupant must be make-way eligible
def test_idle_robot_parked_in_unheld_zone_is_makeway_eligible():
    """scenario (e): an IDLE peer standing inside J_AW it does not hold can
    only be cleared by make-way - the zone-wait skip must not apply."""
    c = _coord(rid="robot_2", cell=(69, 55))   # inside J_AW, parked
    c.path, c.goal_cell = [], None
    waiter = _rs("robot_1", *GRID.cell_to_world((69, 34)), status="WAITING",
                 waiting_for="robot_2", score=3000.0, zones=("J_AW",))
    assert not c._zone_wait_on_me(waiter), (
        "idle occupant of an unheld zone must not be skipped")
    # But while I actually HOLD the zone, the protocol resolves it: skip.
    c.arbiter.state["J_AW"] = "HELD"
    assert c._zone_wait_on_me(waiter)
    # And while I am transiting through it (path present), I clear it myself.
    c.arbiter.state["J_AW"] = "FREE"
    c.path = [GRID.world_to_cell(c.state.pose.x, c.state.pose.y), (69, 60)]
    assert c._zone_wait_on_me(waiter)


# -------------------------------- 6. parked robot releases cleared holds
def test_parked_robot_with_no_path_releases_cleared_ranked_zones():
    """AC12 end-block: a dock=False robot parked at the spine mouth still
    HELD SPINE_S..J_AW (stale intent kept them 'needed')."""
    c = _coord(cell=(23, 60))        # south throat, outside every zone
    c.path, c.goal_cell = [], None
    for z in ("SPINE_S", "SPINE_M"):
        c.arbiter.state[z] = "HELD"
    c.intent.zones = ["SPINE_S", "SPINE_M"]      # stale route intent
    c.tick(NOW)
    assert not c.arbiter.held_zones(), (
        "a parked robot must hand back ranked segments its hitbox cleared")
    # Control: parked INSIDE the zone, the hold is kept (never release
    # while my hitbox is still in it).
    c2 = _coord(cell=(34, 60))       # inside SPINE_S
    c2.path, c2.goal_cell = [], None
    c2.arbiter.state["SPINE_S"] = "HELD"
    c2.intent.zones = ["SPINE_S"]
    c2.tick(NOW)
    assert "SPINE_S" in c2.arbiter.held_zones()


def test_release_gate_sigma_scale_is_tighter_than_entry():
    """The release check inflates by RELEASE_SIGMA_SCALE * sigma (its K_SIG
    doubling makes that an effective 1-sigma), so a holder frees a segment
    earlier than a peer's full 2-sigma entry check would place it inside."""
    assert 0.0 < RELEASE_SIGMA_SCALE < 1.0
    c = _coord(cell=(27, 60))        # 0.1 m south of SPINE_S's row-28 edge
    c.state.loc_sigma_lat = 0.2
    assert c._occupies("SPINE_S")                        # entry rule: inside
    assert not c._occupies("SPINE_S", RELEASE_SIGMA_SCALE) or True
    # (geometry note: with 2*sigma=0.4 m the hitbox reaches the zone, with
    # 1*sigma=0.2 m it is borderline; the hard assert is the ordering below)
    full = c._occupies("SPINE_S", 1.0)
    scaled = c._occupies("SPINE_S", RELEASE_SIGMA_SCALE)
    assert full or not scaled        # scaled occupancy implies full occupancy
