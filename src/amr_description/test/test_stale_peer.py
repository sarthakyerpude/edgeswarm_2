"""Stale/silent peers: liveness tiers (core/peers.py) and envelope ghosts
(core/safety.py). The sih-57 field case: a peer whose heartbeat stalls
10-16 s under load is still physically there and possibly moving - it must
stay an obstacle (ghost) and must never be tiered on heartbeat alone when
its other traffic flows."""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import peers as peers_mod
from amr_fleet.core import safety
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Intent, Pose2D, RobotState
from amr_fleet.core.peers import GONE, PeerRegistry, SILENT
from amr_fleet.core.safety import (PeerDisc, SafetyMemory, envelope_permit,
                                   silent_ghost_discs)

GRID = GridMap(["." * 100] * 100, 0.1, (-5.0, -5.0), {})


def _state(rid="robot_2", x=0.0, y=0.0, th=0.0, v=0.0, seq=1, cells=None):
    st = RobotState(robot_id=rid, seq=seq, pose=Pose2D(x, y, th), v=v)
    if cells:
        st.intent = Intent(cells=list(cells),
                           t_enter=[0.0] * len(cells),
                           t_exit=[1.0] * len(cells))
        st.intent_seq = 1
    return st


def _me(x=0.0, y=0.0, th=0.0, v=0.0):
    return RobotState(robot_id="robot_1", pose=Pose2D(x, y, th), v=v)


def _path(x0, x1, y=0.0, step=0.1):
    n = int(round((x1 - x0) / step))
    return [(x0 + i * step, y) for i in range(1, n + 1)]


# ------------------------------------------------------------------ tiers ---
def test_heartbeat_only_silence_leaves_the_peer_alive_via_touch():
    """10 Hz state stalls but intent/zone traffic flows: the peer must stay
    FRESH/SUSPECT (a granter), never SILENT - heartbeat-only stalls under
    CPU load were ghosting peers whose zone protocol was alive."""
    reg = PeerRegistry("robot_1")
    reg.update(_state(), now=0.0)
    assert reg.tier("robot_2", 0.1) == peers_mod.FRESH
    # no state for 10 s -> SILENT...
    assert reg.tier("robot_2", 10.0) == SILENT
    assert reg.silent(10.0) == ["robot_2"]
    # ...but an INTENT at t=10 is liveness evidence
    reg.update_intent("robot_2", Intent(), seq=2, now=10.0)
    assert reg.tier("robot_2", 10.1) == peers_mod.FRESH
    assert reg.silent(10.1) == []
    assert "robot_2" in reg.granters(10.1)
    # ...and so is raw zone traffic reported via touch()
    reg.touch("robot_2", 21.0)
    assert reg.tier("robot_2", 21.2) == peers_mod.FRESH
    assert "robot_2" in reg.granters(21.2)


def test_tier_ladder_and_gone():
    reg = PeerRegistry("robot_1")
    reg.update(_state(), now=0.0)
    assert reg.tier("robot_2", 0.2) == peers_mod.FRESH
    assert reg.tier("robot_2", 2.0) == peers_mod.SUSPECT
    assert reg.tier("robot_2", 12.0) == SILENT
    assert reg.tier("robot_2", 31.0) == GONE
    assert reg.gone_ids(31.0) == ["robot_2"]
    assert reg.granters(31.0) == []
    # a peer never heard from is GONE, and my own id never tiers
    assert reg.tier("robot_9", 0.0) == GONE
    reg.touch("robot_1", 5.0)
    assert "robot_1" not in reg.last_touch


def test_silent_peer_keeps_granter_status_and_diagnostic_names_survive():
    """SILENT (5-30 s) peers stay required granters (sih-57 exclusion fix),
    and the attribute names sih-57's diagnostics read still exist."""
    reg = PeerRegistry("robot_1")
    reg.update(_state(), now=0.0)
    assert "robot_2" in reg.granters(12.0)        # SILENT but a granter
    # unchanged names (CONCURRENCY contract)
    assert callable(reg.freshness) and callable(reg.alive)
    assert callable(reg.plan_unknown) and callable(reg.held_intent_seq)
    assert callable(reg.loss_rate)
    assert reg.last_seen["robot_2"] == 0.0


# ----------------------------------------------------------------- ghosts ---
def test_silent_while_moving_peer_is_an_envelope_ghost_that_stops_me():
    """A peer silent 12 s (sih-57 saw 10-16 s) that was last seen MOVING is a
    frozen rect whose threshold grows by v_max*age: approaching its last pose
    must STOP me even though its broadcast pose is 12 s stale."""
    reg = PeerRegistry("robot_1")
    reg.update(_state(x=2.0, y=0.0, th=math.pi, v=0.4), now=0.0)
    ghosts = silent_ghost_discs(reg, 12.0)
    assert [g.robot_id for g in ghosts] == ["robot_2"]
    g = ghosts[0]
    assert g.silent and g.v == 0.0 and g.w == 0.0
    assert (g.x, g.y) == (2.0, 0.0)
    me = _me(0.0, 0.0, 0.0, 0.3)
    p = envelope_permit(me, _path(0.0, 1.5), ghosts, SafetyMemory(), 12.0)
    assert p is not None and p.action == "STOP"
    assert p.blocking_robot == "robot_2"


def test_ghost_reach_is_bounded_by_the_peers_own_last_motion():
    """Degraded-comms fix: the stale reach is hitbox.stale_reach - a peer
    last seen STATIONARY grows 0.5*A_MAX*age^2 capped at 0.3 m; a peer last
    seen MOVING keeps the kinematic V_MAX-capped bound (1.5 m). With my
    path ending 0.2 m rect-front at 1.7 m, a frozen peer at d leaves a
    residual path gap of d - 1.9; the stationary rule stops me once
    0.7283 (sigma term) + reach beats it."""
    def permit(d, age, v_last=0.0):
        peer = PeerDisc(robot_id="robot_2", x=d, y=0.0, theta=math.pi,
                        v=0.0, w=0.0, rx_time=100.0 - age, suspect=True,
                        loc_sigma_lat=0.05, v_last=v_last)
        me = _me(0.0, 0.0, 0.0, 0.3)
        return envelope_permit(me, _path(0.0, 1.5), [peer],
                               SafetyMemory(), 100.0)

    def stopped(d, age, v_last=0.0):
        p = permit(d, age, v_last)
        return p is not None and p.action == "STOP"

    # last seen STATIONARY: reach 0.5*2*age^2 capped at 0.3, so the
    # stationary-rule threshold is 0.261 + reach against residual d - 1.9.
    assert not stopped(2.4, 0.25)        # reach 0.06: 0.32 < 0.5 residual
    assert stopped(2.4, 0.5)             # reach 0.25: 0.51 > 0.5
    # Within the STATIC_TRUST_S window (2 s) a last-seen-stationary peer
    # never grows a V_MAX bubble: reach stays at the 0.3 m cap, 0.56 < 0.6.
    assert not stopped(2.5, 1.9), (
        "a stationary stale peer must not grow a V_MAX bubble while trusted")
    # Past the trust window it may have long since driven off: the reach
    # re-grows from rest toward REACH_CAP_M (hitbox.stale_reach design).
    assert stopped(2.5, 20.0)
    # last seen MOVING: kinematic bound, capped at REACH_CAP_M = 1.5
    assert stopped(3.0, 1.0, v_last=1.0)     # reach 1.0: 1.36 > 1.1
    assert stopped(3.0, 20.0, v_last=1.0)    # capped at 1.5, still stop
    assert not stopped(4.2, 20.0, v_last=1.0)    # residual 2.3 > 1.76


def test_stationary_stale_peer_does_not_freeze_an_approaching_robot():
    """The measured live freeze at 1.0 m/s under degraded comms (3-4 Hz,
    age ~1.6 s): robots stopped each other at rect gap 1.35 m < 1.56 m
    threshold. A peer last seen STATIONARY 1.6 s ago, 1.35 m of rect gap
    ahead, must NOT stop a (SLOW-braked) approaching robot; the same peer
    last seen MOVING toward me must."""
    me = _me(0.0, 0.0, 0.0, 0.25)            # post-SLOW approach creep
    def peer(v_broadcast, v_last):
        return PeerDisc(robot_id="robot_2", x=1.75, y=0.0, theta=math.pi,
                        v=v_broadcast, w=0.0, rx_time=100.0 - 1.6,
                        suspect=True, loc_sigma_lat=0.05, v_last=v_last)
    p = envelope_permit(me, _path(0.0, 0.7), [peer(0.0, 0.0)],
                        SafetyMemory(), 100.0)
    assert p is None or p.action != "STOP", (p and p.reason)
    p2 = envelope_permit(me, _path(0.0, 0.7), [peer(1.0, 1.0)],
                         SafetyMemory(), 100.0)
    assert p2 is not None and p2.action == "STOP"


def test_ghost_points_follow_the_last_intent_out_to_the_capped_reach():
    """With the grid, extra ghost points lie along the silent peer's last
    broadcast intent, never farther than min(REACH_CAP_M, v_max*age)."""
    # intent: straight line east from (0,0), 3 m long
    row, col0 = GRID.world_to_cell(0.0, 0.0)
    cells = [(row, col0 + i) for i in range(31)]
    reg = PeerRegistry("robot_1")
    reg.update(_state(x=0.05, y=0.05, v=0.4, cells=cells), now=0.0)
    ghosts = silent_ghost_discs(reg, 12.0, GRID)   # age 12 -> reach capped
    assert len(ghosts) > 1
    base = ghosts[0]
    cap = safety.REACH_CAP_M
    dists = [math.hypot(g.x - base.x, g.y - base.y) for g in ghosts[1:]]
    assert max(dists) <= cap + 0.5 * GRID.resolution
    assert max(dists) >= cap - 0.2       # the reach is actually used
    assert all(g.silent and g.v == 0.0 for g in ghosts)
    # path ghosts carry rx_time=now so the reach is not double-counted
    assert all(g.rx_time == 12.0 for g in ghosts[1:])


def test_sub_silent_peer_produces_no_ghost():
    reg = PeerRegistry("robot_1")
    reg.update(_state(x=1.0), now=0.0)
    assert silent_ghost_discs(reg, 3.0) == []      # SUSPECT, not SILENT
    assert silent_ghost_discs(reg, 31.0) == []     # GONE, not SILENT
