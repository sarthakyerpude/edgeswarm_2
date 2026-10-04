"""Priority-independent safety envelope (core/safety.py, design layer L1)."""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import safety
from amr_fleet.core.coordinator import FleetCoordinator
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Permit, Pose2D, RobotState
from amr_fleet.core.safety import PeerDisc, safety_permit

# Open 6 x 6 m floor, 0.1 m cells, world (0, 0) at cell (30, 30).
GRID = GridMap(["." * 60] * 60, 0.1, (-3.0, -3.0), {})
NOW = 100.0


def _me(x=0.0, y=0.0, theta=0.0, v=0.0, rid="robot_1"):
    return RobotState(robot_id=rid, pose=Pose2D(x, y, theta), v=v)


def _line(x0, x1, y=0.0):
    """Grid path along constant y from x0 to x1."""
    r = GRID.world_to_cell(0.0, y)[0]
    c0 = GRID.world_to_cell(x0, y)[1]
    c1 = GRID.world_to_cell(x1, y)[1]
    step = 1 if c1 >= c0 else -1
    return [(r, c) for c in range(c0, c1 + step, step)]


def _disc(rid, x, y, theta=0.0, v=0.0, rx_time=NOW, suspect=False):
    return PeerDisc(robot_id=rid, x=x, y=y, theta=theta, v=v,
                    rx_time=rx_time, suspect=suspect)


def test_head_on_stops_both_robots():
    """1.0 m apart, 0.4 m/s each, closing 0.8 m/s: separation falls below
    0.67 m within 0.42 s. Neither side looks at priority."""
    a = _me(0.0, 0.0, 0.0, 0.4, "robot_1")
    b = _me(1.0, 0.0, math.pi, 0.4, "robot_2")
    pa = safety_permit(a, _line(0.0, 2.0), [_disc("robot_2", 1.0, 0.0, math.pi, 0.4)],
                       GRID, NOW)
    pb = safety_permit(b, _line(1.0, -1.0), [_disc("robot_1", 0.0, 0.0, 0.0, 0.4)],
                       GRID, NOW)
    assert pa is not None and pa.action == "STOP" and pa.blocking_robot == "robot_2"
    assert pb is not None and pb.action == "STOP" and pb.blocking_robot == "robot_1"


def _coordinator(rid, x, theta, goal_x, v, task_priority):
    c = FleetCoordinator(rid, GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(x, 0.0, theta)
    c.state.v = v
    c.state.task_priority = task_priority
    assert c.set_goal(GRID.world_to_cell(goal_x, 0.0), now=NOW)
    return c


def _peer(rid, x, theta, v, score):
    return RobotState(robot_id=rid, seq=1, pose=Pose2D(x, 0.0, theta), v=v,
                      priority_score=score)


def test_head_on_priority_winner_and_loser_both_stop_in_coordinator():
    winner = _coordinator("robot_1", 0.0, 0.0, 2.0, 0.4, task_priority=3)
    winner.on_peer_state(_peer("robot_2", 1.0, math.pi, 0.4, 0.0), NOW)
    p1 = winner.tick(NOW)
    ttc = [c for c in winner.last_conflicts if c.kind == "TTC"]
    assert ttc and all(c.i_have_priority for c in ttc)
    assert p1.action == "STOP" and p1.blocking_robot == "robot_2"

    loser = _coordinator("robot_2", 1.0, math.pi, -1.0, 0.4, task_priority=0)
    loser.on_peer_state(_peer("robot_1", 0.0, 0.0, 0.4, 1.0), NOW)
    p2 = loser.tick(NOW)
    assert p2.action == "STOP" and p2.blocking_robot == "robot_1"


def test_stationary_peer_on_path_stops_priority_winner():
    """0.8 m is outside D_stop (0.67), so this is the path rule, not the
    proximity rule. The old TTC layer let the winner creep on."""
    c = _coordinator("robot_1", 0.0, 0.0, 2.0, 0.0, task_priority=3)
    c.on_peer_state(_peer("robot_2", 0.8, math.pi, 0.0, 0.0), NOW)
    p = c.tick(NOW)
    assert c.state.priority_score > 0.0
    assert p.action == "STOP" and p.blocking_robot == "robot_2"
    assert "stationary" in p.reason


def test_offset_lane_peer_is_go():
    """Keep-right lanes are 1.0 m apart: a peer passing in the other lane
    (or parked in it) must not stop me."""
    me = _me(0.0, 0.0, 0.0, 0.4)
    path = _line(0.0, 2.0)
    oncoming = _disc("robot_2", 1.0, 1.0, math.pi, 0.4)
    parked = _disc("robot_3", 0.5, 1.0, 0.0, 0.0)
    same_way = _disc("robot_4", 0.0, -1.0, 0.0, 0.4)
    assert safety_permit(me, path, [oncoming], GRID, NOW) is None
    assert safety_permit(me, path, [parked], GRID, NOW) is None
    assert safety_permit(me, path, [same_way], GRID, NOW) is None


def test_suspect_peer_disc_is_inflated_by_age():
    me = _me(0.0, 0.0, 0.0, 0.0)
    path = _line(0.0, 2.0)
    fresh = _disc("robot_2", 1.1, 0.0)
    assert safety_permit(me, path, [fresh], GRID, NOW) is None
    stale = _disc("robot_2", 1.1, 0.0, rx_time=NOW - 1.0, suspect=True)
    p = safety_permit(me, path, [stale], GRID, NOW)
    # D_stop = 0.67 + 0.5 m/s x 1.0 s = 1.17 m > 1.1 m.
    assert p is not None and p.action == "STOP" and p.blocking_robot == "robot_2"


def test_suspect_inflation_wired_through_registry_freshness():
    c = _coordinator("robot_1", 0.0, 0.0, 2.0, 0.0, task_priority=0)
    # Parked one lane over (0.8 m lateral, min rect gap ~0.39 m): clear of a
    # FRESH peer's threshold, inside it once the peer is SUSPECT and its
    # threshold grows by v_max * age (the rect envelope stops for ANY
    # stationary peer ON the path, so an on-path peer no longer
    # discriminates freshness).
    peer = RobotState(robot_id="robot_2", seq=1,
                      pose=Pose2D(1.1, 0.8, math.pi), v=0.0)
    c.on_peer_state(peer, NOW)
    assert c.tick(NOW).action == "GO"
    p = c.tick(NOW + 1.0)                      # SUSPECT, not yet DEAD
    assert p.action == "STOP" and p.blocking_robot == "robot_2"


def test_path_leading_away_from_peer_behind_is_not_stopped():
    """Even inside D_stop: driving away is the only way out of a standoff."""
    path = _line(0.0, -2.0)
    behind = _disc("robot_2", 0.45, 0.0)
    assert safety_permit(_me(0.0, 0.0, math.pi, 0.0), path, [behind],
                         GRID, NOW) is None
    assert safety_permit(_me(0.0, 0.0, math.pi, 0.3), path, [behind],
                         GRID, NOW) is None


def test_inside_d_stop_with_path_toward_peer_stops():
    p = safety_permit(_me(0.0, 0.0, 0.0, 0.0), _line(0.0, 2.0),
                      [_disc("robot_2", 0.5, 0.0)], GRID, NOW)
    assert p is not None and p.action == "STOP"


def test_localization_sigma_widens_d_stop():
    assert abs(safety.d_stop(0.0, 0.0) - 0.67) < 1e-9
    assert abs(safety.d_stop(0.05, 0.1) - 0.87) < 1e-9
    path = _line(0.0, 2.0)
    peer = _disc("robot_2", 0.3, 0.75)         # ahead-left, 0.81 m away
    assert safety_permit(_me(), path, [peer], GRID, NOW) is None
    me = _me()
    me.loc_sigma_lat = 0.1                     # D_stop 0.87 m
    p = safety_permit(me, path, [peer], GRID, NOW)
    assert p is not None and p.action == "STOP"
    peer.loc_sigma_lat = 0.1
    assert safety_permit(_me(), path, [peer], GRID, NOW).action == "STOP"


def test_ttc_band_slows_without_stopping():
    """2.5 m apart closing at 0.8 m/s: TTC = (2.5 - 0.67)/0.8 = 2.29 s."""
    p = safety_permit(_me(0.0, 0.0, 0.0, 0.4), _line(0.0, 2.0),
                      [_disc("robot_2", 2.5, 0.0, math.pi, 0.4)], GRID, NOW)
    assert p is not None and p.action == "SLOW"
    assert abs(p.speed_scale - 2.29 / 3.5) < 0.02
    assert p.blocking_robot == "robot_2"


def test_most_restrictive_keeps_zone_and_never_loosens():
    go = Permit(action="GO", speed_scale=1.0, zone_id="Z")
    stop = Permit(action="STOP", speed_scale=0.0, blocking_robot="robot_2")
    merged = safety.most_restrictive(go, stop)
    assert merged.action == "STOP" and merged.zone_id == "Z"
    wait = Permit(action="STOP", speed_scale=0.0, zone_id="Z")
    slow = Permit(action="SLOW", speed_scale=0.5)
    assert safety.most_restrictive(wait, slow) is wait
    assert safety.most_restrictive(go, None) is go


# =========================================================================
# Increment 3/4: envelope_permit - the oriented-rect envelope.
# =========================================================================
from amr_fleet.core.safety import SafetyMemory, envelope_permit  # noqa: E402


def _rme(x=0.0, y=0.0, theta=0.0, v=0.0, rid="robot_1", sigma=0.05):
    st = RobotState(robot_id=rid, pose=Pose2D(x, y, theta), v=v)
    st.loc_sigma_lat = sigma
    return st


def _wpath(x0, x1, y=0.0, step=0.1):
    n = int(round(abs(x1 - x0) / step))
    s = step if x1 >= x0 else -step
    return [(x0 + i * s, y) for i in range(1, n + 1)]


def test_rect_head_on_stops_both_and_names_the_blocker():
    a = _rme(0.0, 0.0, 0.0, 0.4)
    b = _rme(1.2, 0.0, math.pi, 0.4, rid="robot_2")
    pa = envelope_permit(a, _wpath(0.0, 2.0),
                         [_disc("robot_2", 1.2, 0.0, math.pi, 0.4)],
                         SafetyMemory(), NOW)
    pb = envelope_permit(b, _wpath(1.2, -1.0),
                         [_disc("robot_1", 0.0, 0.0, 0.0, 0.4)],
                         SafetyMemory(), NOW)
    assert pa and pa.action == "STOP" and pa.blocking_robot == "robot_2"
    assert pb and pb.action == "STOP" and pb.blocking_robot == "robot_1"


def test_offset_lane_pass_is_not_stopped_by_the_rect_envelope():
    """Two robots in the R7 spine lanes (+-0.4 m) pass with a 0.39 m rect
    gap. The disc model (D_stop 0.67) made this single-file; the rect
    envelope must let it happen - abreast is clear, and the approach is at
    most a SLOW, never a STOP."""
    me = _rme(-0.4, 0.0, math.pi / 2, 0.4)
    path_n = [(-0.4, 0.1 * i) for i in range(1, 21)]
    # abreast
    abreast = PeerDisc(robot_id="robot_2", x=0.4, y=0.0,
                       theta=-math.pi / 2, v=0.4, rx_time=NOW,
                       loc_sigma_lat=0.05)
    assert envelope_permit(me, path_n, [abreast], SafetyMemory(), NOW) is None
    # oncoming in the other lane, 2 m ahead
    oncoming = PeerDisc(robot_id="robot_2", x=0.4, y=2.0,
                        theta=-math.pi / 2, v=0.4, rx_time=NOW,
                        loc_sigma_lat=0.05)
    p = envelope_permit(me, path_n, [oncoming], SafetyMemory(), NOW)
    assert p is None or p.action == "SLOW"


def test_rect_envelope_path_leading_away_is_an_escape():
    """Inside the threshold with a path that monotonically opens the gap:
    driving away is the only way out of a standoff and must stay allowed."""
    me = _rme(0.0, 0.0, math.pi, 0.3)
    behind = _disc("robot_2", 0.55, 0.0)
    p = envelope_permit(me, _wpath(0.0, -2.0), [behind], SafetyMemory(), NOW)
    assert p is None


def test_rect_envelope_stationary_peer_on_path_stops_me():
    me = _rme(0.0, 0.0, 0.0, 0.3)
    parked = _disc("robot_3", 1.1, 0.0, math.pi, 0.0)
    p = envelope_permit(me, _wpath(0.0, 2.0), [parked], SafetyMemory(), NOW)
    assert p is not None and p.action == "STOP"
    assert p.blocking_robot == "robot_3"
    # the same peer BESIDE the path (1.0 m lateral) does not stop me
    beside = _disc("robot_3", 1.1, 1.0, 0.0, 0.0)
    assert envelope_permit(me, _wpath(0.0, 2.0), [beside],
                           SafetyMemory(), NOW) is None


def test_hysteresis_raises_the_threshold_until_five_clear_ticks():
    """After a STOP for P the threshold grows by HYST_M until P is clear 5
    consecutive ticks - the anti-flip rule, measured in a SafetyMemory, no
    module state."""
    mem = SafetyMemory()
    path = _wpath(0.0, 2.0)

    def permit_at(d, memory=None):
        # head-on peer MOVING at 0.3 (so only the proximity rule, not the
        # stationary-path rule, is in play); closing speed 0.5 saturates the
        # v_close margin: g = 0.12 + 0.13 + 0.1414 = 0.391, band to 0.491.
        me = _rme(0.0, 0.0, 0.0, 0.2)
        peer = _disc("robot_2", d, 0.0, math.pi, 0.3)
        return envelope_permit(me, path, [peer],
                               mem if memory is None else memory, NOW)

    # 1. clearly inside (min gap 0.3 < 0.391): STOP arms the hysteresis
    p = permit_at(1.2)
    assert p is not None and p.action == "STOP"
    # 2. in the hysteresis band (min gap 0.44, between g and g + 0.10)
    p = permit_at(1.34)
    assert p is not None and p.action == "STOP"
    # control: a fresh memory (hysteresis off) would NOT stop here
    q = permit_at(1.34, memory=SafetyMemory(enabled=False))
    assert q is None or q.action != "STOP"
    # 3. five clear ticks drop the bonus
    for _ in range(5):
        p = permit_at(3.5)
        assert p is None or p.action != "STOP"
    p = permit_at(1.34)
    assert p is None or p.action != "STOP"


def test_spinning_peer_is_its_circumscribed_disc():
    """A spinning robot sweeps r=0.286, wider than its rect: a lateral
    clearance that passes a still peer must stop for a spinning one."""
    path = _wpath(0.0, 2.0)

    def permit(w):
        me = _rme(0.0, 0.0, 0.0, 0.2)
        peer = PeerDisc(robot_id="robot_2", x=1.2, y=0.78, theta=0.0,
                        v=0.0, w=w, rx_time=NOW, loc_sigma_lat=0.05)
        return envelope_permit(me, path, [peer], SafetyMemory(), NOW)

    still = permit(0.0)
    assert still is None or still.action != "STOP"
    spinning = permit(0.6)
    assert spinning is not None and spinning.action == "STOP"
    assert spinning.blocking_robot == "robot_2"


def test_rotate_in_place_exception_lets_a_stopped_robot_turn_out():
    """Nearly stationary, path leading away, swept gap within 0.09 of now:
    the robot must be allowed to rotate toward its exit (w sweeps the rect,
    which would otherwise re-trigger the STOP it is resolving)."""
    me = _rme(0.0, 0.0, 0.5, 0.0)
    me.w = 0.6                      # turning in place toward the exit
    peer = _disc("robot_2", 0.62, 0.0, math.pi, 0.0)
    p = envelope_permit(me, _wpath(0.0, -1.0), [peer], SafetyMemory(), NOW)
    assert p is None
