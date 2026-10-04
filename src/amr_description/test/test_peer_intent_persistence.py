"""Peer intents survive the 10 Hz state stream (the P0 zone-mutex bypass)."""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.coordinator import FleetCoordinator
from amr_fleet.core.gridmap import GridMap, Zone
from amr_fleet.core.models import Intent, Pose2D, RobotState
from amr_fleet.core.peers import DEAD, FRESH, SUSPECT, PeerRegistry


def _state(seq, boot=1, x=0.0, y=0.0, theta=0.0, v=0.0, w=0.0,
           intent_seq=0):
    # Exactly what conversions.state_from_ros builds: no intent attached.
    return RobotState(robot_id="robot_2", seq=seq, boot_id=boot,
                      pose=Pose2D(x, y, theta), v=v, w=w,
                      intent_seq=intent_seq)


def _intent(zones=("Z",)):
    return Intent(cells=[(5, 5), (5, 6)], t_enter=[0.0, 0.25],
                  t_exit=[0.75, 1.0], zones=list(zones))


def _grid():
    """10 m x 2 m open strip; zone NEAR at x~1 m, zone FAR at x~8 m."""
    occ = ["." * 100 for _ in range(20)]
    zones = {"NEAR": Zone("NEAR", "intersection",
                          {(r, c) for r in range(5, 15) for c in range(8, 12)}),
             "FAR": Zone("FAR", "intersection",
                         {(r, c) for r in range(5, 15) for c in range(78, 82)})}
    return GridMap(occ, 0.1, (0.0, 0.0), zones)


def test_state_intent_state_keeps_peer_needing_zone():
    reg = PeerRegistry("robot_1")
    assert reg.update(_state(1, intent_seq=1), now=0.0)
    assert reg.update_intent("robot_2", _intent(), seq=1, now=0.05)
    assert reg.update(_state(2, intent_seq=1), now=0.1)     # fresh, empty
    assert reg.peers_needing_zone("Z") == {"robot_2"}
    assert reg.peers["robot_2"].intent.cells == [(5, 5), (5, 6)]


def test_coordinator_path_keeps_peer_intent_across_states():
    grid = _grid()
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.on_peer_state(_state(1, intent_seq=1), now=0.0)
    c.registry.update_intent("robot_2", _intent(("NEAR",)), seq=1, now=0.05)
    for k in range(2, 12):                                   # 1 s of states
        c.on_peer_state(_state(k, intent_seq=1), now=0.1 * k)
        assert c.registry.peers_needing_zone("NEAR") == {"robot_2"}


def test_intent_before_first_state_is_buffered_and_applied():
    reg = PeerRegistry("robot_1")
    assert reg.update_intent("robot_2", _intent(), seq=3, now=0.0)
    assert "robot_2" not in reg.peers                  # not a peer yet
    assert reg.peers_needing_zone("Z") == set()
    assert reg.stats["intents_buffered"] == 1
    reg.update(_state(1, boot=7, intent_seq=3), now=0.1)
    assert reg.peers_needing_zone("Z") == {"robot_2"}
    assert reg.intents["robot_2"].boot_id == 7
    assert not reg.plan_unknown("robot_2")


def test_new_boot_id_drops_old_intent():
    reg = PeerRegistry("robot_1")
    reg.update(_state(10, boot=1, intent_seq=4), now=0.0)
    reg.update_intent("robot_2", _intent(), seq=4, now=0.0)
    assert reg.peers_needing_zone("Z") == {"robot_2"}
    reg.update(_state(1, boot=2), now=0.1)             # restarted, no intent
    assert "robot_2" not in reg.intents
    assert reg.peers["robot_2"].intent.cells == []
    assert reg.peers_needing_zone("Z") == set()
    assert reg.stats["intents_dropped_reboot"] == 1


def test_older_intent_rejected_and_same_seq_keepalive_needs_newer_stamp():
    reg = PeerRegistry("robot_1")
    reg.update(_state(1), now=0.0)
    assert reg.update_intent("robot_2", _intent(("A",)), seq=5, now=0.0,
                             stamp=100.0)
    assert not reg.update_intent("robot_2", _intent(("B",)), seq=4, now=0.1,
                                 stamp=101.0)
    assert not reg.update_intent("robot_2", _intent(("B",)), seq=5, now=0.1,
                                 stamp=99.0)
    assert reg.peers["robot_2"].intent.zones == ["A"]
    assert reg.update_intent("robot_2", _intent(("C",)), seq=5, now=0.2,
                             stamp=101.0)
    assert reg.peers["robot_2"].intent.zones == ["C"]


def test_intent_seq_ahead_means_unknown_plan_needing_nearby_zones():
    grid = _grid()
    reg = PeerRegistry("robot_1")
    reg.zones_near = grid.zones_within
    # Peer at x=1.0 m: NEAR (x 0.8-1.2) within 3 m, FAR (x 7.8-8.2) not.
    reg.update(_state(1, x=1.0, y=1.0, intent_seq=2), now=0.0)
    reg.update_intent("robot_2", _intent(("FAR",)), seq=1, now=0.0)
    assert reg.plan_unknown("robot_2")
    assert reg.peers_needing_zone("NEAR") == {"robot_2"}
    assert reg.peers_needing_zone("FAR") == {"robot_2"}   # listed in intent
    reg.update_intent("robot_2", _intent(()), seq=2, now=0.05)
    assert not reg.plan_unknown("robot_2")
    assert reg.peers_needing_zone("NEAR") == set()


def test_state_without_any_intent_is_unknown_plan():
    reg = PeerRegistry("robot_1")
    reg.update(_state(1, intent_seq=1), now=0.0)
    assert reg.plan_unknown("robot_2")
    assert reg.peers_needing_zone("anything") == {"robot_2"}   # no locator


def test_embedded_intent_still_works_for_core_callers():
    reg = PeerRegistry("robot_1")
    st = _state(1)
    st.intent = _intent()
    reg.update(st, now=0.0)
    reg.update(_state(2), now=0.1)
    assert reg.peers_needing_zone("Z") == {"robot_2"}


def test_freshness_levels_from_rx_time():
    reg = PeerRegistry("robot_1", t_suspect=2.0, t_dead=5.0)
    reg.update(_state(1), now=10.0)
    assert reg.freshness("robot_2", 10.2) == FRESH
    assert reg.freshness("robot_2", 10.5) == SUSPECT
    assert reg.freshness("robot_2", 13.0) == SUSPECT
    assert reg.freshness("robot_2", 15.5) == DEAD
    assert reg.freshness("nobody", 10.0) == DEAD
    # The existing health/alive API is unchanged.
    reg.tick(15.5)
    assert reg.health["robot_2"] == DEAD
    assert "robot_2" not in reg.alive()


def test_extrapolated_pose_uses_v_w_over_age_plus_lead():
    reg = PeerRegistry("robot_1")
    reg.update(_state(1, x=1.0, y=2.0, theta=0.0, v=0.5), now=0.0)
    p = reg.extrapolated_pose("robot_2", 0.4)                  # 0.5 s
    assert math.isclose(p.x, 1.25, abs_tol=1e-9)
    assert math.isclose(p.y, 2.0, abs_tol=1e-9)
    # Turning: same arc model as geometry.extrapolate.
    reg.update(_state(2, x=0.0, y=0.0, theta=0.0, v=0.4, w=0.5), now=1.0)
    q = reg.extrapolated_pose("robot_2", 1.9)                  # 1.0 s
    assert math.isclose(q.theta, 0.5, abs_tol=1e-9)
    assert q.y > 0.0
    # DEAD peers are frozen obstacles, not extrapolated.
    reg.tick(10.0)
    f = reg.extrapolated_pose("robot_2", 10.0)
    assert (f.x, f.y) == (0.0, 0.0)
    assert reg.extrapolated_pose("nobody", 0.0) is None
