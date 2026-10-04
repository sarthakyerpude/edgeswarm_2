"""Return-to-dock policy: idle grace, low-battery hold, precise alignment on
the spawn pose, bounded retries, and no in-place turns next to a peer."""
import math
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.docking import (ALIGNING, BACK_OFF, BACKING_OFF, DOCK,
                                    DOCKED, GO_HOME, RETURNING, DockPolicy,
                                    ROTATE_CLEAR_M)

HOME = (-3.0, -4.0, math.pi / 2)
FIVE_DEG = math.radians(5.0)


def mk(**kw):
    return DockPolicy((10, 30), home_pose=HOME, idle_s=4.0, low_pct=30.0,
                      resume_pct=80.0, approach_m=0.35, pos_tol_m=0.08,
                      yaw_tol_rad=FIVE_DEG, align_timeout_s=20.0,
                      max_retries=2, backoff_m=0.4, **kw)


def step(p, now, pos=5.0, yaw=0.0, task=False, transient=False, goal=False,
         batt=90.0, settled=True, clear=True):
    return p.step(now, has_task=task, transient=transient, pos_err_m=pos,
                  yaw_err_rad=yaw, settled=settled, has_goal=goal,
                  battery_pct=batt, rotation_clear=clear)


def to_aligning(p, t=0.0):
    step(p, t); step(p, t + 4.0)                       # idle -> RETURNING
    assert p.phase == RETURNING
    step(p, t + 10.0, pos=0.3, yaw=0.5, goal=True)     # near -> ALIGNING
    assert p.phase == ALIGNING
    return t + 10.0


# ------------------------------------------------------------ return home
def test_idle_robot_waits_grace_then_goes_home():
    p = mk()
    assert step(p, 0.0) is None
    assert step(p, 3.9) is None
    assert step(p, 4.0) == GO_HOME
    assert p.phase == RETURNING


def test_low_battery_goes_home_without_grace_and_holds():
    p = mk()
    assert step(p, 0.0, batt=25.0) == GO_HOME
    assert p.charge_hold


def test_hold_has_hysteresis():
    p = mk()
    p.update_hold(29.0); assert p.charge_hold
    p.update_hold(60.0); assert p.charge_hold
    p.update_hold(80.0); assert not p.charge_hold


def test_lost_home_goal_is_reissued():
    p = mk()
    step(p, 0.0); step(p, 4.0)
    assert step(p, 5.0, goal=True) is None
    assert step(p, 6.0, goal=False) == GO_HOME


# ------------------------------------------------------------- alignment
def test_near_is_not_docked_until_aligned():
    """Arriving within reach is NOT docking: position AND heading must match."""
    p = mk()
    t = to_aligning(p)
    assert step(p, t + 1, pos=0.06, yaw=0.3, goal=True) is None   # heading off
    assert step(p, t + 2, pos=0.12, yaw=0.0, goal=True) is None   # position off
    assert step(p, t + 3, pos=0.05, yaw=0.02, goal=True, settled=False) is None
    assert p.phase == ALIGNING
    assert step(p, t + 4, pos=0.05, yaw=0.02, goal=True) == DOCK
    assert p.phase == DOCKED and not p.misaligned


def test_yaw_tolerance_is_five_degrees_both_ways():
    for yaw, ok in ((math.radians(4.9), True), (math.radians(-4.9), True),
                    (math.radians(5.5), False), (math.radians(-5.5), False)):
        p = mk()
        t = to_aligning(p)
        got = step(p, t + 1, pos=0.03, yaw=yaw, goal=True)
        assert (got == DOCK) is ok, (math.degrees(yaw), got)


def test_robot_already_on_spawn_pose_docks_immediately():
    """Startup: robots spawn on their docks and report CHARGING."""
    p = mk()
    assert step(p, 0.0, pos=0.01, yaw=0.0) == DOCK
    assert p.phase == DOCKED


def test_robot_near_but_skewed_at_startup_squares_up():
    p = mk()
    assert step(p, 0.0, pos=0.1, yaw=0.6) == GO_HOME
    assert p.phase == ALIGNING


def test_alignment_timeout_backs_off_and_reapproaches():
    p = mk()
    t = to_aligning(p)
    assert step(p, t + 10, pos=0.1, yaw=0.3, goal=True) is None
    assert step(p, t + 21, pos=0.1, yaw=0.3, goal=True) == BACK_OFF
    assert p.phase == BACKING_OFF and p.retries == 1
    assert step(p, t + 22, pos=0.15, goal=True) is None          # still close
    assert step(p, t + 25, pos=0.35, goal=True) == GO_HOME       # backed off
    assert p.phase == RETURNING


def test_alignment_gives_up_after_max_retries_without_looping():
    p = mk()
    t = to_aligning(p)
    for attempt in (1, 2):
        step(p, t + 0.1, pos=0.1, yaw=0.3, goal=True)
        assert step(p, t + 21, pos=0.1, yaw=0.3, goal=True) == BACK_OFF
        assert p.retries == attempt
        assert step(p, t + 23, pos=0.4, goal=True) == GO_HOME
        t += 30
        step(p, t, pos=0.2, yaw=0.3, goal=True)                  # ALIGNING again
        assert p.phase == ALIGNING
    step(p, t + 0.1, pos=0.1, yaw=0.3, goal=True)
    assert step(p, t + 21, pos=0.1, yaw=0.3, goal=True) == DOCK
    assert p.phase == DOCKED and p.misaligned


def test_no_in_place_turn_with_peer_within_reach():
    p = mk()
    t = to_aligning(p)
    # On the spot, heading off, peer close: hold still, clock paused.
    assert step(p, t + 1, pos=0.03, yaw=0.5, goal=True, clear=False) is None
    assert p.hold_motion
    assert step(p, t + 40, pos=0.03, yaw=0.5, goal=True, clear=False) is None
    assert p.phase == ALIGNING and p.retries == 0      # no timeout while held
    # Peer gone: resume turning, no hold.
    assert step(p, t + 41, pos=0.03, yaw=0.5, goal=True, clear=True) is None
    assert not p.hold_motion
    assert step(p, t + 42, pos=0.03, yaw=0.01, goal=True) == DOCK


def test_driving_in_is_not_held_by_a_nearby_peer():
    """Only the in-place turn is held; the straight approach is the normal
    coordination layer's business."""
    p = mk()
    t = to_aligning(p)
    assert step(p, t + 1, pos=0.3, yaw=0.0, goal=True, clear=False) is None
    assert not p.hold_motion


def test_rotate_clearance_covers_two_swept_chassis():
    assert ROTATE_CLEAR_M > 2 * math.hypot(0.20, 0.16)


# ---------------------------------------------------- interruptions/reset
def test_task_resets_dock_intent():
    p = mk()
    to_aligning(p)
    assert step(p, 50.0, task=True) is None
    assert p.phase is None
    assert step(p, 51.0) is None
    assert step(p, 55.0) == GO_HOME            # full grace again


def test_transient_dwell_or_retreat_defers_decision():
    p = mk()
    assert step(p, 0.0, transient=True) is None
    assert step(p, 100.0, transient=True) is None
    assert p.phase is None


def test_pushed_off_dock_returns_and_realigns():
    p = mk()
    step(p, 0.0, pos=0.0)
    assert p.phase == DOCKED
    assert step(p, 1.0, pos=0.5) is None        # within hysteresis
    assert step(p, 2.0, pos=2.0) is None        # left: grace restarts
    assert step(p, 6.0, pos=2.0) == GO_HOME


def test_no_home_means_no_actions():
    p = DockPolicy(None)
    assert step(p, 0.0) is None
    assert step(p, 100.0) is None


def test_backoff_pose_is_in_front_of_dock():
    x, y = mk().backoff_pose()
    assert math.isclose(x, -3.0, abs_tol=1e-9) and math.isclose(y, -3.6)


def test_errors_wrap_heading():
    p = mk()
    pe, ye = p.errors(-3.0, -4.0, math.pi / 2 + 2 * math.pi + 0.05)
    assert pe == 0.0 and math.isclose(ye, -0.05, abs_tol=1e-9)
