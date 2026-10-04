"""
Return-to-dock policy: an idle robot drives home, aligns, and charges there.

Every robot's spawn pose doubles as its charging dock. A robot with no task
waits a short grace period (so a queued task re-announced by the auction's
no-bid retry can still win it), then heads home. It is only DOCKED - and only
reports CHARGING - once it sits on the spawn pose itself: position within
pos_tol_m and heading within yaw_tol_rad of the spawn yaw, at rest. Arriving
"somewhere near" is not docking.

    idle -> RETURNING --near--> ALIGNING --aligned--> DOCKED
                                   |  timeout (<= max_retries)
                                   v
                              BACKING_OFF --> RETURNING (fresh approach)

The final approach is bounded: after align_timeout_s the robot backs off
backoff_m along its dock heading and approaches again; after max_retries it
docks anyway, flagged `misaligned`, rather than looping forever.

Rotating in place sweeps a circle wider than the chassis, so the policy never
does it with a peer that close: it raises `hold_motion` (the node publishes a
STOP permit) and pauses the alignment clock until the peer has passed.

Low battery is a HYSTERESIS hold, not a threshold: at or below low_pct the
robot goes home straight after its current task and stops bidding until it is
back above resume_pct. Without the gap it would bid, drain a little, and
bounce between work and dock on every task.

Pure decision logic - the node supplies pose errors and applies the actions.
"""
import math
from typing import Optional, Tuple

from .models import Cell

RETURNING = "RETURNING"
ALIGNING = "ALIGNING"
BACKING_OFF = "BACKING_OFF"
DOCKED = "DOCKED"

# Actions returned by DockPolicy.step()
GO_HOME = "GO_HOME"        # (re)issue the precise approach to the dock pose
BACK_OFF = "BACK_OFF"      # drive to backoff_pose() before re-approaching
DOCK = "DOCK"              # aligned (or gave up): stop and charge

# Rotation-in-place clearance: two chassis swept radii (0.40 x 0.32 m box ->
# hypot(0.20, 0.16) = 0.256 m each) plus a 0.15 m gap. Callers add 2 sigma.
ROTATE_CLEAR_M = 2 * math.hypot(0.20, 0.16) + 0.15


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class DockPolicy:
    def __init__(self, home_cell: Optional[Cell],
                 home_pose: Optional[Tuple[float, float, float]] = None,
                 idle_s: float = 4.0,
                 low_pct: float = 30.0, resume_pct: float = 80.0,
                 approach_m: float = 0.35,
                 pos_tol_m: float = 0.08,
                 yaw_tol_rad: float = math.radians(5.0),
                 align_timeout_s: float = 20.0,
                 max_retries: int = 2,
                 backoff_m: float = 0.4,
                 tolerance_m: Optional[float] = None):
        # tolerance_m: the original position-only API (no home_pose, no
        # heading): "docked" = within tolerance_m and at rest. Kept so older
        # callers such as test/fleet_sim.py keep working unchanged.
        if tolerance_m is not None:
            approach_m = pos_tol_m = tolerance_m
        self.home_cell = home_cell
        self.home_pose = home_pose          # (x, y, yaw) of the spawn/dock
        self.idle_s = max(0.0, idle_s)
        self.low_pct = low_pct
        self.resume_pct = max(low_pct, resume_pct)
        self.approach_m = approach_m
        self.pos_tol_m = pos_tol_m
        self.yaw_tol_rad = yaw_tol_rad
        self.align_timeout_s = align_timeout_s
        self.max_retries = max(0, int(max_retries))
        self.backoff_m = backoff_m

        self.phase: Optional[str] = None
        self.charge_hold = False
        self.hold_motion = False            # node: publish a STOP permit
        self.misaligned = False             # docked after giving up aligning
        self.retries = 0
        self._idle_since: Optional[float] = None
        self._align_elapsed = 0.0
        self._last_t: Optional[float] = None
        self._backoff_since: Optional[float] = None

    # ------------------------------------------------------------- helpers
    def update_hold(self, battery_pct: float) -> None:
        if battery_pct <= self.low_pct:
            self.charge_hold = True
        elif self.charge_hold and battery_pct >= self.resume_pct:
            self.charge_hold = False

    def reset(self) -> None:
        """A task (or a fault) owns the robot now; forget any dock intent."""
        self.phase = None
        self.hold_motion = False
        self.retries = 0
        self._idle_since = None
        self._align_elapsed = 0.0
        self._backoff_since = None

    def backoff_pose(self) -> Optional[Tuple[float, float]]:
        """A point backoff_m in front of the dock, along the dock heading."""
        if self.home_pose is None:
            return None
        x, y, yaw = self.home_pose
        return (x + self.backoff_m * math.cos(yaw),
                y + self.backoff_m * math.sin(yaw))

    def errors(self, x: float, y: float, theta: float) -> Tuple[float, float]:
        """(position error m, signed yaw error rad) to the dock pose."""
        hx, hy, hyaw = self.home_pose
        return math.hypot(x - hx, y - hy), wrap_angle(hyaw - theta)

    def _start_align(self) -> None:
        self.phase = ALIGNING
        self._align_elapsed = 0.0

    # ---------------------------------------------------------------- step
    def step(self, now: float, has_task: bool, transient: bool,
             pos_err_m: Optional[float] = None, yaw_err_rad: float = 0.0,
             settled: bool = True, has_goal: bool = False,
             battery_pct: float = 100.0, rotation_clear: bool = True,
             dist_home_m: Optional[float] = None) -> Optional[str]:
        """One decision per tick. Returns GO_HOME, BACK_OFF, DOCK or None.

        has_task       : the robot owns an auction task
        transient      : a dwell or a retreat is in progress - wait it out
        pos_err_m      : distance from the believed pose to the dock pose
        yaw_err_rad    : dock yaw minus believed heading (wrapped)
        has_goal       : the coordinator currently has a navigation goal
        rotation_clear : no peer close enough to hit while rotating in place
        dist_home_m    : old name for pos_err_m (position-only callers)
        """
        if pos_err_m is None:
            pos_err_m = dist_home_m if dist_home_m is not None else float("inf")
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now
        self.update_hold(battery_pct)
        self.hold_motion = False
        if self.home_cell is None:
            return None
        if has_task:
            self.reset()
            return None
        if transient:
            return None

        near = pos_err_m <= self.approach_m
        aligned = (pos_err_m <= self.pos_tol_m
                   and abs(yaw_err_rad) <= self.yaw_tol_rad and settled)

        if self.phase == DOCKED:
            # Stay docked unless pushed well off it (a peer's deadlock
            # retreat); then go back through the full approach.
            if pos_err_m <= self.approach_m + 0.2:
                return None
            self.phase = None
            self._idle_since = None
            self.misaligned = False

        if self.phase == BACKING_OFF:
            if self._backoff_since is None:
                self._backoff_since = now
            done = pos_err_m >= self.backoff_m - 0.1
            if done or now - self._backoff_since > self.align_timeout_s:
                self.phase = RETURNING
                self._backoff_since = None
                return GO_HOME
            return None if has_goal else BACK_OFF

        if self.phase == RETURNING:
            if not near:
                return None if has_goal else GO_HOME
            self._start_align()             # fall through into ALIGNING

        if self.phase == ALIGNING:
            if aligned:
                self.phase = DOCKED
                self.misaligned = False
                self.retries = 0
                return DOCK
            # About to turn on the spot: only with nobody within reach.
            turning = (pos_err_m <= 2.0 * self.pos_tol_m
                       and abs(yaw_err_rad) > self.yaw_tol_rad)
            if turning and not rotation_clear:
                self.hold_motion = True     # paused: the clock stops too
                return None
            self._align_elapsed += dt
            if self._align_elapsed > self.align_timeout_s:
                if self.retries < self.max_retries:
                    self.retries += 1
                    self.phase = BACKING_OFF
                    self._backoff_since = now
                    return BACK_OFF
                self.phase = DOCKED         # bounded: charge where it is
                self.misaligned = True
                self.retries = 0
                return DOCK
            return None if has_goal else GO_HOME

        # ---- idle, not yet heading home
        if near:
            if aligned:                     # e.g. at startup, on the spawn
                self.phase = DOCKED
                return DOCK
            self._start_align()             # close but off: square up
            return GO_HOME
        if self._idle_since is None:
            self._idle_since = now
        if self.charge_hold or now - self._idle_since >= self.idle_s:
            self.phase = RETURNING
            return GO_HOME
        return None
