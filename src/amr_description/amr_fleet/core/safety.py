"""
Priority-independent safety envelope (design layer L1).

Lidar cannot see peers (the scan plane is above the chassis) and Nav2's
collision_monitor is scan-only, so this V2V check is the only thing that keeps
two robot bodies apart. It runs every tick for every robot, whatever the
priority order and whatever the zone state. Priority only decides who resumes
first after a mutual stop, through the deadlock layer, which is why every STOP
names its blocking_robot.

  HARD STOP  a peer disc comes within D_stop of me now, or along a
             constant-twist prediction of both robots over the next 1.0 s.
  HARD STOP  a stationary peer within 0.9 m whose disc lies within 0.6 m of my
             next 1.5 m of path, on a stretch that approaches it.
  SLOW       1.0 s < TTC < 3.5 s, scale ttc/3.5 (min 0.25).

Robot-to-robot contact is what wrecks localization in this fleet: the shoved
robot's wheel odometry slips and AMCL follows it.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from . import hitbox
from .geometry import (ROBOT_RADIUS_M, SAFETY_MARGIN_M, extrapolate,
                       time_to_collision)
from .hitbox import (HYST_CLEAR_TICKS, HYST_M, REACH_CAP_M, V_MAX,
                     clean_sigma, gap_stop, is_spinning, rect_gap)
from .models import Permit

# 0.52 m footprint sum + 0.15 m margin, before the 2-sigma localization term.
D_STOP_BASE_M = 2 * ROBOT_RADIUS_M + SAFETY_MARGIN_M
STOP_HORIZON_S = 1.0
PREDICT_STEP_S = 0.05
SLOW_HORIZON_S = 3.5
MIN_SLOW_SCALE = 0.25
STATIONARY_V_MPS = 0.05
STATIONARY_RANGE_M = 0.9
PATH_LOOKAHEAD_M = 1.5
PATH_CLEARANCE_M = 0.6
# A SUSPECT peer's disc grows by v_max x age: it may have driven anywhere
# within that radius since its last state. 1.0 = the fleet's cruise speed.
V_MAX_MPS = 1.0
# Escape from inside D_stop: allowed while neither the joint prediction nor
# my next ESCAPE_LOOKAHEAD_M of path brings me more than ESCAPE_TOL_M closer.
# A dot-product "is the peer ahead" test held a robot forever beside a parked
# peer that sat a few cm ahead of its shoulder.
ESCAPE_LOOKAHEAD_M = 0.5
ESCAPE_TOL_M = 0.02
# A NaN sigma would make every "< D_stop" comparison False and switch the
# envelope off; treat it as the default loc_max_sigma_xy_m gate instead.
NONFINITE_SIGMA_M = 0.6


@dataclass
class PeerDisc:
    """What the envelope needs to know about one alive peer.

    (x, y, theta) is the pose already extrapolated to ``now``
    (PeerRegistry.extrapolated_pose). ``rx_time`` is the receiver-clock
    arrival of its newest state; with ``suspect`` it sets the disc inflation.
    """
    robot_id: str
    x: float
    y: float
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0
    rx_time: float = 0.0
    suspect: bool = False
    loc_sigma_lat: float = 0.0
    # Increment 3/4 (rect envelope): a SILENT peer (5-30 s quiet) is a frozen
    # ghost whose reach grows with age; a spinning peer sweeps its
    # circumscribed disc instead of its rect.
    silent: bool = False
    spinning: bool = False
    # Last-known speed for the stale-reach bound (hitbox.stale_reach) when
    # ``v`` has been zeroed to freeze the prediction (silent ghosts). None
    # means "use v".
    v_last: Optional[float] = None


def _sigma(s) -> float:
    s = float(s or 0.0)
    return max(0.0, s) if math.isfinite(s) else NONFINITE_SIGMA_M


def d_stop(my_sigma: float, peer_sigma: float, inflation: float = 0.0) -> float:
    return (D_STOP_BASE_M + 2.0 * max(_sigma(my_sigma), _sigma(peer_sigma))
            + inflation)


def safety_permit(me, my_path: Sequence, peers: Sequence[PeerDisc], grid,
                  now: float, path_index: Optional[int] = None,
                  v_max: float = V_MAX_MPS) -> Optional[Permit]:
    """Return a STOP or SLOW Permit, or None when the envelope is clear.

    ``me`` is my RobotState (pose, v, w, loc_sigma_lat). ``my_path`` is my
    grid path and ``path_index`` my current index on it (nearest cell if
    None). With no path, only the proximity/prediction rule can stop me, and
    only while the separation is closing.
    """
    if not peers:
        return None
    mx, my, mth = me.pose.x, me.pose.y, me.pose.theta
    mv, mw = me.v, me.w
    my_sigma = me.loc_sigma_lat
    ahead = _path_ahead(my_path, grid, mx, my, path_index)
    near = [(cx, cy) for cx, cy, s in ahead if s <= ESCAPE_LOOKAHEAD_M]
    if not near and ahead:
        near = [ahead[0][:2]]

    n_steps = int(round(STOP_HORIZON_S / PREDICT_STEP_S))
    mine = [extrapolate(mx, my, mth, mv, mw, k * PREDICT_STEP_S)[:2]
            for k in range(n_steps + 1)]

    worst_stop: Optional[Tuple[float, Permit]] = None
    worst_slow: Optional[Permit] = None
    for p in peers:
        infl = v_max * max(0.0, now - p.rx_time) if p.suspect else 0.0
        dstop = d_stop(my_sigma, p.loc_sigma_lat, infl)
        px, py, pth = p.x, p.y, p.theta
        d_now = math.hypot(px - mx, py - my)

        # 1. Predicted separation, now and over the next STOP_HORIZON_S.
        d_min = d_now
        for k in range(1, n_steps + 1):
            bx, by, _ = extrapolate(px, py, pth, p.v, p.w, k * PREDICT_STEP_S)
            ax, ay = mine[k]
            d_min = min(d_min, math.hypot(bx - ax, by - ay))
        if d_min < dstop:
            # Already inside D_stop but not closing, and my path leads away
            # or past it: that is the only way out, so it must stay allowed.
            escaping = (d_min >= d_now - ESCAPE_TOL_M and all(
                math.hypot(cx - px, cy - py) >= d_now - ESCAPE_TOL_M
                for cx, cy in near))
            if not escaping:
                margin = d_min - dstop
                if worst_stop is None or margin < worst_stop[0]:
                    worst_stop = (margin, Permit(
                        action="STOP", speed_scale=0.0,
                        reason=(f"safety: {p.robot_id} predicted "
                                f"{d_min:.2f}m < {dstop:.2f}m "
                                f"within {STOP_HORIZON_S:.1f}s"),
                        blocking_robot=p.robot_id))
                continue

        # 2. Stationary peer on the stretch of path that approaches it. A
        # path leading away never gets closer than I am now, so a robot
        # yielding away from a peer is never blocked by it.
        if abs(p.v) < STATIONARY_V_MPS and d_now - infl <= STATIONARY_RANGE_M:
            for cx, cy, _s in ahead:
                dp = math.hypot(cx - px, cy - py)
                if (dp - infl <= PATH_CLEARANCE_M
                        and dp < d_now - ESCAPE_TOL_M):
                    margin = dp - infl - PATH_CLEARANCE_M
                    if worst_stop is None or margin < worst_stop[0]:
                        worst_stop = (margin, Permit(
                            action="STOP", speed_scale=0.0,
                            reason=(f"safety: stationary {p.robot_id} on my "
                                    f"path {d_now:.2f}m away"),
                            blocking_robot=p.robot_id))
                    break

        # 3. TTC slow band, on the same inflated disc.
        if worst_stop is None:
            ttc, _ = time_to_collision(
                mx, my, mv * math.cos(mth), mv * math.sin(mth),
                px, py, p.v * math.cos(pth), p.v * math.sin(pth),
                r_sum=dstop, margin=0.0, horizon=SLOW_HORIZON_S)
            if ttc is not None and STOP_HORIZON_S < ttc < SLOW_HORIZON_S:
                scale = max(MIN_SLOW_SCALE, ttc / SLOW_HORIZON_S)
                if worst_slow is None or scale < worst_slow.speed_scale:
                    worst_slow = Permit(
                        action="SLOW", speed_scale=scale,
                        reason=f"safety: TTC {ttc:.1f}s with {p.robot_id}",
                        blocking_robot=p.robot_id)

    if worst_stop is not None:
        return worst_stop[1]
    return worst_slow


def most_restrictive(base: Permit, envelope: Optional[Permit]) -> Permit:
    """Combine the zone/TTC permit with the safety envelope's verdict."""
    if envelope is None:
        return base
    if envelope.action == "STOP" or base.action == "GO" or (
            base.action == "SLOW" and envelope.speed_scale < base.speed_scale):
        envelope.zone_id = envelope.zone_id or base.zone_id
        return envelope
    return base


def _path_ahead(path: Sequence, grid, mx: float, my: float,
                path_index: Optional[int]
                ) -> List[Tuple[float, float, float]]:
    """(x, y, distance travelled from my pose) for the path points after the
    current index, up to the lookahead. Converts only the cells it returns."""
    if not path:
        return []
    if path_index is None:
        pts = [grid.cell_to_world(c) for c in path]
        path_index = min(range(len(pts)),
                         key=lambda i: (pts[i][0] - mx) ** 2
                                       + (pts[i][1] - my) ** 2)
    out = []
    travelled = 0.0
    lx, ly = mx, my
    for i in range(path_index + 1, len(path)):
        cx, cy = grid.cell_to_world(path[i])
        travelled += math.hypot(cx - lx, cy - ly)
        if travelled > PATH_LOOKAHEAD_M:
            break
        out.append((cx, cy, travelled))
        lx, ly = cx, cy
    return out


# =========================================================================
# Increment 3/4: oriented-rectangle envelope (replaces the disc D_stop).
# The disc API above is kept verbatim as the legacy path until every caller
# has migrated to envelope_permit; new code must use envelope_permit.
# =========================================================================
ESCAPE_MONO_TOL_M = 0.005       # tolerated gap decrease per path step
ESCAPE_MIN_GAIN_M = 0.03        # the path must end this much clearer
ROTATE_V_MPS = 0.05             # |v| below this counts as rotate-in-place
LANE_PASS_W_MAX = 0.9           # pass-rule spin veto (see envelope_permit)
PASS_HEADING_TOL_RAD = 0.5236   # 30 deg: (anti-)parallel pass tolerance
ROTATE_GAP_FLOOR_M = 0.08       # rotate exception: gap >= max(this, now-0.09)
ROTATE_GAP_DROP_M = 0.09
# Stationary-peer scan horizon along my path: 2.5 s of path at the 1.0 m/s
# cruise (was 1.5 m = 3.75 s at 0.4 m/s; keeping the metres would have cut
# the anticipation to 1.5 s at the new speed).
STATIONARY_PATH_M = 2.5


class SafetyMemory:
    """Per-robot hysteresis state for envelope_permit. Owned and held by the
    coordinator instance and passed in every tick - NO module state, and time
    only ever arrives through `now`.

    After a STOP for peer P the stop threshold against P grows by HYST_M
    until P has been clear for HYST_CLEAR_TICKS consecutive calls, which
    kills the STOP/GO flip-flop two robots otherwise produce at exactly the
    threshold distance. `enabled=False` is for A/B measurement only."""

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self._active: Dict[str, int] = {}     # peer -> consecutive clear ticks

    def active(self, peer_id: str) -> bool:
        return self.enabled and peer_id in self._active

    def note_stop(self, peer_id: str) -> None:
        if self.enabled:
            self._active[peer_id] = 0

    def note_clear(self, peer_id: str) -> None:
        if peer_id in self._active:
            self._active[peer_id] += 1
            if self._active[peer_id] >= HYST_CLEAR_TICKS:
                del self._active[peer_id]

    def forget(self, peer_id: str) -> None:
        self._active.pop(peer_id, None)


def _me_fields(me) -> Tuple[float, float, float, float, float, float]:
    """(x, y, theta, v, w, sigma) from a RobotState-shaped object."""
    pose = me.pose
    degraded = getattr(me, "loc_health", 0) == 1
    return (pose.x, pose.y, pose.theta, me.v, me.w,
            clean_sigma(me.loc_sigma_lat, degraded=degraded))


def _pair_gap(ma: Tuple[float, float, float], a_spin: bool,
              pb: Tuple[float, float, float], b_spin: bool,
              infl: float = 0.0) -> float:
    """Rect/rect gap with spin-disc substitution (r = HB_R_CIRC) for either
    side. `infl` inflates the PEER side (age/ghost reach)."""
    if a_spin and b_spin:
        return hitbox.disc_gap(ma[0], ma[1], hitbox.HB_R_CIRC,
                               pb[0], pb[1], hitbox.HB_R_CIRC + infl)
    if a_spin:
        return hitbox.rect_disc_gap(pb, infl, ma[0], ma[1], hitbox.HB_R_CIRC)
    if b_spin:
        return hitbox.rect_disc_gap(ma, 0.0, pb[0], pb[1],
                                    hitbox.HB_R_CIRC + infl)
    return rect_gap(ma, pb, 0.0, infl, cutoff=2.0)


def _path_poses(path_pts: Sequence[Tuple[float, float]], mx: float, my: float,
                mth: float, horizon_m: float
                ) -> List[Tuple[float, float, float, float]]:
    """(x, y, theta, travelled) along my upcoming path points, heading taken
    from the segment direction (my current heading for the first point)."""
    out: List[Tuple[float, float, float, float]] = []
    travelled = 0.0
    lx, ly, lth = mx, my, mth
    for px, py in path_pts:
        seg = math.hypot(px - lx, py - ly)
        if seg > 1e-9:
            lth = math.atan2(py - ly, px - lx)
        travelled += seg
        if travelled > horizon_m:
            break
        out.append((px, py, lth, travelled))
        lx, ly = px, py
    return out


def envelope_permit(me, path_pts: Sequence[Tuple[float, float]],
                    peers: List[PeerDisc], mem: SafetyMemory,
                    now: float, v_max: float = V_MAX,
                    lane_pass: Optional[Dict[str, float]] = None
                    ) -> Optional[Permit]:
    """STOP/SLOW verdict from wheel-inclusive oriented rects, or None.

    Rules (design doc, increments 3+4):
      STOP   predicted rect gap < g_ij within 1.0 s of constant-twist motion
             (0.05 s steps), unless the map-frame gap along my next path poses
             is monotone non-decreasing (tol 0.005) and ends >= 0.03 clearer
             (the escape rule), or the rotate-in-place exception holds.
      STOP   stationary peer whose rect comes within g_ij of my rect anywhere
             on my next 1.5 m of path, on an approaching stretch.
      SLOW   first breach between 1.0 s and 3.5 s: scale t/3.5, min 0.25.
      g_ij = gap_stop(sig_i, sig_j, v_close) + v_max*age (SUSPECT/SILENT,
             capped REACH_CAP_M) + HYST_M while hysteresis is active.
    Every STOP names blocking_robot. `path_pts` are MAP-FRAME points of my
    path AHEAD of me (empty when idle).

    LANE-PASS exception (webots_final4 fix: two 0.41 m robots MUST be able
    to pass abreast in a 1.6 m corridor). `lane_pass` maps peer ids the
    CALLER attests are lane-keeping with me in a two-lane corridor (both in
    the band, both heading along the corridor axis, neither turning - see
    coordinator._lane_pass_peers) to the corridor travel-axis angle. For
    such a peer the verdict is judged on the LATERAL rect gap only
    (hitbox.lateral_gap), against the sigma-only threshold
    gap_stop(sig_i, sig_j, v_close=0) + age: the longitudinal closing speed
    of an opposing parallel pass cannot close the lateral gap, so it is not
    charged the G_MARGIN_VCLOSE term that made the omnidirectional rule
    stop every pass (0.25 + 0.141 = 0.39 m >= the 0.39 m lane gap even at
    the sigma floor). The lateral gap must clear the threshold now AND over
    the whole 1.0 s joint prediction; otherwise the full omnidirectional
    rule below applies unchanged, as it always does when either robot is
    spinning/turning, SUSPECT or SILENT."""
    if not peers:
        return None
    mx, my, mth, mv, mw, msig = _me_fields(me)
    my_spin = is_spinning(mw, path_pts)

    n_slow = int(round(SLOW_HORIZON_S / PREDICT_STEP_S))
    n_stop = int(round(STOP_HORIZON_S / PREDICT_STEP_S))
    mine = [extrapolate(mx, my, mth, mv, mw, k * PREDICT_STEP_S)
            for k in range(n_slow + 1)]
    ahead = _path_poses(path_pts, mx, my, mth, STATIONARY_PATH_M)

    worst_stop: Optional[Tuple[float, Permit]] = None
    worst_slow: Optional[Permit] = None
    for p in peers:
        stale = p.suspect or p.silent
        # Reach bounded by the peer's OWN last-known motion (hitbox.
        # stale_reach): a peer last seen stationary does not grow a V_MAX
        # bubble - the measured degraded-comms freeze (robots stopping each
        # other at > 1.3 m while heartbeats ran at 3-4 Hz).
        age_infl = 0.0
        if stale:
            v_ref = p.v_last if p.v_last is not None else p.v
            reach = getattr(hitbox, "stale_reach", None)
            age = max(0.0, now - p.rx_time)
            age_infl = (reach(v_ref, age) if callable(reach)
                        else min(REACH_CAP_M, v_max * age))
        p_spin = p.spinning or abs(p.w) > hitbox.SPIN_W_RAD_S

        # PARALLEL-PASS exception (road model: every corridor is a two-way
        # street). Judged on the LATERAL gap only, with the sigma-only
        # threshold, over the full joint 1.0 s prediction. It applies when
        # I am driving straight (no sharp path turn within TURN_AHEAD_M,
        # |my w| bounded) against a non-stale peer that is
        #   - attested lane-keeping by the caller (lane_pass: band axis), or
        #   - stationary and not rotating (a parked robot's only gap that
        #     matters along a straight drive past it is lateral), or
        #   - moving (anti-)parallel to me within PASS_HEADING_TOL_RAD
        #     (axis = my heading).
        # The w veto is LANE_PASS_W_MAX (0.9 rad/s), looser than
        # SPIN_W_RAD_S: at the 1.0 m/s cruise lane-keeping corrections
        # spike |w| past 0.3 every few ticks and flickered the pass into a
        # STOP/GO dither (measured). Safe because the guard is the
        # PREDICTED lateral gap, whose constant-twist extrapolation
        # includes both w's: a robot actually yawing toward the other
        # fails the prediction and the full omnidirectional rule below
        # applies unchanged - as it always does for stale peers, flagged
        # path-turns (p.spinning) and genuine rotation.
        axis: Optional[float] = None
        if (not stale and not p.spinning
                and abs(mw) <= LANE_PASS_W_MAX
                and not is_spinning(0.0, path_pts)):
            if lane_pass and p.robot_id in lane_pass:
                if abs(p.w) <= LANE_PASS_W_MAX:
                    axis = float(lane_pass[p.robot_id])
            elif (abs(p.v) < STATIONARY_V_MPS
                  and abs(p.w) <= hitbox.SPIN_W_RAD_S):
                axis = mth
            elif (abs(p.w) <= LANE_PASS_W_MAX
                  and abs(p.v) >= STATIONARY_V_MPS):
                # Only a peer actually DRIVING (anti-)parallel qualifies. A
                # robot turning on the spot (v ~ 0, |w| > SPIN_W) sweeps its
                # 0.286 m disc - wider than the rect whose lateral gap this
                # branch would judge - so it falls through to the full rule.
                dh = math.atan2(math.sin(p.theta - mth),
                                math.cos(p.theta - mth))
                if min(abs(dh), math.pi - abs(dh)) <= PASS_HEADING_TOL_RAD:
                    axis = mth
        if axis is not None:
            g_lat = gap_stop(msig, p.loc_sigma_lat, 0.0, age_infl)
            lat_ok = True
            for k in range(0, n_stop + 1):
                pb = extrapolate(p.x, p.y, p.theta, p.v, p.w,
                                 k * PREDICT_STEP_S)
                if hitbox.lateral_gap(mine[k], pb, axis) < g_lat:
                    lat_ok = False
                    break
            if lat_ok:
                mem.note_clear(p.robot_id)
                continue

        def pair(ma, pb):
            return _pair_gap(ma, my_spin, pb, p_spin)

        # Joint constant-twist prediction. A silent ghost is frozen (v=0 was
        # set by silent_ghost_discs; its reach is in age_infl via g).
        gap_now = pair(mine[0], (p.x, p.y, p.theta))
        min_gap, t_min = gap_now, 0.0
        gaps = [gap_now]
        for k in range(1, n_slow + 1):
            pb = extrapolate(p.x, p.y, p.theta, p.v, p.w, k * PREDICT_STEP_S)
            gk = pair(mine[k], pb)
            gaps.append(gk)
            if k <= n_stop and gk < min_gap:
                min_gap, t_min = gk, k * PREDICT_STEP_S

        # v_close is the RECT-GAP closing rate toward the predicted minimum,
        # not the centre-to-centre closing speed: two robots passing in
        # offset lanes close fast in centre distance while their rect gap
        # barely shrinks, and must not be charged the closing margin.
        v_close = ((gap_now - min_gap) / t_min) if t_min > 0.0 else 0.0
        g = gap_stop(msig, p.loc_sigma_lat, v_close, age_infl)
        # Hysteresis kills the STOP/GO flip-flop two MOVING robots produce
        # at exactly the threshold. Against a STATIONARY peer the geometry
        # is entirely under my control, and the extra 0.10 m made the
        # maximum pass separation a 1.6 m aisle offers (~0.90 m, rect gap
        # 0.44-0.49) un-drivable once a single STOP had latched it - the
        # measured 'rerouted clear of X' -> 'no progress' freeze.
        # A spinning robot is NOT a static obstacle (its swept disc moves),
        # so it gets none of the stationary relaxations.
        p_stationary = abs(p.v) < STATIONARY_V_MPS and not p_spin
        g_eff = g + (HYST_M if mem.active(p.robot_id) and not p_stationary
                     else 0.0)
        t_breach: Optional[float] = None
        for k in range(1, n_slow + 1):
            if gaps[k] < g:
                t_breach = k * PREDICT_STEP_S
                break

        stopped_for_p = False
        if min_gap < g_eff:
            if not _escape_ok(ahead, gap_now, min_gap, mv, my_spin,
                              (p.x, p.y, p.theta), p_spin,
                              stationary=p_stationary, g_eff=g_eff):
                stopped_for_p = True
                margin = min_gap - g_eff
                if worst_stop is None or margin < worst_stop[0]:
                    worst_stop = (margin, Permit(
                        action="STOP", speed_scale=0.0,
                        reason=(f"safety: {p.robot_id} rect gap "
                                f"{min_gap:.2f}m < {g_eff:.2f}m "
                                f"within {STOP_HORIZON_S:.1f}s"),
                        blocking_robot=p.robot_id))

        # Stationary peer on the approaching stretch of my next 1.5 m.
        # (age inflation lives in the threshold only - inflating the rect too
        # would double-count it). Two deliberate differences from g_eff:
        # - no v_close charge: against a STATIC object the closing speed is
        #   entirely my own and the SLOW band already brakes me; charging it
        #   demanded ~0.97 m of path clearance, more than a 1.6 m aisle can
        #   give, so every in-aisle pass of a parked robot froze (measured);
        # - the stationary-escape exemption: without it a robot interlocked
        #   with a parked peer was frozen FOREVER even when its (retreat)
        #   path led directly away - the rect swing while turning out dipped
        #   the gap below gap_now and the measured seed-7 standoff replayed
        #   to the end of the run.
        g_st = gap_stop(msig, p.loc_sigma_lat, 0.0, age_infl)
        if not stopped_for_p and abs(p.v) < STATIONARY_V_MPS:
            # A spinning peer (v=0, |w| high) still gets the path scan, but
            # with the full moving-grade threshold and no escape exemption:
            # its swept disc is not static geometry.
            g_path = g_st if p_stationary else g_eff
            exempt = (p_stationary
                      and _escape_ok(ahead, gap_now, min_gap, mv, my_spin,
                                     (p.x, p.y, p.theta), p_spin,
                                     stationary=True, g_eff=g_st))
            if not exempt:
                for px, py, pth, _trav in ahead:
                    gp = _pair_gap((px, py, pth), my_spin,
                                   (p.x, p.y, p.theta), p_spin)
                    if gp < g_path and gp < gap_now - ESCAPE_MONO_TOL_M:
                        stopped_for_p = True
                        margin = gp - g_path
                        if worst_stop is None or margin < worst_stop[0]:
                            worst_stop = (margin, Permit(
                                action="STOP", speed_scale=0.0,
                                reason=(f"safety: stationary {p.robot_id} on "
                                        f"my path, rect gap {gp:.2f}m"),
                                blocking_robot=p.robot_id))
                        break

        if not stopped_for_p:
            mem.note_clear(p.robot_id)
            if (t_breach is not None
                    and STOP_HORIZON_S < t_breach <= SLOW_HORIZON_S):
                scale = max(MIN_SLOW_SCALE, t_breach / SLOW_HORIZON_S)
                if worst_slow is None or scale < worst_slow.speed_scale:
                    worst_slow = Permit(
                        action="SLOW", speed_scale=scale,
                        reason=(f"safety: rect gap breaches in "
                                f"{t_breach:.1f}s with {p.robot_id}"),
                        blocking_robot=p.robot_id)
        else:
            mem.note_stop(p.robot_id)

    if worst_stop is not None:
        return worst_stop[1]
    return worst_slow


def _escape_ok(ahead, gap_now: float, min_gap: float, mv: float,
               my_spin: bool, peer_pose, p_spin: bool,
               stationary: bool = False, g_eff: float = 0.0) -> bool:
    """May I keep moving while inside the (effective) threshold?

    MAP-FRAME monotone escape: the rect gap evaluated at my next path poses
    (peer frozen where it is) never decreases by more than ESCAPE_MONO_TOL_M
    per step and ends at least ESCAPE_MIN_GAIN_M above the start. Driving
    away is the only exit from a standoff, so it must stay allowed.

    Rotate-in-place exception: nearly stationary (|v| < ROTATE_V_MPS), path
    not approaching the peer, and the swept gap stays above
    max(ROTATE_GAP_FLOOR_M, gap_now - ROTATE_GAP_DROP_M): turning toward the
    exit dips the predicted gap only because the rect sweeps, so it must not
    re-trigger the STOP that the robot is trying to resolve."""
    gaps = [gap_now]
    for px, py, pth, _trav in ahead:
        gaps.append(_pair_gap((px, py, pth), my_spin, peer_pose, p_spin))
    approaching = any(g < gap_now - ESCAPE_MONO_TOL_M for g in gaps[1:])
    if abs(mv) < ROTATE_V_MPS and not approaching:
        if min_gap >= max(ROTATE_GAP_FLOOR_M, gap_now - ROTATE_GAP_DROP_M):
            return True
    if len(gaps) < 2:
        return False
    # STATIONARY peer: it cannot close the gap, so my own poses control it
    # completely. A bounded dip is tolerated (the rect swings while turning
    # out of the standoff) as long as every pose keeps the sigma/age part of
    # the threshold (g_eff minus the base standoff) and the path genuinely
    # leaves. Without this, two interlocked robots below g_eff could NEVER
    # separate: any first motion dips the swept gap, the monotone rule
    # vetoed it, and the measured seed-7 standoff froze for 375 s.
    if stationary and g_eff > 0.0:
        floor = max(ROTATE_GAP_FLOOR_M, g_eff - hitbox.G_MARGIN_BASE)
        if (all(g >= floor for g in gaps[1:])
                and gaps[-1] >= gaps[0] + ESCAPE_MIN_GAIN_M):
            return True
    for a, b in zip(gaps, gaps[1:]):
        if b < a - ESCAPE_MONO_TOL_M:
            return False
    return gaps[-1] >= gaps[0] + ESCAPE_MIN_GAIN_M


def silent_ghost_discs(registry, now: float, grid=None) -> List[PeerDisc]:
    """SILENT peers (5-30 s quiet, core/peers.py tiers) as envelope ghosts.

    Each ghost is the peer FROZEN at its last known pose (v = w = 0,
    silent=True): envelope_permit then grows its threshold by v_max * state
    age, capped at REACH_CAP_M - the 'frozen rect + reach' rule. With `grid`,
    extra ghost points are laid along the peer's last broadcast intent path
    out to that same reach (rx_time=now so they are not double-inflated),
    because a silent robot that kept driving is most likely ON its last
    announced route. The contract signature is (registry, now); `grid` is an
    optional extension needed to convert intent cells to map points."""
    out: List[PeerDisc] = []
    for rid in registry.silent(now):
        st = registry.peers.get(rid)
        if st is None:
            continue
        age = max(0.0, now - registry.last_seen.get(rid, st.rx_time))
        base = PeerDisc(robot_id=rid, x=st.pose.x, y=st.pose.y,
                        theta=st.pose.theta, v=0.0, w=0.0,
                        rx_time=registry.last_seen.get(rid, st.rx_time),
                        suspect=True, loc_sigma_lat=st.loc_sigma_lat,
                        silent=True, v_last=getattr(st, "v", None))
        out.append(base)
        if grid is None:
            continue
        held = registry.intents.get(rid)
        cells = held.intent.cells if held is not None else []
        if not cells:
            continue
        reach_fn = getattr(hitbox, "stale_reach", None)
        reach = (reach_fn(getattr(st, "v", None), age) if callable(reach_fn)
                 else min(REACH_CAP_M, V_MAX * age))
        if reach <= 0.0:
            continue
        # Walk the intent from the cell nearest the frozen pose.
        pts = [grid.cell_to_world(c) for c in cells]
        i0 = min(range(len(pts)),
                 key=lambda i: (pts[i][0] - st.pose.x) ** 2
                               + (pts[i][1] - st.pose.y) ** 2)
        travelled = 0.0
        last_emit = 0.0
        lx, ly = pts[i0]
        for px, py in pts[i0 + 1:]:
            travelled += math.hypot(px - lx, py - ly)
            lx, ly = px, py
            if travelled > reach:
                break
            if travelled - last_emit >= 0.3 or travelled >= reach - 1e-9:
                out.append(PeerDisc(
                    robot_id=rid, x=px, y=py, theta=st.pose.theta,
                    v=0.0, w=0.0, rx_time=now, suspect=False,
                    loc_sigma_lat=st.loc_sigma_lat, silent=True))
                last_emit = travelled
    return out
