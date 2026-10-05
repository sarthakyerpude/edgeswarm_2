"""
Oriented-rectangle hitbox for the 0.40 x 0.32 m chassis (R2).

The SAFETY rectangle is WHEEL-INCLUSIVE: the chassis is 0.40 x 0.32 m but the
drive wheels protrude to |y| = 0.205 m in the body frame (warehouse.wbt), so
the rect every safety decision uses is half-length 0.20 m, half-width 0.205 m.
The old 0.26 m disc model (geometry.ROBOT_RADIUS_M) made every 1.6 m aisle
behave single-file; the rect is what unlocks two-abreast passing:
two robots at +-0.4 m on the 1.6 m spine have a true rect gap of
0.8 - 2*0.205 = 0.39 m, against a disc gap of 0.8 - 0.52 = 0.28 m.

Everything here is pure geometry: no state, no clocks, no ROS. The hysteresis
and prediction logic that USES these predicates lives in core/safety.py.

All headings are radians in the map frame; +x of the body frame is the front.
"""
import math
from typing import List, Optional, Sequence, Set, Tuple

# ----------------------------------------------------------------- contract --
# (shared API contract; other builders import these names)
HB_HALF_L = 0.20            # half-length, metres (chassis)
HB_HALF_W = 0.205           # half-width, metres (WHEEL-INCLUSIVE)
HB_CHASSIS_HALF_W = 0.16    # chassis-only half-width (A* static feasibility)
HB_R_CIRC = 0.286           # circumscribed radius of the safety rect
G_MARGIN_BASE = 0.12        # pairwise stop margin at v_close = 0
# Closing-speed margin, re-sized for the 1.0 m/s fleet (was 0.13 saturating
# at v_close 0.4, sized for the 0.4 m/s cruise). The ANTICIPATION against a
# predicted approach lives in safety.STOP_HORIZON_S (1.0 s of joint
# constant-twist prediction = v_close metres of lookahead, which scales
# with speed by construction); this margin covers only the UNPREDICTED
# residual: decision latency (0.1 s tick + ~0.1 s actuation) + 0.2 s
# reaction at the closing speed a 2.0 m/s^2 peer can add unmodelled within
# that time (~0.8 m/s) ~= 0.3 m. Reference closing speed = cruise (1.0):
# at the old operating point (v_close 0.4) the margin is 0.30*0.4 = 0.12,
# within 0.01 of the old 0.13 - low-speed behaviour is unchanged.
G_MARGIN_VCLOSE = 0.30      # extra margin at v_close >= V_CLOSE_REF_MPS
V_CLOSE_REF_MPS = 1.0       # saturation reference (= cruise speed)
K_SIG = 2.0                 # sigma multiplier in the stop threshold
SIGMA_FLOOR = 0.05          # never trust a sigma below this
SIGMA_DEGRADED_MIN = 0.20   # LOC_DEGRADED floor
SIGMA_NONFINITE = 0.60      # NaN/inf sigma -> the loc gate value
HYST_M = 0.10               # threshold bonus after a STOP (anti-flip)
HYST_CLEAR_TICKS = 5        # consecutive clear ticks to drop the bonus
SPIN_W_RAD_S = 0.3          # |w| above this counts as spinning
TURN_AHEAD_RAD = 0.7853981634   # 45 deg: path turn that counts as a spin
TURN_AHEAD_M = 0.8          # ...within this much path (0.8 s at cruise)
V_MAX = 1.0                 # m/s, fleet speed cap (stale-peer reach rate)
REACH_CAP_M = 1.5           # cap on v_max * age inflation / ghost reach
# Stale-peer reach bounded by the peer's OWN last-known motion (degraded-
# comms fix): at 1.0 m/s cruise a blanket V_MAX*age froze robots > 1.3 m
# apart as soon as heartbeats degraded to 3-4 Hz (age ~1.6 s -> +1.5 m on
# the threshold, measured live: 'rect gap 1.35m < 1.56m', zero deliveries).
# A peer LAST SEEN STATIONARY must first decide to move and accelerate
# (A_MAX_MPS2), and its next heartbeat - even a degraded 3-4 Hz one -
# arrives long before it covers real distance, so its reach is
# 0.5*A_MAX*age^2 capped at REACH_STATIC_CAP_M; it also stays a planner
# obstacle and zone occupant regardless. A peer last seen MOVING keeps the
# kinematic bound v_last*... accelerating to V_MAX, capped at REACH_CAP_M.
A_MAX_MPS2 = 2.0            # fleet accel cap (stale-reach kinematics)
REACH_STATIC_CAP_M = 0.3    # reach cap for a peer last seen stationary...
STATIC_TRUST_S = 2.0        # ...trusted only this long: a robot silent for
                            # longer has had time to decide AND drive (the
                            # sih-57 silent-but-driving case), so past the
                            # trust window the reach grows from rest at
                            # A_MAX toward the full REACH_CAP_M again.
STATIC_V_MPS = 0.05         # |v| below this counts as last-seen-stationary

_SQRT2 = math.sqrt(2.0)

Pose = Tuple[float, float, float]           # (x, y, theta)


# ---------------------------------------------------------------- sigmas ----
def clean_sigma(s: float, degraded: bool = False) -> float:
    """Floor/repair a broadcast lateral sigma before it enters any threshold.

    Non-finite -> SIGMA_NONFINITE (a NaN must widen, never disable, the
    envelope); otherwise floored at SIGMA_FLOOR, and at SIGMA_DEGRADED_MIN
    when the sender reports LOC_DEGRADED. Idempotent."""
    try:
        s = float(s)
    except (TypeError, ValueError):
        return SIGMA_NONFINITE
    if not math.isfinite(s):
        return SIGMA_NONFINITE
    floor = SIGMA_DEGRADED_MIN if degraded else SIGMA_FLOOR
    return max(floor, s)


def gap_stop(sig_a: float, sig_b: float, v_close: float = 0.0,
             infl: float = 0.0) -> float:
    """Pairwise rect-gap stop threshold g_ij.

    g = G_MARGIN_BASE + G_MARGIN_VCLOSE * min(1, v_close/V_CLOSE_REF_MPS)
        + K_SIG * sqrt(sa^2 + sb^2) + infl
    where infl carries the v_max*age term for SUSPECT/SILENT peers (already
    capped by the caller at REACH_CAP_M). Sigmas are cleaned here, so passing
    raw broadcast values is safe."""
    sa = clean_sigma(sig_a)
    sb = clean_sigma(sig_b)
    vc = min(1.0, max(0.0, v_close) / V_CLOSE_REF_MPS)
    return (G_MARGIN_BASE + G_MARGIN_VCLOSE * vc
            + K_SIG * math.sqrt(sa * sa + sb * sb) + infl)


def stale_reach(v_last: Optional[float], age_s: float) -> float:
    """How far a stale (SUSPECT/SILENT) peer can have driven since its last
    state, bounded by ITS OWN last-known speed and the fleet accel cap.

    last seen stationary: min(REACH_STATIC_CAP_M, 0.5 * A_MAX * age^2)
    while age <= STATIC_TRUST_S; past the trust window the cap re-grows
    from rest at A_MAX toward REACH_CAP_M (a long-silent 'stationary'
    robot may have long since driven off - sih-57).
    last seen moving (or unknown v): accelerate from min(|v|, V_MAX) to
    V_MAX at A_MAX, then cruise; capped at REACH_CAP_M."""
    age = max(0.0, float(age_s))
    try:
        v0 = abs(float(v_last))
    except (TypeError, ValueError):
        v0 = V_MAX
    if not math.isfinite(v0):
        v0 = V_MAX
    if v0 < STATIC_V_MPS:
        d = 0.5 * A_MAX_MPS2 * age * age
        if age <= STATIC_TRUST_S:
            return min(REACH_STATIC_CAP_M, d)
        over = age - STATIC_TRUST_S
        return min(REACH_CAP_M,
                   REACH_STATIC_CAP_M + 0.5 * A_MAX_MPS2 * over * over)
    v0 = min(v0, V_MAX)
    t_acc = (V_MAX - v0) / A_MAX_MPS2
    if age <= t_acc:
        d = v0 * age + 0.5 * A_MAX_MPS2 * age * age
    else:
        d = (v0 * t_acc + 0.5 * A_MAX_MPS2 * t_acc * t_acc
             + V_MAX * (age - t_acc))
    return min(REACH_CAP_M, d)


def per_robot_inflation(sigma: float, age_s: float = 0.0,
                        stale: bool = False) -> float:
    """One robot's share of the stop threshold, for drawing and rasterising.

    Chosen so that two drawn safety outlines touch at EXACTLY the pairwise
    threshold g when both sigmas are equal (v_close = 0):
        2 * (G_MARGIN_BASE/2 + K_SIG*s/sqrt(2)) = G_MARGIN_BASE + K_SIG*s*sqrt2
    A stale pose (SUSPECT/SILENT) additionally grows by v_max*age, capped."""
    infl = 0.5 * G_MARGIN_BASE + (K_SIG / _SQRT2) * clean_sigma(sigma)
    if stale:
        infl += min(REACH_CAP_M, V_MAX * max(0.0, age_s))
    return infl


# -------------------------------------------------------------- rectangle ---
def corners(x: float, y: float, th: float,
            infl: float = 0.0) -> List[Tuple[float, float]]:
    """The 4 map-frame corners of the (inflated) safety rect.

    Order: front-left, front-right, rear-right, rear-left (+x is front)."""
    hl, hw = HB_HALF_L + infl, HB_HALF_W + infl
    c, s = math.cos(th), math.sin(th)
    return [(x + c * hl - s * hw, y + s * hl + c * hw),
            (x + c * hl + s * hw, y + s * hl - c * hw),
            (x - c * hl + s * hw, y - s * hl - c * hw),
            (x - c * hl - s * hw, y - s * hl + c * hw)]


def _extent(hl: float, hw: float, th: float, ux: float, uy: float) -> float:
    """Projection half-extent of an oriented rect onto unit axis (ux, uy)."""
    c, s = math.cos(th), math.sin(th)
    return hl * abs(ux * c + uy * s) + hw * abs(-ux * s + uy * c)


def rect_gap(a: Pose, b: Pose, infl_a: float = 0.0, infl_b: float = 0.0,
             cutoff: Optional[float] = None) -> float:
    """Separation between two (inflated) safety rects by SAT; 0.0 on overlap.

    The value is the LARGEST separation over the 4 face axes - a lower bound
    on the true Euclidean gap (exact when any faces are parallel, slightly
    conservative corner-to-corner), which is the safe direction for a stop
    threshold. With `cutoff`, far pairs return the cheap circumscribed-circle
    lower bound (centre distance - 2 circ radii) without running SAT."""
    ax, ay, ath = a
    bx, by, bth = b
    dx, dy = bx - ax, by - ay
    d = math.hypot(dx, dy)
    lb = d - (HB_R_CIRC + infl_a) - (HB_R_CIRC + infl_b)
    if cutoff is not None and lb > cutoff:
        return lb
    hla, hwa = HB_HALF_L + infl_a, HB_HALF_W + infl_a
    hlb, hwb = HB_HALF_L + infl_b, HB_HALF_W + infl_b
    ca, sa = math.cos(ath), math.sin(ath)
    cb, sb = math.cos(bth), math.sin(bth)
    best = -math.inf
    for ux, uy in ((ca, sa), (-sa, ca), (cb, sb), (-sb, cb)):
        ea = _extent(hla, hwa, ath, ux, uy)
        eb = _extent(hlb, hwb, bth, ux, uy)
        sep = abs(ux * dx + uy * dy) - ea - eb
        if sep > best:
            best = sep
    return max(0.0, best)


def lateral_gap(a: Pose, b: Pose, axis_theta: float) -> float:
    """Separation of two safety rects along the LATERAL axis of a corridor
    whose travel direction is axis_theta (map-frame radians): the projection
    gap onto the travel axis' unit normal; 0.0 on lateral overlap.

    For two lane-keeping robots passing in parallel lanes this one axis is
    the only one that can close, so the lane-pass rule in safety.py judges
    the pass by THIS gap against the sigma-only threshold. It is exact (not
    a bound) in that direction; the true Euclidean gap is >= this value, so
    a threshold kept on the lateral gap keeps the bodies apart."""
    ux, uy = -math.sin(axis_theta), math.cos(axis_theta)
    ea = _extent(HB_HALF_L, HB_HALF_W, a[2], ux, uy)
    eb = _extent(HB_HALF_L, HB_HALF_W, b[2], ux, uy)
    return max(0.0, abs(ux * (b[0] - a[0]) + uy * (b[1] - a[1])) - ea - eb)


def point_rect_gap(p: Pose, infl: float, px: float, py: float) -> float:
    """Distance from a map point to the (inflated) rect boundary; 0 inside."""
    x, y, th = p
    c, s = math.cos(th), math.sin(th)
    u = c * (px - x) + s * (py - y)
    v = -s * (px - x) + c * (py - y)
    du = max(0.0, abs(u) - (HB_HALF_L + infl))
    dv = max(0.0, abs(v) - (HB_HALF_W + infl))
    return math.hypot(du, dv)


def rect_disc_gap(p: Pose, infl: float, cx: float, cy: float,
                  r: float) -> float:
    """Gap between an (inflated) rect and a disc; 0.0 on overlap."""
    return max(0.0, point_rect_gap(p, infl, cx, cy) - r)


def disc_gap(ax: float, ay: float, ra: float,
             bx: float, by: float, rb: float) -> float:
    return max(0.0, math.hypot(bx - ax, by - ay) - ra - rb)


# ----------------------------------------------------------------- spin -----
def is_spinning(w: float, path_pts: Optional[Sequence[Tuple[float, float]]]
                ) -> bool:
    """A robot rotating in place, or about to turn sharply, sweeps its
    circumscribed disc rather than its rect: |w| > SPIN_W_RAD_S, or the path
    heading changes by more than TURN_AHEAD_RAD within TURN_AHEAD_M."""
    if abs(w) > SPIN_W_RAD_S:
        return True
    if not path_pts or len(path_pts) < 3:
        return False
    h0 = None
    travelled = 0.0
    lx, ly = path_pts[0]
    for px, py in path_pts[1:]:
        seg = math.hypot(px - lx, py - ly)
        if seg < 1e-9:
            continue
        h = math.atan2(py - ly, px - lx)
        if h0 is None:
            h0 = h
        else:
            dh = (h - h0 + math.pi) % (2.0 * math.pi) - math.pi
            if abs(dh) > TURN_AHEAD_RAD:
                return True
        travelled += seg
        if travelled > TURN_AHEAD_M:
            break
        lx, ly = px, py
    return False


# ------------------------------------------------------------- rasterise ----
def cells(grid, x: float, y: float, th: float,
          extra_m: float) -> Set[Tuple[int, int]]:
    """Grid cells whose CENTRE lies inside the rect grown by extra_m plus
    half a cell diagonal (so a cell partially covered by the grown rect is
    still included). This replaces pose-centre point tests for A* peer
    obstacles, ranked-zone occupancy and make-way target validity."""
    half_diag = 0.5 * grid.resolution * _SQRT2
    hl = HB_HALF_L + extra_m + half_diag
    hw = HB_HALF_W + extra_m + half_diag
    reach = math.hypot(hl, hw)
    c, s = math.cos(th), math.sin(th)
    r_lo, c_lo = grid.world_to_cell(x - reach, y - reach)
    r_hi, c_hi = grid.world_to_cell(x + reach, y + reach)
    out: Set[Tuple[int, int]] = set()
    for r in range(max(0, r_lo), min(grid.height - 1, r_hi) + 1):
        for cc in range(max(0, c_lo), min(grid.width - 1, c_hi) + 1):
            cx, cy = grid.cell_to_world((r, cc))
            u = c * (cx - x) + s * (cy - y)
            v = -s * (cx - x) + c * (cy - y)
            if abs(u) <= hl and abs(v) <= hw:
                out.add((r, cc))
    return out


def uturn_swept_cells(grid, x0: float, y0: float, theta0: float,
                      x1: float, y1: float, theta1: float,
                      extra_m: float) -> Set[Tuple[int, int]]:
    """Cells swept by the oriented safety rect TRANSLATING (x0,y0)->(x1,y1)
    while YAWING theta0->theta1 (shortest arc), each half-extent grown by
    extra_m: the union of the rasterised rect over a finely sampled screw
    motion. Translation is sampled every half cell and rotation every
    ~0.15 rad, so the rect boundary moves less than the half-cell-diagonal
    slack cells() already adds between samples - the union is conservative,
    never leaky. Pure rotation (x0==x1, y0==y1) and pure translation
    (theta0==theta1) are the natural special cases.

    This is the static-fit primitive for every lane-crossing manoeuvre:
    traffic.uturn_gate sweeps lane -> pivot -> lane with it (replacing the
    old pivot-disc approximation), traffic.overtake_gate sweeps the borrowed
    opposing-lane stretch, and the planner's turn-shaping tests verify every
    planned >= 90 deg corner against it."""
    dx, dy = x1 - x0, y1 - y0
    dist = math.hypot(dx, dy)
    dth = math.atan2(math.sin(theta1 - theta0), math.cos(theta1 - theta0))
    n = max(1, int(math.ceil(max(dist / (0.5 * grid.resolution),
                                 abs(dth) / 0.15))))
    out: Set[Tuple[int, int]] = set()
    for k in range(n + 1):
        f = k / n
        out |= cells(grid, x0 + f * dx, y0 + f * dy, theta0 + f * dth,
                     extra_m)
    return out


# PLANNING-only cap on the 2*sigma term of the A*-obstacle inflation. At the
# SPINE-fallback sigma of 0.2 the uncapped term is 0.4 m, i.e. a half-width of
# 0.205 + 0.10 + 0.4 = 0.705 m: ONE parked robot then walls off a whole 1.6 m
# aisle for A* (nothing is plannable past it - the measured REPLAN FAILED
# storm, webots_final.log). Capped at 0.2 m the worst planning half-width is
# 0.505 m, which always leaves a plannable thread beside a parked robot in a
# 1.6 m aisle. The cap is safe against the measured 0.47-0.66 m max loc error
# because it bounds PLANNING optimism only: execution is still guarded by the
# safety envelope, which keeps the full uncapped K_SIG*sigma threshold on the
# live pairwise gap. A plan that proves too close simply stops at the envelope
# and replans; an unplannable corridor was unrecoverable.
PLAN_SIGMA_INFL_CAP_M = 0.2


def obstacle_cells(grid, x: float, y: float, th: float,
                   sigma: float) -> Set[Tuple[int, int]]:
    """A* hard-obstacle footprint of a SILENT/FAULTED/stationary peer:
    its wheel-inclusive rect grown by 0.10 m + 2*sigma, with the sigma term
    capped at PLAN_SIGMA_INFL_CAP_M (planning only - see above).
    Replaces the old 0.55 m disc around the peer's pose centre."""
    extra = 0.10 + min(2.0 * clean_sigma(sigma), PLAN_SIGMA_INFL_CAP_M)
    return cells(grid, x, y, th, extra)


# ------------------------------------------------------------------ RViz ----
# Chassis half-width is 0.16 m; the wheel bumps reach 0.205 m between
# x = -0.09 and +0.09 (drawn only - the SAFETY rect already includes them).
_WHEEL_X = 0.09


def _tf(x: float, y: float, th: float,
        pts: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    c, s = math.cos(th), math.sin(th)
    return [(x + c * u - s * v, y + s * u + c * v) for u, v in pts]


def marker_geometry(x: float, y: float, th: float, sigma: float) -> dict:
    """Everything the RViz node needs to draw one robot, in map frame:

      body       closed polyline: chassis outline with wheel bumps
      front_edge the 2 chassis front corners (midpoint == pose + 0.20*u)
      arrow      centre -> 0.35 m along the heading
      nose       filled heading triangle (3 pts)
      safety     closed polyline: rect inflated by per_robot_inflation(sigma)
    """
    hl, hw, ww = HB_HALF_L, HB_CHASSIS_HALF_W, HB_HALF_W
    wx = _WHEEL_X
    body = [(hl, hw), (hl, -hw),
            (wx, -hw), (wx, -ww), (-wx, -ww), (-wx, -hw),
            (-hl, -hw), (-hl, hw),
            (-wx, hw), (-wx, ww), (wx, ww), (wx, hw), (hl, hw)]
    front = [(hl, hw), (hl, -hw)]
    nose = [(hl - 0.01, 0.0), (0.05, 0.09), (0.05, -0.09)]
    arrow = [(0.0, 0.0), (0.35, 0.0)]
    infl = per_robot_inflation(sigma)
    safety = corners(0.0, 0.0, 0.0, infl)
    safety.append(safety[0])
    return {"body": _tf(x, y, th, body),
            "front_edge": _tf(x, y, th, front),
            "arrow": _tf(x, y, th, arrow),
            "nose": _tf(x, y, th, nose),
            "safety": _tf(x, y, th, safety)}
