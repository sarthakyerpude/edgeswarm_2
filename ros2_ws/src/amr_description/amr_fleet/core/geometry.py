"""
Continuous-space collision prediction: closest point of approach and
time-to-collision for two robots modelled as discs.

This is the SHORT-HORIZON safety check (0-4 s). It complements, and does not
replace, the space-time reservation check in conflict.py, which handles the
long horizon where constant-velocity extrapolation is meaningless.
"""
import math
from typing import Optional, Tuple

# Tuned for a 0.16 m footprint radius. Keep these consistent with the URDF
# collision geometry in amr_description/urdf/.
ROBOT_RADIUS_M = 0.16
SAFETY_MARGIN_M = 0.15
DEFAULT_HORIZON_S = 8.0


def time_to_collision(ax, ay, avx, avy, bx, by, bvx, bvy,
                      r_sum: float = 2 * ROBOT_RADIUS_M,
                      margin: float = SAFETY_MARGIN_M,
                      horizon: float = DEFAULT_HORIZON_S
                      ) -> Tuple[Optional[float], float]:
    """Return (t_collision | None, distance_at_closest_approach).

    Derivation
    ----------
      relative position  r = p_b - p_a
      relative velocity  v = v_b - v_a

      Minimise |r + v t|^2 over t:
          d/dt |r + v t|^2 = 2(r.v) + 2t|v|^2 = 0
          t_cpa = -(r.v) / |v|^2

      t_cpa < 0 means the robots are already separating - no future collision.

      d_cpa = |r + v t_cpa|
      A collision is predicted when d_cpa < D, where D = r_sum + margin.

      First contact time solves |r + v t| = D:
          |v|^2 t^2 + 2(r.v) t + (|r|^2 - D^2) = 0
          disc = (r.v)^2 - |v|^2 (|r|^2 - D^2)
          t    = ( -(r.v) - sqrt(disc) ) / |v|^2      (smaller root)
    """
    rx, ry = bx - ax, by - ay
    vx, vy = bvx - avx, bvy - avy
    v2 = vx * vx + vy * vy
    dist_now = math.hypot(rx, ry)
    D = r_sum + margin

    # Relatively stationary: no CPA is defined, only current separation.
    if v2 < 1e-9:
        return (0.0 if dist_now < D else None), dist_now

    rv = rx * vx + ry * vy
    t_cpa = -rv / v2
    if t_cpa < 0.0:
        return None, dist_now                      # separating
    t_cpa = min(t_cpa, horizon)

    d_cpa = math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)
    if d_cpa >= D:
        return None, d_cpa

    disc = rv * rv - v2 * (rx * rx + ry * ry - D * D)
    if disc < 0.0:
        return None, d_cpa
    t_hit = (-rv - math.sqrt(disc)) / v2
    if t_hit > horizon:
        return None, d_cpa
    return max(0.0, t_hit), d_cpa


def safety_distance(v_rel: float, t_react: float = 0.15,
                    a_brake: float = 1.0, loc_err: float = 0.10,
                    comm_latency: float = 0.10) -> float:
    """Minimum separation that still allows a full stop.

        d_safe = R1+R2 + v_rel*t_react + v_rel^2/(2*a_brake)
                 + localisation error + v_rel*comm_latency

    At v_rel = 1.0 m/s this gives roughly 1.17 m.
    """
    return (2 * ROBOT_RADIUS_M
            + v_rel * t_react
            + (v_rel * v_rel) / (2.0 * a_brake)
            + loc_err
            + v_rel * comm_latency)


def extrapolate(x, y, theta, v, w, dt) -> Tuple[float, float, float]:
    """Constant-twist motion model. Exact arc integration, not Euler.

    Euler integration visibly diverges from the true arc within ~0.5 s at
    realistic turn rates, which produces false conflicts on every curve.
    """
    if abs(w) < 1e-6:
        return x + v * dt * math.cos(theta), y + v * dt * math.sin(theta), theta
    r = v / w
    nt = theta + w * dt
    return (x + r * (math.sin(nt) - math.sin(theta)),
            y - r * (math.cos(nt) - math.cos(theta)),
            nt)
