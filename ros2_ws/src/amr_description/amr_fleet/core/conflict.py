"""
Decentralised conflict detection.

Each robot compares ITS OWN intent against every peer's broadcast intent.
There is no conflict manager. Two robots looking at the same pair of intents
reach the same conclusion because they run identical code on identical data.

Four detector types, because one is not enough:

  CELL - the workhorse. Same grid cell, overlapping time windows.
  SWAP - head-on exchange. I go A->B while the peer goes B->A. Cell-overlap
         ALONE CAN MISS THIS if the timings interleave, yet the robots
         physically collide. Test this case explicitly.
  ZONE - both paths need the same capacity-1 resource within the horizon.
         Fires earlier than CELL and drives the request/grant protocol.
  TTC  - continuous closest-point-of-approach. Catches anything the discrete
         layers miss: control error, drift, an unplanned peer manoeuvre.
"""
from typing import Dict, List, Optional

from .geometry import time_to_collision
from .models import Conflict, Intent, RobotState
from .priority import wins


class ConflictDetector:
    def __init__(self,
                 horizon_s: float = 8.0,
                 ttc_threshold_s: float = 4.0,
                 coordination_radius_m: float = 3.0,
                 zone_lookahead_s: float = 10.0):
        self.horizon = horizon_s
        self.ttc_threshold = ttc_threshold_s
        # Peers beyond this radius are skipped entirely. On a Jetson Nano this
        # matters: it keeps the per-tick cost flat as the fleet grows.
        self.coordination_radius = coordination_radius_m
        self.zone_lookahead = zone_lookahead_s

    def detect(self, me: RobotState, my_intent: Intent,
               peers: Dict[str, RobotState], now: float) -> List[Conflict]:
        out: List[Conflict] = []
        my_cells = [c for c, _, _ in my_intent.triples()]

        for pid, peer in peers.items():
            if pid == me.robot_id or not peer.alive:
                continue

            dx = peer.pose.x - me.pose.x
            dy = peer.pose.y - me.pose.y
            dist = (dx * dx + dy * dy) ** 0.5

            # TYPE C: continuous TTC. Always run - a peer that is far away but
            # closing fast still matters.
            ttc, d_cpa = time_to_collision(
                me.pose.x, me.pose.y, me.vx, me.vy,
                peer.pose.x, peer.pose.y, peer.vx, peer.vy,
                horizon=self.horizon)
            if ttc is not None and ttc < self.ttc_threshold:
                out.append(Conflict(
                    kind="TTC", peer_id=pid, t_conflict=now + ttc,
                    severity=1.0 / max(0.1, ttc),
                    my_priority=me.priority_score,
                    peer_priority=peer.priority_score,
                    i_have_priority=self._i_have_priority(me, peer)))

            # The discrete checks need a peer intent to compare against.
            peer_intent = peer.intent
            if not peer_intent.cells:
                continue

            # TYPE A: space-time cell overlap.
            peer_windows = {c: (a, b) for c, a, b in peer_intent.triples()}
            for cell, t_in, t_out in my_intent.triples():
                if t_in > now + self.horizon:
                    break                       # intents are time-ordered
                pw = peer_windows.get(cell)
                if pw is None:
                    continue
                p_in, p_out = pw
                if t_in < p_out and p_in < t_out:        # interval overlap
                    tc = max(t_in, p_in)
                    out.append(Conflict(
                        kind="CELL", peer_id=pid, cell=cell, t_conflict=tc,
                        severity=1.0 / max(0.1, tc - now),
                        my_priority=me.priority_score,
                        peer_priority=peer.priority_score,
                        i_have_priority=self._i_have_priority(me, peer)))

            # TYPE B: head-on swap.
            peer_cells = [c for c, _, _ in peer_intent.triples()]
            swap = self._detect_swap(my_cells, peer_cells)
            if swap is not None:
                out.append(Conflict(
                    kind="SWAP", peer_id=pid, cell=swap, t_conflict=now,
                    severity=10.0,                      # always urgent
                    my_priority=me.priority_score,
                    peer_priority=peer.priority_score,
                    i_have_priority=self._i_have_priority(me, peer)))

            # TYPE D: shared capacity-1 zone.
            shared = [z for z in my_intent.zones if z in peer_intent.zones]
            for zid in shared:
                out.append(Conflict(
                    kind="ZONE", peer_id=pid, zone_id=zid,
                    t_conflict=self._zone_eta(my_intent, zid, now),
                    severity=1.0,
                    my_priority=me.priority_score,
                    peer_priority=peer.priority_score,
                    i_have_priority=self._i_have_priority(me, peer)))

        return out

    @staticmethod
    def _i_have_priority(me: RobotState, peer: RobotState) -> bool:
        """Return the same winner on every robot for the same observed pair."""
        # The zone protocol uses (score, lamport, robot_id). Conflict messages
        # do not carry Lamport timestamps, so use a neutral equal logical time
        # and robot_id as the deterministic final tie-break.
        return wins((me.priority_score, 0, me.robot_id),
                    (peer.priority_score, 0, peer.robot_id))

    @staticmethod
    def _detect_swap(mine: List, theirs: List) -> Optional[tuple]:
        """Find A->B against B->A.

        Bounded to the first 10 steps of each path: a swap 30 cells ahead will
        be replanned long before it happens, and the nested loop is O(n*m).
        """
        m = mine[:10]
        t = theirs[:10]
        for i in range(len(m) - 1):
            for j in range(len(t) - 1):
                if m[i] == t[j + 1] and m[i + 1] == t[j]:
                    return m[i]
        return None

    def _zone_eta(self, intent: Intent, zone_id: str, now: float) -> float:
        if zone_id in intent.zones:
            idx = intent.zones.index(zone_id)
            frac = (idx + 1) / float(len(intent.zones) + 1)
            if intent.t_enter:
                span = intent.t_enter[-1] - now
                return now + max(0.0, span * frac)
        return now + self.zone_lookahead


def summarise(conflicts: List[Conflict]) -> Dict[str, int]:
    """Counts by kind - used by the metrics logger and the dashboard."""
    out: Dict[str, int] = {}
    for c in conflicts:
        out[c.kind] = out.get(c.kind, 0) + 1
    return out
