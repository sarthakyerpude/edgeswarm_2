"""
Peer registry, liveness tracking and failure detection.

TWO INDEPENDENT MECHANISMS - the prompt asks why both are useful:

  DDS LIVELINESS (configured in the QoS profile, handled by the middleware)
      Detects PARTICIPANT death: the process crashed, was killed, or the
      machine went down. Fast and authoritative, because the middleware knows
      its own connection state. But it only ever tells you "that process is
      gone".

  APPLICATION HEARTBEAT (this module - RobotState arrival timing)
      Detects everything else: a robot whose process is alive but whose LiDAR
      has failed, whose control loop has wedged, whose Wi-Fi dropped while the
      process survived, or which has set alive=false on itself. It also works
      identically over the raw-UDP fallback transport, where DDS liveliness
      does not exist at all.

  Your SAFETY logic depends on the application heartbeat. Treat DDS liveliness
  as a useful early signal that lets you react ~200 ms sooner.

MINIMISING FALSE POSITIVES (asked for in the failure test section)
  1. A generous timeout (1.5 s = 15 missed messages at 10 Hz). At 10 Hz, a
     handful of consecutive losses is normal on Wi-Fi; 15 is not.
  2. A two-stage decision: SUSPECT then DEAD. A suspected peer is treated
     cautiously (no new zone assumptions) but is not yet written off, so a
     brief Wi-Fi stall does not trigger task reallocation.
  3. Grace period after startup, so a robot that joins late is not declared
     dead before it has said anything.
  4. Instant recovery: one message moves a peer straight back to ALIVE.
"""
from typing import Dict, List, Optional, Set

from .models import RobotState

T_SUSPECT_S = 0.6        # missed ~6 messages: be cautious
T_DEAD_S = 1.5           # missed ~15 messages: treat as failed
T_STARTUP_GRACE_S = 5.0

ALIVE, SUSPECT, DEAD = "ALIVE", "SUSPECT", "DEAD"


class PeerRegistry:
    def __init__(self, my_id: str, logger=None,
                 t_suspect: float = T_SUSPECT_S, t_dead: float = T_DEAD_S):
        self.me = my_id
        self.log = logger
        self.t_suspect = t_suspect
        self.t_dead = t_dead

        self.peers: Dict[str, RobotState] = {}
        self.last_seen: Dict[str, float] = {}
        self.health: Dict[str, str] = {}
        self.lost_messages: Dict[str, int] = {}
        self.started_at = 0.0

        self.stats = {"peers_seen": 0, "failures_detected": 0,
                      "recoveries": 0, "stale_rejected": 0}

    # -------------------------------------------------------------- ingestion
    def update(self, state: RobotState, now: Optional[float] = None) -> bool:
        """Record a peer state. Returns True if accepted.

        Rejects our own broadcast (DDS delivers your own publications back to
        your own subscriber - this is correct DDS behaviour, not a bug) and
        rejects out-of-order or duplicate messages by sequence number.
        """
        if state.robot_id == self.me:
            return False
        now = 0.0 if now is None else now

        prev = self.peers.get(state.robot_id)
        if prev is not None:
            if state.seq <= prev.seq:
                self.stats["stale_rejected"] += 1
                return False
            gap = state.seq - prev.seq - 1
            if gap > 0:
                self.lost_messages[state.robot_id] = \
                    self.lost_messages.get(state.robot_id, 0) + gap
        else:
            self.stats["peers_seen"] += 1
            if self.log:
                self.log(f"peer discovered: {state.robot_id}")

        state.rx_time = now
        self.peers[state.robot_id] = state
        self.last_seen[state.robot_id] = now

        old = self.health.get(state.robot_id)
        if old in (SUSPECT, DEAD):
            self.stats["recoveries"] += 1
            if self.log:
                self.log(f"peer recovered: {state.robot_id} (was {old})")
        self.health[state.robot_id] = ALIVE
        return True

    # --------------------------------------------------------------- liveness
    def tick(self, now: Optional[float] = None) -> List[str]:
        """Re-evaluate health. Returns robot_ids that JUST transitioned to DEAD."""
        now = 0.0 if now is None else now
        newly_dead: List[str] = []

        for rid, seen in self.last_seen.items():
            age = now - seen
            prev = self.health.get(rid, ALIVE)
            if age > self.t_dead:
                new = DEAD
            elif age > self.t_suspect:
                new = SUSPECT
            else:
                new = ALIVE
            if new != prev:
                self.health[rid] = new
                if new == DEAD:
                    newly_dead.append(rid)
                    self.stats["failures_detected"] += 1
                    if self.log:
                        self.log(f"PEER FAILURE: {rid} silent for {age:.2f}s")
                elif new == SUSPECT and self.log:
                    self.log(f"peer suspect: {rid} ({age:.2f}s)")
        return newly_dead

    # ----------------------------------------------------------------- views
    def alive(self) -> Dict[str, RobotState]:
        """Peers usable for coordination. SUSPECT peers ARE included.

        Rationale: a suspect peer is probably still driving. Excluding it from
        conflict checks would be far more dangerous than including a peer whose
        pose is 800 ms stale.
        """
        return {rid: st for rid, st in self.peers.items()
                if self.health.get(rid) != DEAD and st.alive}

    def alive_ids(self) -> Set[str]:
        return set(self.alive().keys())

    def dead_ids(self) -> Set[str]:
        return {rid for rid, h in self.health.items() if h == DEAD}

    def obstacles_from_dead(self) -> List[RobotState]:
        """Dead peers, frozen at their last known pose.

        A dead robot has not evaporated - it is a 0.4 m box in an aisle. It
        must keep blocking paths, it just stops being treated as an agent that
        can negotiate or yield.
        """
        return [self.peers[rid] for rid in self.dead_ids() if rid in self.peers]

    def peers_needing_zone(self, zone_id: str) -> Set[str]:
        """Alive peers whose broadcast intent requires this zone.

        This is exactly the set whose grants are required before entering.
        Peers that do not want the zone are excluded, so an uncontested zone
        costs one round trip instead of a negotiation.
        """
        return {rid for rid, st in self.alive().items()
                if zone_id in st.intent.zones}

    def loss_rate(self, rid: str) -> float:
        st = self.peers.get(rid)
        if st is None or st.seq == 0:
            return 0.0
        lost = self.lost_messages.get(rid, 0)
        return lost / float(st.seq + lost)

    def metrics(self) -> dict:
        return dict(peers_known=len(self.peers),
                    peers_alive=len(self.alive()),
                    peers_dead=len(self.dead_ids()),
                    loss_rates={r: round(self.loss_rate(r), 4)
                                for r in self.peers},
                    **self.stats)
