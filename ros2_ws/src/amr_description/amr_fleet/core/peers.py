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

PEER INTENTS live in their own dict, not only on the stored RobotState.
A state message (10 Hz) carries no path, so it would otherwise wipe the
1 Hz intent; the registry re-attaches the held intent on every state update,
buffers intents that arrive before the peer's first state, and drops them
when the peer's boot_id changes.
"""
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Set

from .geometry import extrapolate
from .models import Intent, Pose2D, RobotState

T_SUSPECT_S = 0.6        # missed ~6 messages: be cautious
T_DEAD_S = 1.5           # missed ~15 messages: treat as failed
T_STARTUP_GRACE_S = 5.0
# Increment 3/4 liveness TIERS, judged on the age of ANY message (state,
# intent OR zone traffic - see touch()), not only the 10 Hz state heartbeat.
# Webots under load stalls heartbeats 0.2-4.4 s (25 false DEADs in 8 min) and
# sih-57 measured 10-16 s drops, so:
#   FRESH   < T_SUSPECT_S        normal
#   SUSPECT < T_SILENT_S  (5 s)  stale pose, still negotiating
#   SILENT  < T_GONE_S   (30 s)  frozen ghost: KEEPS every zone hold/grant,
#                                stays a required granter (zones survive
#                                silence - the sih-57 double-hold fix)
#   GONE    >= T_GONE_S          claims reclaimable except zones containing
#                                its frozen hitbox (coordinator rule)
T_SILENT_S = 5.0
T_GONE_S = 30.0
# Data-age levels from the receiver's rx_time (never the sender's stamp).
T_FRESH_S = 0.3
# ADAPTIVE freshness (degraded-comms fix, 1.0 m/s live run): under host
# load the state link degrades to 3-4 Hz with ~37 % loss - and the lag can
# be RECEIVER-side (executor backlog), so a laggy robot saw 9-10 Hz senders
# as SUSPECT and froze itself against fresh peers. The FRESH horizon is
# therefore per-peer: FRESH_INTERVALS x the receiver-measured inter-arrival
# EMA (registry rx deltas, never sender stamps), floored at T_FRESH_S and
# ceilinged at T_FRESH_CEIL_S. SILENT/GONE exclusion semantics and t_dead
# are untouched.
T_FRESH_CEIL_S = 1.2
FRESH_INTERVALS = 3.0
FRESH_EMA_ALPHA = 0.2
# Pose extrapolation: (now - rx_time) plus this lead, capped, because
# constant-twist prediction is meaningless after ~2 s.
EXTRAPOLATION_LEAD_S = 0.1
MAX_EXTRAPOLATION_S = 2.0
# A peer whose broadcast intent_seq is ahead of the intent held here has an
# unknown plan: it is assumed to need every zone within this radius.
UNKNOWN_PLAN_ZONE_RADIUS_M = 3.0

ALIVE, SUSPECT, DEAD = "ALIVE", "SUSPECT", "DEAD"
FRESH = "FRESH"
SILENT, GONE = "SILENT", "GONE"


@dataclass
class HeldIntent:
    intent: Intent
    seq: int
    stamp: Optional[float]          # sender clock; orders same-seq resends
    boot_id: Optional[int]          # None while buffered before first state
    rx_time: float


def _seq_ahead(a: int, b: int) -> bool:
    """True if uint32 serial number a is strictly ahead of b."""
    delta = (int(a) - int(b)) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


class PeerRegistry:
    def __init__(self, my_id: str, logger=None,
                 t_suspect: float = T_SUSPECT_S, t_dead: float = T_DEAD_S):
        self.me = my_id
        self.log = logger
        self.t_suspect = t_suspect
        self.t_dead = t_dead

        self.peers: Dict[str, RobotState] = {}
        self.last_seen: Dict[str, float] = {}
        # Receiver-measured state inter-arrival EMA per peer (rx deltas).
        self._rx_interval: Dict[str, float] = {}
        # Receiver-clock arrival of ANY message from the peer (state, intent,
        # zone request/grant) - the tier clock. last_seen stays state-only
        # because it also dates the POSE for extrapolation.
        self.last_touch: Dict[str, float] = {}
        self.health: Dict[str, str] = {}
        self.lost_messages: Dict[str, int] = {}
        self.intents: Dict[str, HeldIntent] = {}
        self.started_at = 0.0
        # (x, y, radius_m) -> zone ids within radius. Set by the coordinator
        # from the grid; None means "any zone" (conservative).
        self.zones_near: Optional[Callable[[float, float, float],
                                           Set[str]]] = None

        self.stats = {"peers_seen": 0, "failures_detected": 0,
                      "recoveries": 0, "stale_rejected": 0,
                      "process_restarts": 0, "intents_buffered": 0,
                      "intents_stale_rejected": 0, "intents_dropped_reboot": 0}

    # -------------------------------------------------------------- ingestion
    def update(self, state: RobotState, now: Optional[float] = None) -> bool:
        """Record a peer state. Returns True if accepted.

        Rejects our own broadcast (DDS delivers your own publications back to
        your own subscriber - this is correct DDS behaviour, not a bug),
        recognizes a new process epoch, and rejects duplicate/out-of-order
        messages with uint32 serial-number arithmetic within one epoch.
        """
        if state.robot_id == self.me:
            return False
        now = 0.0 if now is None else now

        prev = self.peers.get(state.robot_id)
        if prev is not None:
            new_process = state.boot_id != prev.boot_id
            if new_process:
                self.stats["process_restarts"] += 1
                self.lost_messages[state.robot_id] = 0
                if self.log:
                    self.log(f"peer process restarted: {state.robot_id}")
            else:
                # Serial-number arithmetic accepts uint32 wraparound while
                # rejecting duplicates and packets older by at least half the
                # sequence space.
                delta = (int(state.seq) - int(prev.seq)) & 0xFFFFFFFF
                if delta == 0 or delta >= 0x80000000:
                    self.stats["stale_rejected"] += 1
                    return False
                gap = delta - 1
                if gap > 0:
                    self.lost_messages[state.robot_id] = \
                        self.lost_messages.get(state.robot_id, 0) + gap
        else:
            self.stats["peers_seen"] += 1
            if self.log:
                self.log(f"peer discovered: {state.robot_id}")

        state.rx_time = now
        self._attach_intent(state, now)
        # Receiver-side inter-arrival EMA (adaptive FRESH horizon). Measured
        # from OUR rx clock deltas, so a laggy receiver relaxes its own
        # thresholds symmetrically instead of ghosting healthy senders.
        prev_rx = self.last_seen.get(state.robot_id)
        if prev_rx is not None and now > prev_rx:
            dt = now - prev_rx
            ema = self._rx_interval.get(state.robot_id)
            self._rx_interval[state.robot_id] = (
                dt if ema is None
                else (1.0 - FRESH_EMA_ALPHA) * ema + FRESH_EMA_ALPHA * dt)
        self.peers[state.robot_id] = state
        self.last_seen[state.robot_id] = now
        self.touch(state.robot_id, now)

        old = self.health.get(state.robot_id)
        if old in (SUSPECT, DEAD):
            self.stats["recoveries"] += 1
            if self.log:
                self.log(f"peer recovered: {state.robot_id} (was {old})")
        self.health[state.robot_id] = ALIVE
        return True

    def _attach_intent(self, state: RobotState, now: float) -> None:
        """Give an accepted state the held intent of the same process epoch.

        A state that already carries a path (core callers and tests build
        RobotState with an embedded intent) is stored as that peer's intent;
        a ROS state carries none and inherits the held one.
        """
        rid = state.robot_id
        held = self.intents.get(rid)
        if held is not None and held.boot_id is None:
            held.boot_id = state.boot_id          # buffered before first state
        elif held is not None and held.boot_id != state.boot_id:
            del self.intents[rid]
            held = None
            self.stats["intents_dropped_reboot"] += 1
        if state.intent.cells or state.intent.zones:
            held = HeldIntent(state.intent, int(state.intent_seq), None,
                              state.boot_id, now)
            self.intents[rid] = held
        if held is not None:
            state.intent = held.intent

    def update_intent(self, robot_id: str, intent: Intent, seq: int,
                      now: Optional[float] = None,
                      stamp: Optional[float] = None) -> bool:
        """Record a peer intent (times already on my clock). True if accepted.

        Kept independently of the state so the 10 Hz state stream cannot
        erase it. An intent from a peer with no state yet is buffered and
        attached when its first state arrives. Within one process epoch,
        older seqs are rejected; a same-seq resend (the keepalive) replaces
        the held copy only if its sender stamp is newer.
        """
        if robot_id == self.me:
            return False
        now = 0.0 if now is None else now
        seq = int(seq) & 0xFFFFFFFF
        st = self.peers.get(robot_id)
        boot = st.boot_id if st is not None else None
        held = self.intents.get(robot_id)
        if held is not None and held.boot_id == boot:
            if _seq_ahead(held.seq, seq):
                self.stats["intents_stale_rejected"] += 1
                return False
            if (held.seq == seq and stamp is not None
                    and held.stamp is not None and stamp <= held.stamp):
                self.stats["intents_stale_rejected"] += 1
                return False
        self.intents[robot_id] = HeldIntent(intent, seq, stamp, boot, now)
        self.touch(robot_id, now)
        if st is not None:
            st.intent = intent
        else:
            self.stats["intents_buffered"] += 1
        return True

    # ------------------------------------------------------------- tiers
    def touch(self, rid: str, now: float) -> None:
        """Record liveness evidence from ANY message (state, intent, zone
        request/grant...). Called by update()/update_intent() internally and
        by the node/coordinator for zone traffic, so a robot whose heartbeat
        stalls under CPU load but whose zone protocol flows is never tiered
        SILENT and never loses exclusion it still exercises."""
        if rid == self.me:
            return
        prev = self.last_touch.get(rid)
        if prev is None or now > prev:
            self.last_touch[rid] = now

    def tier(self, rid: str, now: float) -> str:
        """FRESH / SUSPECT / SILENT / GONE from the age of the newest message
        of ANY kind. A peer never heard from at all is GONE. The
        FRESH->SUSPECT boundary is adaptive (fresh_horizon) so a slow-but-
        healthy link, or a laggy RECEIVER, does not ghost live peers;
        SILENT/GONE exclusion semantics are fixed constants."""
        seen = self.last_touch.get(rid, self.last_seen.get(rid))
        if seen is None:
            return GONE
        age = now - seen
        if age < max(self.t_suspect, self.fresh_horizon(rid)):
            return FRESH
        if age < T_SILENT_S:
            return SUSPECT
        if age < T_GONE_S:
            return SILENT
        return GONE

    def silent(self, now: float) -> List[str]:
        """Peers in the SILENT tier (quiet 5-30 s). They keep every zone hold
        and grant, stay required granters, and become frozen envelope ghosts
        (safety.silent_ghost_discs)."""
        return [rid for rid in self.peers
                if self.tier(rid, now) == SILENT]

    def gone_ids(self, now: float) -> List[str]:
        """Peers quiet for T_GONE_S or longer: the only silence that may
        reclaim their claims (except zones containing their frozen hitbox)."""
        return [rid for rid in self.peers
                if self.tier(rid, now) == GONE]

    def granters(self, now: float) -> List[str]:
        """Every peer whose grant a ranked-zone entry must collect: alive
        (not self-faulted) and not GONE. SILENT peers ARE included - that is
        the sih-57 fix: a stale-but-present peer keeps its exclusion."""
        return [rid for rid, st in self.peers.items()
                if st.alive and self.tier(rid, now) != GONE]

    def held_intent_seq(self, rid: str) -> Optional[int]:
        held = self.intents.get(rid)
        return None if held is None else held.seq

    def plan_unknown(self, rid: str) -> bool:
        """The peer's broadcast intent_seq is ahead of the intent held here
        (lost, delayed or not yet received), so its plan is unknown."""
        st = self.peers.get(rid)
        if st is None or not st.intent_seq:
            return False
        held = self.intents.get(rid)
        return held is None or _seq_ahead(st.intent_seq, held.seq)

    # -------------------------------------------------------------- freshness
    def fresh_horizon(self, rid: str) -> float:
        """Per-peer FRESH age limit: FRESH_INTERVALS x the receiver-measured
        inter-arrival EMA, bounded to [T_FRESH_S, T_FRESH_CEIL_S]. With no
        measurement yet, the classic T_FRESH_S."""
        ema = self._rx_interval.get(rid)
        if ema is None:
            return T_FRESH_S
        return min(T_FRESH_CEIL_S, max(T_FRESH_S, FRESH_INTERVALS * ema))

    def freshness(self, rid: str, now: float) -> str:
        """FRESH (< fresh_horizon), SUSPECT, or DEAD (existing health
        logic), judged from the receiver's rx_time."""
        st = self.peers.get(rid)
        if st is None:
            return DEAD
        age = now - self.last_seen.get(rid, st.rx_time)
        if self.health.get(rid) == DEAD or age > self.t_dead:
            return DEAD
        return FRESH if age < self.fresh_horizon(rid) else SUSPECT

    def extrapolated_pose(self, rid: str, now: float) -> Optional[Pose2D]:
        """Peer pose predicted to `now` with its broadcast v and w.

        Integrates over (now - rx_time + EXTRAPOLATION_LEAD_S), capped at
        MAX_EXTRAPOLATION_S. A DEAD or faulted peer stays frozen at its last
        pose (it is an obstacle, not a mover).
        """
        st = self.peers.get(rid)
        if st is None:
            return None
        p = st.pose
        if not st.alive or self.freshness(rid, now) == DEAD:
            return Pose2D(p.x, p.y, p.theta)
        dt = min(MAX_EXTRAPOLATION_S,
                 max(0.0, now - st.rx_time) + EXTRAPOLATION_LEAD_S)
        x, y, th = extrapolate(p.x, p.y, p.theta, st.v, st.w, dt)
        return Pose2D(x, y, th)

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
        """Unavailable peers, frozen at their last known pose.

        A process-dead or explicitly faulted robot has not evaporated - it is
        still a physical obstacle in the aisle. It must keep blocking paths,
        but it is no longer treated as an agent that can negotiate or yield.
        """
        return [state for rid, state in self.peers.items()
                if (not state.alive or self.health.get(rid) == DEAD)]

    def peers_needing_zone(self, zone_id: str) -> Set[str]:
        """Alive peers whose broadcast intent requires this zone.

        This is exactly the set whose grants are required before entering.
        Peers that do not want the zone are excluded, so an uncontested zone
        costs one round trip instead of a negotiation. A peer whose plan is
        unknown (see plan_unknown) is assumed to need every zone within
        UNKNOWN_PLAN_ZONE_RADIUS_M of its pose.
        """
        out = set()
        for rid, st in self.alive().items():
            if zone_id in st.intent.zones:
                out.add(rid)
            elif self.plan_unknown(rid):
                if self.zones_near is None or zone_id in self.zones_near(
                        st.pose.x, st.pose.y, UNKNOWN_PLAN_ZONE_RADIUS_M):
                    out.add(rid)
        return out

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
