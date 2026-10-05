"""
Distributed mutual exclusion over named capacity-1 zones.

Ricart-Agrawala. ORDERING KEY (increment 3+): (lamport, robot_id) - PURE
FIFO. priority_score is still carried on the wire (diagnostics, legacy
consumers) but NEVER orders or preempts a zone: the sih-57 field runs showed
a stale-score reorder race producing a double hold, and FIFO plus unlimited
same-stamp resends bounds every waiter by queue transit instead.

SAFETY INVARIANT
    A robot enters zone Z only after receiving granted=true from EVERY peer
    that is alive AND needs Z within the horizon.

    It never enters on the basis of "I computed that I have priority".
    That distinction is the difference between a system that works and one
    that collides when a single packet is lost.

TIMEOUTS EXIST BECAUSE THE NETWORK IS NOT PERFECT
    A lost grant must never wedge the fleet.
      - UNRANKED (legacy) zones keep the old escapes: resend up to
        MAX_RETRIES, then ZoneTimeout after T_ZONE_S -> reroute.
      - RANKED traffic zones NEVER time out: the SAME request (same req_id,
        same lamport) is resent every T_GRANT_RESEND_S, unlimited. A fresh
        lamport is stamped only on a genuinely NEW request, so a waiter
        keeps its FIFO queue position across resends (the measured 2x60 s
        acquisition starvation came from timeout + re-request losing it).

LAMPORT-VALIDATED GRANTS (the measured flush/reorder double hold)
    peer_req_lc[(zone, peer)] records the newest request lamport seen from
    each peer. A grant whose lamport is below that is stale (sent before the
    peer's newest request) and is discarded; a peer request newer than a
    held grant voids that grant. Either delivery order of {flushed grant,
    new request} therefore leaves at most one holder.

drop_peer() is called by the coordinator ONLY on boot_id change, FAULT or
GONE (>= 30 s silent) - never on a brief stall: a silent peer keeps every
hold and grant and remains a required granter (strict partition).
"""
import secrets
from typing import Callable, Dict, List, Optional, Set

FREE, REQUESTING, HELD = "FREE", "REQUESTING", "HELD"

T_GRANT_S = 1.0          # resend a request if a live peer has not replied
T_GRANT_RESEND_S = 1.0   # ranked zones: same-stamp resend period, unlimited
MAX_RETRIES = 3
T_ZONE_S = 10.0          # UNRANKED zones only: give up entirely and reroute
T_RECLAIM_S = 2.0        # superseded by the registry's GONE tier (30 s);
                         # kept for legacy readers


class ZoneTimeout(Exception):
    """Raised when acquisition exceeds T_ZONE_S. Caller should reroute."""

    def __init__(self, zone_id: str):
        super().__init__(f"zone acquisition timed out: {zone_id}")
        self.zone_id = zone_id


class ZoneArbiter:
    def __init__(self, robot_id: str,
                 send_request: Callable[[dict], None],
                 send_grant: Callable[[dict], None],
                 logger=None):
        self.me = robot_id
        self._send_request = send_request
        self._send_grant = send_grant
        self.log = logger

        self.lamport: int = 0
        self.state: Dict[str, str] = {}            # zone -> FREE/REQUESTING/HELD
        self.grants: Dict[str, Set[str]] = {}      # zone -> granters
        self.deferred: Dict[str, List[tuple]] = {}   # zone -> peers awaiting my grant
        self.req_time: Dict[str, float] = {}
        self.req_id: Dict[str, int] = {}
        self.retries: Dict[str, int] = {}
        self.my_key: Dict[str, tuple] = {}         # zone -> (score, lamport, id)
        self.consecutive_wins: Dict[str, int] = {}
        # Ranked traffic zones: no ZoneTimeout, same-stamp resend forever.
        self.ranked: Dict[str, bool] = {}          # zone -> requested as ranked
        self._last_resend: Dict[str, float] = {}
        # Lamport validation of grants (see module docstring).
        self.peer_req_lc: Dict[tuple, int] = {}    # (zone, peer) -> newest req lc
        self.grant_lc: Dict[tuple, int] = {}       # (zone, peer) -> grant lc held
        # Zone grants are retained for late joiners. A process-local counter
        # starting at 1 would collide with a pre-restart grant for this robot
        # and could falsely satisfy a new mutual-exclusion request. Randomize
        # the process epoch within the uint32 wire field, then increment it.
        self._next_req_id = secrets.randbelow(0xFFFFFFFF) + 1

        # Diagnostics for your report.
        self.stats = {"requests": 0, "grants_sent": 0, "grants_recv": 0,
                      "deferrals": 0, "timeouts": 0, "retries": 0}

    # ------------------------------------------------------------- clock
    def tick_clock(self, observed: int = 0) -> int:
        """Lamport rule: lc = max(lc, observed) + 1 on every coordination event."""
        self.lamport = max(self.lamport, observed) + 1
        return self.lamport

    # ----------------------------------------------------------- requesting
    def request(self, zone_id: str, my_score: float,
                t_enter: float, t_exit: float, now: float = 0.0,
                ranked: bool = False) -> None:
        """A genuinely NEW request: fresh lamport, fresh req_id, never reused.
        Resends (lost-message recovery) go through _resend with the SAME
        stamp, so the FIFO queue position is kept."""
        if self.state.get(zone_id) in (REQUESTING, HELD):
            return                                  # already in progress
        lc = self.tick_clock()
        rid = self._next_req_id
        self._next_req_id = (rid + 1) & 0xFFFFFFFF
        if self._next_req_id == 0:
            self._next_req_id = 1

        self.state[zone_id] = REQUESTING
        self.grants[zone_id] = set()
        self.deferred.setdefault(zone_id, [])
        self.req_time[zone_id] = now
        self.req_id[zone_id] = rid
        self.retries[zone_id] = 0
        self.ranked[zone_id] = bool(ranked)
        self._last_resend[zone_id] = now
        self.my_key[zone_id] = (my_score, lc, self.me)
        # grant_lc entries belong to my PREVIOUS request for this zone.
        for key in [k for k in self.grant_lc if k[0] == zone_id]:
            del self.grant_lc[key]
        self.stats["requests"] += 1

        self._send_request(dict(robot_id=self.me, zone_id=zone_id,
                                lamport_ts=lc, priority_score=my_score,
                                t_enter_est=t_enter, t_exit_est=t_exit,
                                req_id=rid))
        if self.log:
            self.log(f"ZONE REQ {zone_id} score={my_score:.6f} lc={lc} id={rid}")

    # ----------------------------------------------------- handling a request
    def on_request(self, msg: dict, i_need_zone: bool,
                   relevant_peers: Optional[Set[str]] = None) -> None:
        """Decide how to answer a peer's ZoneRequest.

        i_need_zone: does MY current plan require this zone within the horizon?
        If not, grant immediately - an uncontested zone must not cost a
        negotiation round.

        relevant_peers: who must grant ME this zone. Used to detect that I am
        already the effective owner (see the ownership check below).
        """
        zid = msg["zone_id"]
        peer = msg["robot_id"]
        if peer == self.me:
            return                                  # my own broadcast

        lc_msg = int(msg.get("lamport_ts", 0))
        self.tick_clock(lc_msg)
        # Newest-request bookkeeping: any grant from `peer` older than this
        # lamport is stale from now on (see on_grant).
        if lc_msg > self.peer_req_lc.get((zid, peer), -1):
            self.peer_req_lc[(zid, peer)] = lc_msg
        st = self.state.get(zid, FREE)

        if st == HELD:
            self._defer(zid, peer, int(msg.get("req_id", 0)))
            return

        # -------------------------------------------------------------------
        # OWNERSHIP CHECK - fixes a real mutual-exclusion bug found by
        # test_zone_protocol.py::test_mutual_exclusion_never_violated.
        #
        # THE BUG: peer A requests first. I am not competing yet, so I grant.
        # A moment later I want the zone too, and I happen to outrank A. A
        # naive priority comparison would make A grant to me as well - and now
        # BOTH of us hold grants from the other and both enter. Collision.
        #
        # THE FIX: once I have collected grants from every relevant peer I am
        # the effective owner, even before may_enter() flips my state to HELD.
        # An owner always defers. Priority arbitrates CONCURRENT contention;
        # it does NOT preempt a lock that has already been granted away. A
        # preemptive scheme would also allow livelock, where a high-priority
        # robot repeatedly snatches a zone from one that already acquired it.
        # -------------------------------------------------------------------
        if st == REQUESTING and relevant_peers is not None:
            have = self.grants.get(zid, set())
            if relevant_peers.issubset(have):
                self._defer(zid, peer, int(msg.get("req_id", 0)))
                return

        # -------------------------------------------------------------------
        # OVERTAKEN-GRANT VOID (lamport-validated grants). A request from
        # `peer` NEWER than the grant I hold from it means that grant was
        # flushed/reordered on its side: it is void. The grant-order check
        # below then no longer applies to this peer, and the FIFO comparison
        # decides. Because the peer's clock at grant time was already past my
        # request's lamport, its new request always loses the FIFO compare,
        # so I defer it and wait for its (re-)grant - at most one holder,
        # whatever the delivery order (the measured 0.15 s jitter race).
        # -------------------------------------------------------------------
        if (st == REQUESTING and peer in self.grants.get(zid, set())
                and lc_msg > self.grant_lc.get((zid, peer), 1 << 62)):
            self.grants[zid].discard(peer)
            self.grant_lc.pop((zid, peer), None)
            self.stats["grants_voided"] = self.stats.get("grants_voided", 0) + 1
            if self.log:
                self.log(f"ZONE {zid}: grant from {peer} overtaken by its "
                         f"newer request (lc {lc_msg}) - voided")

        # -------------------------------------------------------------------
        # GRANT-ORDER CHECK - a peer that has already granted my CURRENT
        # request (and has NOT overtaken that grant: this is a resend of the
        # request from before its grant) is ordered after me. Defer it and
        # grant on release.
        # -------------------------------------------------------------------
        if st == REQUESTING and peer in self.grants.get(zid, set()):
            self._defer(zid, peer, int(msg.get("req_id", 0)))
            return

        if st == REQUESTING and i_need_zone:
            # PURE FIFO: (lamport, robot_id). priority_score is carried on
            # the wire but never orders a zone (stale-score races).
            mine = self.my_key.get(zid, (0.0, 0, self.me))
            if (lc_msg, peer) < (mine[1], self.me):
                self._grant(peer, zid, msg["req_id"])   # they asked first
            else:
                self._defer(zid, peer, int(msg.get("req_id", 0)))
            return

        self._grant(peer, zid, msg["req_id"])

    def _defer(self, zone_id: str, peer: str, req_id: int) -> None:
        """Queue a deferred grant, keeping only the peer's NEWEST request.

        A peer that timed out and re-requested has a new req_id; a grant sent
        against the stale id would be discarded by its on_grant req_id check.
        """
        queue = self.deferred.setdefault(zone_id, [])
        queue[:] = [(p, r) for p, r in queue if p != peer]
        queue.append((peer, req_id))
        self.stats["deferrals"] += 1

    def _grant(self, requester: str, zone_id: str, req_id: int) -> None:
        lc = self.tick_clock()
        self.stats["grants_sent"] += 1
        self._send_grant(dict(granter_id=self.me, requester_id=requester,
                              zone_id=zone_id, req_id=req_id,
                              granted=True, lamport_ts=lc))

    # ------------------------------------------------------ receiving a grant
    def on_grant(self, msg: dict) -> None:
        if msg["requester_id"] != self.me:
            return
        zid = msg["zone_id"]
        # Ignore a late reply to a request we already abandoned.
        if msg.get("req_id") != self.req_id.get(zid):
            return
        lc = int(msg.get("lamport_ts", 0))
        self.tick_clock(lc)
        granter = msg["granter_id"]
        # Lamport validation: a grant older than the granter's newest request
        # for this zone was flushed before it re-requested - stale, discard.
        if lc < self.peer_req_lc.get((zid, granter), 0):
            self.stats["grants_stale"] = self.stats.get("grants_stale", 0) + 1
            if self.log:
                self.log(f"ZONE {zid}: stale grant from {granter} "
                         f"(lc {lc} < its newest request) - discarded")
            return
        if msg.get("granted", False):
            self.grants.setdefault(zid, set()).add(granter)
            self.grant_lc[(zid, granter)] = lc
            self.stats["grants_recv"] += 1

    # -------------------------------------------------------------- entering
    def may_enter(self, zone_id: str, relevant_peers: Set[str], now: float = 0.0) -> bool:
        """relevant_peers: alive peers whose plans also need this zone.

        Raises ZoneTimeout if acquisition has taken too long; the caller should
        catch it, penalise the zone, and replan.
        """
        st = self.state.get(zone_id, FREE)
        if st == HELD:
            return True
        if st != REQUESTING:
            return False

        got = self.grants.get(zone_id, set())
        if relevant_peers.issubset(got):
            self.state[zone_id] = HELD
            self.consecutive_wins[zone_id] = self.consecutive_wins.get(zone_id, 0) + 1
            if self.log:
                self.log(f"ZONE HELD {zone_id} (grants from {sorted(got)})")
            return True

        if self.ranked.get(zone_id):
            # Ranked traffic zones NEVER time out and never re-stamp: the
            # SAME (req_id, lamport) goes out every T_GRANT_RESEND_S so a
            # lost message cannot cost the FIFO queue position. The wait is
            # bounded by queue transit, and the coordinator's acquisition
            # timeout (20 s) re-enables the deadlock backstop above that.
            last = self._last_resend.get(zone_id, now)
            if now - last >= T_GRANT_RESEND_S:
                self._last_resend[zone_id] = now
                self.stats["resends"] = self.stats.get("resends", 0) + 1
                self._resend(zone_id)
            return False

        elapsed = now - self.req_time.get(zone_id, now)
        if elapsed > T_ZONE_S:
            self.stats["timeouts"] += 1
            self.state[zone_id] = FREE
            self.grants.pop(zone_id, None)
            # Flush the deferred queue BEFORE abandoning: release() early-
            # returns once state is FREE, so without this every peer we
            # deferred is black-holed into its own full T_ZONE_S timeout.
            for peer, peer_req_id in self.deferred.pop(zone_id, []):
                self._grant(peer, zone_id, peer_req_id)
            raise ZoneTimeout(zone_id)

        if elapsed > T_GRANT_S * (self.retries.get(zone_id, 0) + 1):
            if self.retries.get(zone_id, 0) < MAX_RETRIES:
                self.retries[zone_id] = self.retries.get(zone_id, 0) + 1
                self.stats["retries"] += 1
                self._resend(zone_id)
        return False

    def _resend(self, zone_id: str) -> None:
        key = self.my_key.get(zone_id)
        if key is None:
            return
        self._send_request(dict(robot_id=self.me, zone_id=zone_id,
                                lamport_ts=key[1], priority_score=key[0],
                                t_enter_est=0.0, t_exit_est=0.0,
                                req_id=self.req_id[zone_id]))

    # -------------------------------------------------------------- releasing
    def release(self, zone_id: str) -> None:
        """Free the zone and honour every deferred request.

        Sending the deferred grants is what makes the protocol live: a deferred
        peer is WAITING FOR THIS MESSAGE, not polling.
        """
        if self.state.get(zone_id) == FREE:
            return
        self.state[zone_id] = FREE
        self.grants.pop(zone_id, None)
        self.ranked.pop(zone_id, None)
        self._last_resend.pop(zone_id, None)
        for key in [k for k in self.grant_lc if k[0] == zone_id]:
            del self.grant_lc[key]
        for peer, peer_req_id in self.deferred.pop(zone_id, []):
            self._grant(peer, zone_id, peer_req_id)
        if self.log:
            self.log(f"ZONE RELEASE {zone_id}")

    def release_all(self) -> None:
        """Called on FAULT or shutdown.

        A faulted robot holding an intersection blocks the fleet indefinitely.
        Releasing on fault turns a robot failure into a short delay instead of
        a fleet-wide stall.
        """
        for zid in list(self.state.keys()):
            self.release(zid)

    def drop_peer(self, peer_id: str) -> None:
        """Discard a peer's grants and deferred requests.

        Call ONLY on boot_id change, FAULT, or GONE (>= 30 s silent). A peer
        that is merely stale/silent keeps its grants and deferrals: it may
        still be physically inside a zone (strict partition, sih-57 case).
        """
        for granters in self.grants.values():
            granters.discard(peer_id)
        for zid in list(self.deferred.keys()):
            self.deferred[zid] = [entry for entry in self.deferred[zid] if entry[0] != peer_id]
        for key in [k for k in self.grant_lc if k[1] == peer_id]:
            del self.grant_lc[key]
        for key in [k for k in self.peer_req_lc if k[1] == peer_id]:
            del self.peer_req_lc[key]

    def held_zones(self) -> List[str]:
        return [z for z, s in self.state.items() if s == HELD]

    def in_any_zone(self) -> bool:
        return any(s == HELD for s in self.state.values())
