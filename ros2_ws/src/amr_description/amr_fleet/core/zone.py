"""
Distributed mutual exclusion over named capacity-1 zones.

Ricart-Agrawala, with the ordering key changed from pure Lamport timestamp to
(priority_score, lamport, robot_id). Priority decides first so the fleet makes
GOOD decisions; Lamport and robot_id make the order TOTAL so it makes
CONSISTENT ones.

SAFETY INVARIANT
    A robot enters zone Z only after receiving granted=true from EVERY peer
    that is alive AND needs Z within the horizon.

    It never enters on the basis of "I computed that I have priority".
    That distinction is the difference between a system that works and one
    that collides when a single packet is lost.

TIMEOUTS EXIST BECAUSE THE NETWORK IS NOT PERFECT
    A lost grant must never wedge the fleet. Three separate escapes:
      - no reply from a live peer in T_GRANT   -> resend (up to MAX_RETRIES)
      - peer goes dead                          -> its grant is no longer required
      - total acquisition exceeds T_ZONE        -> abandon, penalise, reroute
"""
from typing import Callable, Dict, List, Optional, Set

from .priority import wins

FREE, REQUESTING, HELD = "FREE", "REQUESTING", "HELD"

T_GRANT_S = 1.0          # resend a request if a live peer has not replied
MAX_RETRIES = 3
T_ZONE_S = 10.0          # give up on the zone entirely and reroute
T_RECLAIM_S = 2.0        # force-release a dead peer's zone after this


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
        self._next_req_id = 1

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
                t_enter: float, t_exit: float, now: float = 0.0) -> None:
        if self.state.get(zone_id) in (REQUESTING, HELD):
            return                                  # already in progress
        lc = self.tick_clock()
        rid = self._next_req_id
        self._next_req_id += 1

        self.state[zone_id] = REQUESTING
        self.grants[zone_id] = set()
        self.deferred.setdefault(zone_id, [])
        self.req_time[zone_id] = now
        self.req_id[zone_id] = rid
        self.retries[zone_id] = 0
        self.my_key[zone_id] = (my_score, lc, self.me)
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

        self.tick_clock(int(msg.get("lamport_ts", 0)))
        st = self.state.get(zid, FREE)

        if st == HELD:
            self.deferred.setdefault(zid, []).append((peer, int(msg.get("req_id", 0))))
            self.stats["deferrals"] += 1
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
                self.deferred.setdefault(zid, []).append((peer, int(msg.get("req_id", 0))))
                self.stats["deferrals"] += 1
                return

        if st == REQUESTING and i_need_zone:
            their = (float(msg["priority_score"]), int(msg["lamport_ts"]), peer)
            mine = self.my_key.get(zid, (0.0, 0, self.me))
            if wins(their, mine):
                self._grant(peer, zid, msg["req_id"])   # they beat me
            else:
                self.deferred.setdefault(zid, []).append((peer, int(msg.get("req_id", 0))))
                self.stats["deferrals"] += 1
            return

        self._grant(peer, zid, msg["req_id"])

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
        self.tick_clock(int(msg.get("lamport_ts", 0)))
        if msg.get("granted", False):
            self.grants.setdefault(zid, set()).add(msg["granter_id"])
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

        elapsed = now - self.req_time.get(zone_id, now)
        if elapsed > T_ZONE_S:
            self.stats["timeouts"] += 1
            self.state[zone_id] = FREE
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
        """A peer died: stop waiting for its grant and discard its deferrals."""
        for zid in list(self.deferred.keys()):
            self.deferred[zid] = [entry for entry in self.deferred[zid] if entry[0] != peer_id]

    def held_zones(self) -> List[str]:
        return [z for z, s in self.state.items() if s == HELD]

    def in_any_zone(self) -> bool:
        return any(s == HELD for s in self.state.values())
