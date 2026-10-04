"""
Decentralised deadlock detection via a distributed wait-for graph.

HOW IT IS DECENTRALISED
Every robot broadcasts a single field: waiting_for (the robot_id it is
currently blocked by, or ""). Every robot collects those edges from the peer
states it already receives, and builds the SAME directed graph locally.

    robot_1.waiting_for = "robot_2"     ->  R1 -> R2
    robot_2.waiting_for = "robot_3"     ->  R2 -> R3
    robot_3.waiting_for = "robot_1"     ->  R3 -> R1     CYCLE

Because every robot has the same edges, every robot finds the same cycle, and
because victim selection uses the same total order from priority.py, every
robot picks the same victim WITH NO NEGOTIATION ROUND AT ALL. The only
messages involved are the state broadcasts that were already happening.

TWO INDEPENDENT TRIGGERS - implement BOTH
  structural: a cycle in the wait-for graph. Fast (about 200 ms) and precise.
  timeout   : waiting longer than T_DEADLOCK with no progress. This is the
              safety net. The structural detector trusts peers to report
              waiting_for correctly; if that message is lost, or a peer has a
              bug, or a peer crashed while holding a zone, ONLY the timeout
              saves you.
"""
from typing import Dict, List, Optional

from .models import RobotState
from .priority import select_deadlock_victim

T_DEADLOCK_S = 8.0        # timeout trigger
T_YIELD_S = 15.0          # a victim that cannot yield in this long escalates
# Timeout with no structural cycle: the waiting PAIR (me + waiting_for) is
# resolved by priority, so a CARRYING robot never self-victimises against a
# lower-priority blocker (measured: group=[me] livelocked 6 of 7 noise-free
# seeds at 0-7 deliveries/480 s, with loaded robots retreating 0.1 m from
# their dropoff). If holding has not worked for this long, fall back to the
# old single-member group so a non-cooperating blocker cannot park me forever.
# 15 s: make-way on the blocker fires within ~0.5 s when it can move at all,
# and a zone-queue transit behind a crossing robot is well under this; 24 s
# measurably pushed converge waits to 24.2 s (> the 20 s AC7-adjacent bound).
T_TIMEOUT_SELF_S = 15.0

# Minimum time a robot must have been waiting before a STRUCTURAL cycle is
# believed. Without this the detector fires on transient handshake states: two
# robots that have both just sent a ZoneRequest briefly point waiting_for at
# each other, which is a textbook 2-cycle that resolves itself within one round
# trip. Reacting to it causes needless rerouting. 1.0 s is ~10 ticks at 10 Hz,
# far longer than a LAN round trip and far shorter than a real deadlock.
T_MIN_CYCLE_WAIT_S = 1.0


class DeadlockManager:
    def __init__(self, robot_id: str, logger=None,
                 t_deadlock: float = T_DEADLOCK_S,
                 t_yield: float = T_YIELD_S,
                 t_min_cycle_wait: float = T_MIN_CYCLE_WAIT_S,
                 t_timeout_self: float = T_TIMEOUT_SELF_S):
        self.me = robot_id
        self.log = logger
        self.t_deadlock = t_deadlock
        self.t_yield = t_yield
        self.t_min_cycle_wait = t_min_cycle_wait
        self.t_timeout_self = t_timeout_self

        self.wait_start: Optional[float] = None
        self.yield_start: Optional[float] = None
        self.current_cycle: List[str] = []
        self.events: List[dict] = []      # feeds your metrics/results directly
        # Election log dedup/rate-limit (webots_strictlanes1: 1868 identical
        # "DEADLOCK timeout victim=robot_1" lines at 10 Hz while the elected
        # victim, holding GO, could never act). One line per (group, victim,
        # trigger) change, else one per LOG_EVERY_S.
        self._last_log_key = None
        self._last_log_t: Optional[float] = None

    LOG_EVERY_S = 5.0

    # ------------------------------------------------------------- detection
    @staticmethod
    def build_wait_for_graph(me: RobotState,
                             peers: Dict[str, RobotState]) -> Dict[str, str]:
        g: Dict[str, str] = {}
        if me.waiting_for:
            g[me.robot_id] = me.waiting_for
        for pid, p in peers.items():
            if p.alive and p.waiting_for:
                g[pid] = p.waiting_for
        return g

    @staticmethod
    def find_cycle(graph: Dict[str, str], start: str) -> Optional[List[str]]:
        """Walk the single out-edge chain from `start`.

        Each node has at most one outgoing edge (a robot waits for exactly one
        other robot), so cycle detection is a simple walk with a visited set -
        no full DFS needed. O(n).

        Returns the cycle in order starting at `start`, or None. Note it
        returns None if the walk enters a cycle that does NOT contain `start`:
        that is another robot's problem to resolve, not ours.
        """
        seen = set()
        path: List[str] = []
        node: Optional[str] = start
        while node is not None and node not in seen:
            seen.add(node)
            path.append(node)
            node = graph.get(node)
        if node == start and len(path) >= 2:
            return path
        return None

    # ------------------------------------------------------------------ tick
    def tick(self, me: RobotState, peers: Dict[str, RobotState],
             now: float, timeout_trigger: bool = True) -> Optional[dict]:
        """Run once per control tick.

        Returns None, or a dict:
            {"role": "HOLD"}                      - someone else yields
            {"role": "VICTIM", "cycle": [...]}    - I must yield

        timeout_trigger=False suppresses the no-cycle timeout trigger for
        this tick (structural cycles are still detected). The coordinator
        uses it while a robot waits at a traffic acquisition point: that wait
        is bounded by the rank order (core/traffic.py), not a deadlock.
        """
        if me.status != "WAITING":
            self.wait_start = None
            self.current_cycle = []
            return None

        if self.wait_start is None:
            self.wait_start = now
        waited = now - self.wait_start

        graph = self.build_wait_for_graph(me, peers)
        cycle = self.find_cycle(graph, me.robot_id)

        # Ignore a cycle that is merely a transient handshake state.
        if cycle and waited < self.t_min_cycle_wait:
            cycle = None

        timed_out = timeout_trigger and waited > self.t_deadlock

        if not cycle and not timed_out:
            return None

        # The blocker is already clearing the way (retreat or make-way in
        # progress, both time-bounded) AND is actually moving: waiting for it
        # is PROGRESS, not a deadlock. Without this, the waiter
        # self-victimised at the timeout while its blocker was still driving
        # to its refuge, so BOTH parties retreated and resumed into the
        # identical standoff (the measured mutual-yield churn that gridlocked
        # noise-free seeds 7/13/21). A yielder that is PINNED (stationary -
        # e.g. its only escape runs under me) cannot clear anything, so then
        # I must be the one to move: the group collapses to [me] at once
        # (measured: suppressing on a pinned yielder froze converge at 0
        # pickups with a 189 s wait).
        blocker_pinned = False
        if not cycle:
            p = peers.get(me.waiting_for) if me.waiting_for else None
            if p is not None and p.alive and p.status == "YIELDING":
                if abs(getattr(p, "v", 0.0)) >= 0.05:
                    return None
                blocker_pinned = True

        # Timeout with no structural cycle: resolve the waiting PAIR by
        # priority (R1) - include the blocker I am waiting for, so a
        # lower-priority blocker is the victim and a CARRYING robot holds
        # course. Both sides compare the same two broadcast scores, so the
        # decision agrees without negotiation. After t_timeout_self of no
        # progress the group collapses back to [me]: a blocker that never
        # cooperates (idle, faulted logic, not WAITING itself) must not be
        # able to park me forever.
        group = cycle
        if not group:
            group = [me.robot_id]
            blocker = getattr(me, "waiting_for", "") or ""
            if (blocker and blocker in peers and not blocker_pinned
                    and waited <= self.t_timeout_self):
                # ACTIONABLE victims only (webots_strictlanes1 fix): a
                # blocker that is NEITHER waiting NOR moving - the wedged /
                # collision-monitor-held robot broadcasting GO with v = 0,
                # or an idle robot parked mid-lane - can never run the
                # victim branch (its own detector acts only while WAITING),
                # so electing it is a no-op forever: robot_1 was elected
                # 1868 times with a GO permit and never acted while robot_3
                # held. For such a blocker the group collapses to [me] at
                # once, so the only robot that CAN act (me) retreats/
                # reroutes/overtakes instead of waiting out t_timeout_self.
                # A WAITING blocker keeps the pair election (it will yield),
                # and a MOVING one too (its passage resolves my wait - the
                # old semantics, which self-victimising against measurably
                # cost throughput).
                bst = peers[blocker]
                if (getattr(bst, "status", "") == "WAITING"
                        or abs(getattr(bst, "v", 0.0) or 0.0) >= 0.05):
                    group = [me.robot_id, blocker]
        self.current_cycle = group

        scores = {me.robot_id: me.priority_score}
        for rid in group:
            if rid in peers:
                scores[rid] = peers[rid].priority_score
        victim = select_deadlock_victim(group, scores)

        if self.log:
            key = (tuple(group), victim, bool(cycle))
            due = (self._last_log_t is None
                   or now - self._last_log_t >= self.LOG_EVERY_S)
            if key != self._last_log_key or due:
                self._last_log_key, self._last_log_t = key, now
                if cycle:
                    self.log(f"DEADLOCK cycle={' -> '.join(group)} "
                             f"victim={victim}")
                else:
                    self.log(f"DEADLOCK timeout after {waited:.1f}s "
                             f"victim={victim}")

        if victim != me.robot_id:
            return {"role": "HOLD", "cycle": group, "victim": victim}

        if self.yield_start is None:
            self.yield_start = now
            self.events.append(dict(t_detect=now, cycle=list(group),
                                    trigger="cycle" if cycle else "timeout",
                                    resolved=False, recovery_time=None))

        escalate = (now - self.yield_start) > self.t_yield
        return {"role": "VICTIM", "cycle": group, "victim": victim,
                "escalate": escalate}

    def mark_resolved(self, now: float) -> None:
        """Call when the robot starts moving again after yielding."""
        if self.yield_start is not None and self.events:
            ev = self.events[-1]
            if not ev["resolved"]:
                ev["resolved"] = True
                ev["recovery_time"] = now - ev["t_detect"]
                if self.log:
                    self.log(f"DEADLOCK resolved in {ev['recovery_time']:.2f}s")
        self.yield_start = None
        self.wait_start = None
        self.current_cycle = []
        self._last_log_key = None       # a new episode logs its election

    # ---------------------------------------------------------------- report
    def metrics(self) -> dict:
        resolved = [e for e in self.events if e["resolved"]]
        times = [e["recovery_time"] for e in resolved]
        return dict(
            deadlocks_detected=len(self.events),
            deadlocks_resolved=len(resolved),
            mean_recovery_s=(sum(times) / len(times)) if times else 0.0,
            max_recovery_s=max(times) if times else 0.0)
