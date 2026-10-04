"""
Auctioneer-free task allocation.

THE TRICK: no robot runs the auction. Every robot hears every bid and
independently computes the same winner with the same deterministic rule. The
winner then self-assigns and announces.

    ANNOUNCE  ->  BID (window 0.5 s)  ->  RESOLVE (local, identical)
              ->  AWARD (confirmation, not a request)  ->  MONITOR

Compared with a classic Contract Net, there is no manager role. Compared with
a central allocator, there is no server to lose. The cost is one extra
broadcast per robot per task, which at 3 robots is negligible.

INCREMENT 3+4 ADDITIONS (R4/R5/R6):
  T1  compute_bid lost the IDLE/CHARGING urgency term: ANY robot without a
      task bids, so a robot still driving home but closer to the pickup wins
      over the idle one (R4(3)).
  T2  auctions whose bid windows close within BATCH_WINDOW_S resolve JOINTLY
      as a min-sum-cost assignment (<=5 robots exact, greedy above), which
      fixes the cross-assignment greedy per-task resolution cannot (R4(1)).
  T3  a robot becoming free re-announces the most urgent open task.
  T4  claim collisions are also resolved from peers' broadcast task_id at
      10 Hz (on_peer_task): a LOADED holder always wins, else the lowest id;
      the loser drops silently. An alive assignee broadcasting a different
      task_id for ORPHAN_S orphans the task; LOADED tasks are never
      re-announced before the holder is GONE (R4(4)).
  R5  tasks age: urgency grows as the deadline budget is consumed, feeding
      bid-batch ordering and halving the no-bid retry once overdue.
  R6  on_cancel: a webapp abort before pickup. Cancelled ids live in
      self.cancelled, checked on every path that could resurrect a task.
"""
import math
from typing import Callable, Dict, List, Optional, Set

from .models import Task

# The priority builder owns core/priority.py (urgency / eff_deadline /
# BUDGET_S per the shared API contract). Until it lands, contract-identical
# fallbacks below keep this module self-contained; once the functions exist
# in priority.py they are used (resolved at call time via getattr).
from . import priority as _prio

BID_WINDOW_S = 0.5
BATTERY_FLOOR_PCT = 15.0        # HARD constraint, not a soft penalty
BATTERY_DERATE_PCT = 35.0
# An assignee must be continuously unavailable (DEAD or faulted) this long
# before its task is re-announced. sih-57's mock under load average 60 saw
# heartbeat links DEAD for 10-16 s while the robot drove on fine; 5 s stole
# tasks from living robots, so the release window must exceed those stalls.
TASK_RELEASE_S = 20.0
# A LOADED task is never re-announced before its holder is GONE (peers.py
# tier contract): the item is physically on that robot.
T_GONE_S = 30.0
ORPHAN_S = 3.0                  # assignee alive but claiming another task
BATCH_WINDOW_S = 0.6            # auctions closing this close resolve jointly
BATCH_EXACT_MAX_ROBOTS = 5      # exact assignment up to here, greedy above
PULL_STAGGER_S = 0.5            # non-lowest-id freed robots wait this long
NO_BID_RETRY_S = 5.0            # avoid an unbounded announcement storm
ENERGY_PER_METRE_PCT = 0.35

# Bid cost weights. The urgency and workload terms are gone (T1): cost is
# travel time + remaining dwell + battery risk, nothing else.
W_TRAVEL = 1.0
W_BATTERY = 40.0

# Contract fallbacks (== core/priority.py constants once that lands).
_AGE_STEPS_FALLBACK = (0.5, 0.75, 1.0, 1.25, 1.5)
_BUDGET_S_FALLBACK = {0: 225.0, 1: 188.0, 2: 150.0, 3: 113.0}
_CLASS_CARRYING = 3


def _urgency(task_priority: int, age_frac: float) -> int:
    """0..9; defers to priority.urgency when the priority builder lands."""
    fn = getattr(_prio, "urgency", None)
    if fn is not None:
        return fn(task_priority, age_frac)
    steps = getattr(_prio, "AGE_STEPS", _AGE_STEPS_FALLBACK)
    return min(9, max(0, int(task_priority))
               + sum(1 for s in steps if age_frac >= s))


def _eff_deadline(task, now: float) -> float:
    """task.deadline if set past created_at, else created_at + budget."""
    fn = getattr(_prio, "eff_deadline", None)
    if fn is not None:
        return fn(task, now)
    created = float(getattr(task, "created_at", 0.0) or 0.0)
    if created <= 0.0:
        return float("inf")
    deadline = float(getattr(task, "deadline", 0.0) or 0.0)
    if deadline > created:
        return deadline
    budget = getattr(_prio, "BUDGET_S", _BUDGET_S_FALLBACK)
    prio = min(3, max(0, int(getattr(task, "priority", 0))))
    return created + float(budget[prio])


def age_frac(task, now: float) -> float:
    """Fraction of the task's time budget consumed; 0.0 if unaged."""
    created = float(getattr(task, "created_at", 0.0) or 0.0)
    if created <= 0.0:
        return 0.0
    deadline = _eff_deadline(task, now)
    if not math.isfinite(deadline) or deadline <= created:
        return 0.0
    return max(0.0, (now - created) / (deadline - created))


def task_urgency(task, now: float) -> int:
    """Aged urgency 0..9 used for batch order, pulls and retry halving."""
    return _urgency(int(getattr(task, "priority", 0)), age_frac(task, now))


class TaskAuction:
    def __init__(self, robot_id: str,
                 announce: Callable[[dict], None],
                 bid: Callable[[dict], None],
                 award: Callable[[dict], None],
                 path_length_m: Callable,      # (cell_a, cell_b) -> metres or -1
                 logger=None,
                 release: Optional[Callable[[dict, str], None]] = None):
        self.me = robot_id
        self._announce = announce
        self._bid = bid
        self._award = award
        self._release = release
        self._path_len = path_length_m
        self.log = logger

        self.open_auctions: Dict[str, dict] = {}   # task_id -> {task,bids,t0}
        self.my_task: Optional[Task] = None
        self.assignments: Dict[str, str] = {}      # task_id -> robot_id
        self.task_db: Dict[str, Task] = {}
        self.completed: Set[str] = set()
        # R6: aborted ids. Checked EVERYWHERE completed is, so no retry,
        # re-announce, late announce/award, re-assert or pull resurrects one.
        self.cancelled: Set[str] = set()
        # R5: when I first heard each task (clock-skew-free aging base; the
        # node feeds coord.task_t0 from it at award time).
        self.first_heard: Dict[str, float] = {}
        self._seq = 0
        self._no_bid_retry_after: Dict[str, float] = {}
        # Overridden by the node so it never undercuts the peer dead timeout.
        self.task_release_s = TASK_RELEASE_S
        self._unavailable_since: Dict[str, float] = {}
        self._retry_rank = 0
        # T4: each peer's latest broadcast claim: rid -> [task_id, t_changed,
        # peer_class]; and per-task orphan-mismatch timers.
        self._peer_task: Dict[str, list] = {}
        self._orphan_since: Dict[str, float] = {}
        # T3: a freed robot pulls the most urgent open task after a stagger.
        self._pull_due: Optional[float] = None

        self.stats = {"bids_placed": 0, "tasks_won": 0, "tasks_completed": 0,
                      "reannounced": 0, "claim_collisions": 0,
                      "batches_joint": 0, "cancelled": 0, "pulled": 0}

    # ------------------------------------------------------------- inbound
    def on_announce(self, task: Task, now: float = 0.0) -> None:
        tid = task.task_id
        if tid in self.cancelled:
            return                                 # late announce of an abort
        pending = self.open_auctions.get(tid)
        if ((pending is not None and not pending.get("no_bid_wait"))
                or tid in self.completed or self.assignments.get(tid)):
            return
        self.first_heard.setdefault(tid, now)
        if pending is not None:
            # Someone retried an unbid task I was holding: back my own retry
            # off so only one robot re-announces per period, then bid afresh.
            self._no_bid_retry_after[tid] = max(
                self._no_bid_retry_after.get(tid, 0.0),
                now + NO_BID_RETRY_S * (1 + self._retry_rank))
        if self.my_task is not None and self.my_task.task_id == tid:
            # Someone wrongly thinks I am lost. Re-assert inside their bid
            # window so the auction is cancelled before anyone wins it.
            self._award(dict(task_id=tid, winner_id=self.me, winning_bid=0.0,
                             num_bidders=0, stamp=now))
            if self.log:
                self.log(f"re-asserting ownership of {tid}")
            return
        self.task_db[tid] = task
        self.open_auctions[tid] = dict(task=task, bids={}, t0=now)

        value = self.compute_bid(task)
        if value is not None:
            self.open_auctions[tid]["bids"][self.me] = value
            self._seq += 1
            self.stats["bids_placed"] += 1
            self._bid(dict(robot_id=self.me, task_id=tid, bid=value,
                           est_completion_s=self._last_est,
                           battery_after_pct=self._last_batt_after,
                           seq=self._seq))

    def on_bid(self, msg: dict) -> None:
        if msg["task_id"] in self.cancelled:
            return
        a = self.open_auctions.get(msg["task_id"])
        if a is not None:
            a["bids"][msg["robot_id"]] = float(msg["bid"])

    def on_award(self, msg: dict) -> None:
        tid, winner = msg["task_id"], msg["winner_id"]
        self.open_auctions.pop(tid, None)
        self._no_bid_retry_after.pop(tid, None)
        if tid in self.cancelled:
            # A late award must not resurrect an aborted task; if it is
            # somehow still mine, drop it (the cancel already cleared state).
            if self.my_task is not None and self.my_task.task_id == tid:
                self.my_task = None
            return
        local_claim = self.my_task is not None and self.my_task.task_id == tid

        # Claim collision: bid loss can make multiple robots believe they won.
        # Resolve it identically at every recipient. The lexicographically
        # lowest ID among the conflicting claims wins; keep the assignment
        # table consistent with that decision as well as clearing a loser's
        # active task (the node callback then clears its route).
        effective_winner = winner
        if local_claim and winner != self.me:
            self.stats["claim_collisions"] += 1
            effective_winner = min(self.me, winner)
            if effective_winner == self.me:
                if self.log:
                    self.log(f"claim collision on {tid}, retaining task over {winner}")
            else:
                if self.log:
                    self.log(f"claim collision on {tid}, yielding to {winner}")
                self.my_task = None

        self.assignments[tid] = effective_winner
        self._orphan_since.pop(tid, None)

    # ----------------------------------------------------- peer claims (T4)
    def on_peer_task(self, rid: str, task_id: str, peer_class: int,
                     now: float, my_class: int = 0) -> bool:
        """Resolve duplicate ownership from a peer's broadcast RobotState.

        Called at state rate (10 Hz) with the peer's task_id and priority
        class. Returns True when I must DROP my own claim (the node then
        clears the goal, releases zones and publishes the empty path). The
        loser drops SILENTLY: no release, no re-announce — the winner
        already runs the task.
        """
        if rid == self.me:
            return False
        rec = self._peer_task.get(rid)
        if rec is None or rec[0] != task_id:
            self._peer_task[rid] = [task_id, now, int(peer_class)]
        else:
            rec[2] = int(peer_class)
        if not task_id or task_id in self.completed or task_id in self.cancelled:
            return False

        mine = self.my_task is not None and self.my_task.task_id == task_id
        if mine:
            i_loaded = int(my_class) >= _CLASS_CARRYING
            peer_loaded = int(peer_class) >= _CLASS_CARRYING
            if peer_loaded and not i_loaded:
                lose = True
            elif i_loaded and not peer_loaded:
                lose = False
            else:
                lose = rid < self.me       # identical class: lowest id wins
            if lose:
                self.stats["claim_collisions"] += 1
                self.my_task = None
                self.assignments[task_id] = rid
                self._orphan_since.pop(task_id, None)
                if self.log:
                    self.log(f"claim collision on {task_id} (peer state): "
                             f"yielding to {rid}")
                self.note_freed(now)
                return True
            self.assignments[task_id] = self.me
            return False

        # Not my claim: keep the assignment table honest with the broadcast.
        cur = self.assignments.get(task_id)
        if cur is None or cur == self.me:
            self.assignments[task_id] = rid
            self._orphan_since.pop(task_id, None)
        elif cur != rid:
            # Two distinct claimants, neither me: loaded wins, else lowest id
            # (the same rule the claimants apply between themselves).
            if int(peer_class) >= _CLASS_CARRYING or rid < cur:
                self.assignments[task_id] = rid
                self._orphan_since.pop(task_id, None)
        else:
            self._orphan_since.pop(task_id, None)
        return False

    # ---------------------------------------------------------- cancel (R6)
    def on_cancel(self, task_id: str, now: float,
                  my_phase: Optional[str] = None) -> bool:
        """Abort a task before pickup. Returns False (and changes NOTHING)
        when the task is mine and already loaded (phase DROPOFF); the node
        then logs the refusal and the robot continues to the drop."""
        mine = self.my_task is not None and self.my_task.task_id == task_id
        if mine and my_phase == "DROPOFF":
            return False
        self.cancelled.add(task_id)
        self.open_auctions.pop(task_id, None)
        self.assignments.pop(task_id, None)
        self._no_bid_retry_after.pop(task_id, None)
        self._orphan_since.pop(task_id, None)
        self.stats["cancelled"] += 1
        if mine:
            self.my_task = None
            self.note_freed(now)
            if self.log:
                self.log(f"task {task_id} cancelled; dropped before pickup")
        return True

    # ------------------------------------------------------------- bidding
    def compute_bid(self, task: Task) -> Optional[float]:
        """bid = 1 / (1 + cost). Returns None if I must not bid.

        cost = W_TRAVEL  * travel_s(current cell -> pickup -> dropoff)
             + remaining_dwell_s
             + W_BATTERY * battery_risk

        T1: the urgency term is GONE. Any robot with no task bids — status
        (IDLE, MOVING home, CHARGING-but-able) no longer shapes the bid, so
        a returning robot that is CLOSER to the pickup beats the idle one
        (R4(3)). battery_risk is INFINITE below the floor: a robot that
        strands itself mid-aisle becomes a permanent obstacle for the whole
        fleet, which is far more expensive than any assignment inefficiency.
        That hard constraint stays "do not bid", not a penalty term.
        """
        self._last_est = 0.0
        self._last_batt_after = 0.0

        if self.my_task is not None:
            return None                       # single-task robots, by design
        if self.state_status == "FAULT":
            return None
        if self.charge_hold:
            return None                       # low battery: charging at dock

        d1 = self._path_len(self.current_cell, task.pickup)
        d2 = self._path_len(task.pickup, task.dropoff)
        if d1 < 0 or d2 < 0:
            return None                       # unreachable right now

        total_m = d1 + d2
        travel_s = total_m / max(0.05, self.nominal_speed)
        batt_after = self.battery_pct - ENERGY_PER_METRE_PCT * total_m
        if batt_after <= BATTERY_FLOOR_PCT:
            return None                       # HARD constraint

        risk = (0.0 if batt_after > BATTERY_DERATE_PCT
                else (BATTERY_DERATE_PCT - batt_after) / BATTERY_DERATE_PCT)

        cost = (W_TRAVEL * travel_s + max(0.0, self.remaining_dwell_s)
                + W_BATTERY * risk)

        self._last_est = travel_s
        self._last_batt_after = batt_after
        return round(1.0 / (1.0 + cost), 6)

    # Injected each tick by the agent node so compute_bid stays pure-ish.
    current_cell = (0, 0)
    battery_pct = 100.0
    nominal_speed = 0.4
    state_status = "IDLE"
    charge_hold = False
    remaining_dwell_s = 0.0      # seconds of load/unload dwell still ahead

    # --------------------------------------------------------------- resolve
    def tick(self, now: float, alive_ids: Set[str],
             peers_last_seen: Dict[str, float]) -> None:
        self._resolve_auctions(now, alive_ids)
        self._monitor_assignees(now, peers_last_seen, alive_ids)
        self._pull_open_task(now, alive_ids)

    # The resolver. Normal auctions whose bid windows closed are grouped:
    # every auction closing within BATCH_WINDOW_S of the earliest close
    # resolves JOINTLY (T2); a batch is held open while a member's window is
    # still running, so simultaneous tasks are assigned min-sum-cost.
    def _resolve_auctions(self, now: float, alive_ids: Set[str]) -> None:
        ripe: List[tuple] = []           # (close_t, tid), window elapsed
        pending_close: List[float] = []  # windows still open
        for tid, a in self.open_auctions.items():
            if a.get("no_bid_wait"):
                # The retry timer path below handles these on their due time.
                if now >= a["t0"] + BID_WINDOW_S:
                    ripe.append((a["t0"] + BID_WINDOW_S, tid))
                continue
            close = a["t0"] + BID_WINDOW_S
            if now >= close:
                ripe.append((close, tid))
            else:
                pending_close.append(close)

        while ripe:
            ripe.sort()
            c0 = ripe[0][0]
            # Hold the batch while another auction will close inside its
            # window: resolving early would forfeit the joint assignment.
            if any(c0 < pc <= c0 + BATCH_WINDOW_S + 1e-9
                   for pc in pending_close):
                return
            batch = [tid for close, tid in ripe
                     if close <= c0 + BATCH_WINDOW_S + 1e-9]
            ripe = [(close, tid) for close, tid in ripe
                    if close > c0 + BATCH_WINDOW_S + 1e-9]

            no_bid, bid_tids = [], []
            for tid in batch:
                a = self.open_auctions.get(tid)
                if a is None:
                    continue
                (bid_tids if a["bids"] else no_bid).append(tid)
            for tid in no_bid:
                self._retry_unbid(tid, now, alive_ids)
            if len(bid_tids) == 1:
                self._resolve_single(bid_tids[0], now)
            elif bid_tids:
                self._resolve_joint(bid_tids, now)

    def _retry_unbid(self, tid: str, now: float, alive_ids: Set[str]) -> None:
        """Every robot keeps an unbid task; dropping it on all but one robot
        strands it fleet-wide when that one never heard the announce. Retries
        are staggered by live-id rank: the lowest retries every period, and
        rank k only fires after k extra periods without hearing anyone
        else's retry. R5: the period HALVES once the task is overdue."""
        a = self.open_auctions.pop(tid)
        if tid in self.cancelled:
            return
        period = NO_BID_RETRY_S * (0.5 if age_frac(a["task"], now) >= 1.0
                                   else 1.0)
        ranked = sorted(set(alive_ids) | {self.me})
        self._retry_rank = ranked.index(self.me)
        due = self._no_bid_retry_after.get(tid, now + period * self._retry_rank)
        if now >= due:
            self.stats["reannounced"] += 1
            self._announce(self._task_to_dict(a["task"]))
            due = now + period * (1 + self._retry_rank)
        self._no_bid_retry_after[tid] = due
        a["t0"] = due - BID_WINDOW_S
        a["no_bid_wait"] = True
        self.open_auctions[tid] = a

    def _resolve_single(self, tid: str, now: float) -> None:
        a = self.open_auctions.pop(tid, None)
        if a is None or tid in self.cancelled:
            return
        bids = a["bids"]
        # DETERMINISTIC: highest bid, tie broken by lexicographic robot_id.
        # Every robot sorts the same dict and gets the same answer.
        winner = min(bids.keys(), key=lambda r: (-bids[r], r))
        self._apply_win(tid, a, winner, bids, now)

    def _resolve_joint(self, tids: List[str], now: float) -> None:
        """Min-sum-cost assignment over the batch (T2, R4(1)).

        cost(r, t) = 1/bid - 1 (the bid formula inverted). Up to
        BATCH_EXACT_MAX_ROBOTS robots the optimum is found exactly over all
        injective assignments, preferring MORE tasks assigned, then lower
        total cost; the enumeration order (tasks by (eff_deadline,
        created_at, id), robots sorted, strict improvement only) makes ties
        resolve to the lower robot_id / earlier deadline on every robot
        identically. Above that, greedy in (-urgency, created_at, task_id)
        order. Tasks left without a robot re-enter the no-bid retry path.
        """
        auctions = {tid: self.open_auctions.pop(tid) for tid in tids
                    if tid in self.open_auctions}
        tids = [t for t in auctions if t not in self.cancelled]
        if not tids:
            return
        self.stats["batches_joint"] += 1
        robots = sorted({r for t in tids for r in auctions[t]["bids"]})

        def cost(rid, tid):
            b = auctions[tid]["bids"].get(rid)
            return None if b is None or b <= 0.0 else (1.0 / b) - 1.0

        if len(robots) <= BATCH_EXACT_MAX_ROBOTS:
            order = sorted(tids, key=lambda t: (
                _eff_deadline(auctions[t]["task"], now),
                float(getattr(auctions[t]["task"], "created_at", 0.0)), t))
            best = {"n": -1, "cost": float("inf"), "asg": {}}

            def search(i, used, asg, total):
                if i == len(order):
                    n = len(asg)
                    if (n > best["n"]
                            or (n == best["n"] and total < best["cost"] - 1e-12)):
                        best["n"], best["cost"] = n, total
                        best["asg"] = dict(asg)
                    return
                t = order[i]
                for r in robots:               # ascending id: ties -> low id
                    if r in used:
                        continue
                    c = cost(r, t)
                    if c is None:
                        continue
                    asg[t] = r
                    search(i + 1, used | {r}, asg, total + c)
                    del asg[t]
                search(i + 1, used, asg, total)   # leave t unassigned
            search(0, frozenset(), {}, 0.0)
            assignment = best["asg"]
        else:
            assignment = {}
            used = set()
            for t in sorted(tids, key=lambda t: (
                    -task_urgency(auctions[t]["task"], now),
                    float(getattr(auctions[t]["task"], "created_at", 0.0)), t)):
                cands = [(cost(r, t), r) for r in robots
                         if r not in used and cost(r, t) is not None]
                if cands:
                    _, r = min(cands)
                    assignment[t] = r
                    used.add(r)

        for tid in tids:
            winner = assignment.get(tid)
            if winner is None:
                # Bidders existed but were all taken by other batch members:
                # requeue through the no-bid retry machinery.
                a = auctions[tid]
                a["bids"] = {}
                a["t0"] = now
                a["no_bid_wait"] = True
                self._no_bid_retry_after.setdefault(tid, now + BID_WINDOW_S)
                self.open_auctions[tid] = a
                continue
            self._apply_win(tid, auctions[tid], winner,
                            auctions[tid]["bids"], now)

    def _apply_win(self, tid: str, a: dict, winner: str, bids: dict,
                   now: float) -> None:
        if (winner == self.me and self.my_task is not None
                and self.my_task.task_id != tid):
            # Overlapping bid windows can hand a busy robot a second win.
            # Overwriting my_task would silently starve the first task
            # fleet-wide (peers keep it assigned to me). Return this win
            # to the pool instead; compute_bid is None while busy, so no
            # self-rewin loop.
            self.assignments.pop(tid, None)
            if self._release:
                self._release(self._task_to_dict(a["task"]),
                              "won while busy; returning to auction")
            self.on_announce(a["task"], now=now)
            return
        self.assignments[tid] = winner
        self._orphan_since.pop(tid, None)
        if winner == self.me:
            self.my_task = a["task"]
            self._pull_due = None
            self.stats["tasks_won"] += 1
            self._award(dict(task_id=tid, winner_id=self.me,
                             winning_bid=bids[self.me],
                             num_bidders=len(bids), stamp=now))
            if self.log:
                self.log(f"WON {tid} bid={bids[self.me]:.6f} "
                         f"({len(bids)} bidders)")

    def _monitor_assignees(self, now: float, last_seen: Dict[str, float],
                           alive_ids: Set[str]) -> None:
        """Re-announce a dead robot's task. No supervisor needed - every robot
        reaches this conclusion at the same moment."""
        for rid in list(self._unavailable_since):
            if rid in alive_ids:
                del self._unavailable_since[rid]
        for tid, assignee in list(self.assignments.items()):
            if (assignee == self.me or tid in self.completed
                    or tid in self.cancelled):
                continue
            seen = last_seen.get(assignee, 0.0)
            rec = self._peer_task.get(assignee)
            loaded = rec is not None and rec[0] == tid and rec[2] >= _CLASS_CARRYING
            # A LOADED task is only released once the holder is GONE: the
            # item is on that robot and nobody else can deliver it (T4).
            release_after = max(self.task_release_s,
                                T_GONE_S if loaded else 0.0)
            # DEAD or faulted (alive=false) only counts once it has lasted
            # release_after; heartbeat silence is the fallback for a peer
            # that was never heard from at all.
            if assignee in alive_ids:
                unavailable = False
            else:
                since = self._unavailable_since.setdefault(assignee, now)
                unavailable = (now - since) > release_after
            silent = (now - seen) > release_after

            # ORPHAN (T4): the assignee is alive but has been broadcasting a
            # DIFFERENT task_id continuously for ORPHAN_S — it dropped or
            # swapped the task without a release reaching us.
            orphan = False
            if assignee in alive_ids and rec is not None and rec[0] != tid:
                since = self._orphan_since.setdefault(tid, now)
                orphan = (now - since) >= ORPHAN_S
            elif tid in self._orphan_since:
                self._orphan_since.pop(tid, None)

            if unavailable or silent or orphan:
                self.assignments.pop(tid, None)
                self._orphan_since.pop(tid, None)
                task = self.task_db.get(tid)
                if task is None:
                    continue
                # Deduplication guard, NOT authority: without it every survivor
                # re-announces simultaneously and you get an announce storm.
                # The fleet still works without it, just noisily.
                candidates = set(alive_ids) | {self.me}
                if candidates and self.me == min(candidates):
                    self.stats["reannounced"] += 1
                    self._announce(self._task_to_dict(task))
                    if self.log:
                        why = "orphaned" if orphan else "assignee lost"
                        self.log(f"re-announcing {tid} ({why}: {assignee})")

    # ---------------------------------------------------------- freed pull
    def note_freed(self, now: float) -> None:
        """Mark that my task slot just opened (T3)."""
        self._pull_due = now

    def _pull_open_task(self, now: float, alive_ids: Set[str]) -> None:
        """A robot becoming free re-announces the most urgent open task:
        highest (urgency, age) first. The lowest-id alive robot pulls
        immediately; everyone else waits PULL_STAGGER_S so one announce per
        freeing, not a storm."""
        if self.my_task is not None:
            self._pull_due = None
            return
        if self._pull_due is None:
            return
        ranked = sorted(set(alive_ids) | {self.me})
        delay = 0.0 if ranked[0] == self.me else PULL_STAGGER_S
        if now < self._pull_due + delay:
            return
        self._pull_due = None
        open_tids = [
            tid for tid, task in self.task_db.items()
            if tid not in self.completed and tid not in self.cancelled
            and tid not in self.assignments
            and (tid not in self.open_auctions
                 or self.open_auctions[tid].get("no_bid_wait"))]
        if not open_tids:
            return
        best = min(open_tids, key=lambda tid: (
            -task_urgency(self.task_db[tid], now),
            float(getattr(self.task_db[tid], "created_at", 0.0)), tid))
        task = self.task_db[best]
        self.open_auctions.pop(best, None)
        self._no_bid_retry_after.pop(best, None)
        self.stats["pulled"] += 1
        self.stats["reannounced"] += 1
        self._announce(self._task_to_dict(task))
        self.on_announce(task, now=now)

    def on_complete(self, task_id: str) -> None:
        """Apply a distributed completion event from the winning robot."""
        self.open_auctions.pop(task_id, None)
        self.assignments.pop(task_id, None)
        self.completed.add(task_id)
        self._no_bid_retry_after.pop(task_id, None)
        self._orphan_since.pop(task_id, None)
        if self.my_task is not None and self.my_task.task_id == task_id:
            self.my_task = None

    def complete_current(self, now: float) -> None:
        if self.my_task is None:
            return
        tid = self.my_task.task_id
        self.on_complete(tid)
        self.stats["tasks_completed"] += 1
        if self.log:
            self.log(f"completed {tid}")
        self.my_task = None
        self.note_freed(now)

    def on_release(self, task: Task, robot_id: str, now: float) -> bool:
        """Clear an assignee and reopen bidding after a peer releases a task."""
        tid = task.task_id
        if tid in self.completed or tid in self.cancelled:
            return False
        assignee = self.assignments.get(tid)
        if assignee is not None and assignee != robot_id:
            return False
        if (assignee is None and tid in self.open_auctions
                and not self.open_auctions[tid].get("no_bid_wait")):
            return False  # duplicate or stale release; do not reset the bid timer
        if (self.my_task is not None and self.my_task.task_id == tid and
                robot_id != self.me):
            self.my_task = None
        self.assignments.pop(tid, None)
        self.open_auctions.pop(tid, None)
        self._orphan_since.pop(tid, None)
        self.task_db[tid] = task
        self.on_announce(task, now=now)
        return True

    def release_current(self, now: float = 0.0,
                        reason: str = "assignee released task") -> None:
        """Return my task to all peers and reopen its auction locally."""
        if self.my_task is None:
            return
        task = self.my_task
        self.my_task = None
        self.assignments.pop(task.task_id, None)
        task_data = self._task_to_dict(task)
        if self._release is None:
            # Backward-compatible behavior for core-only callers that do not
            # provide a distributed release transport.
            self._announce(task_data)
        else:
            self._release(task_data, reason)
        self.on_announce(task, now=now)

    @staticmethod
    def _task_to_dict(t: Task) -> dict:
        return dict(task_id=t.task_id, pickup_row=t.pickup[0],
                    pickup_col=t.pickup[1], dropoff_row=t.dropoff[0],
                    dropoff_col=t.dropoff[1], priority=t.priority,
                    created_at=t.created_at, deadline=t.deadline)
