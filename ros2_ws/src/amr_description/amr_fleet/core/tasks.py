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
"""
from typing import Callable, Dict, List, Optional, Set

from .models import Task

BID_WINDOW_S = 0.5
BATTERY_FLOOR_PCT = 15.0        # HARD constraint, not a soft penalty
BATTERY_DERATE_PCT = 35.0
TASK_RELEASE_S = 3.0            # dead assignee -> re-announce after this
ENERGY_PER_METRE_PCT = 0.35

# Bid cost weights
W_TRAVEL = 1.0
W_BATTERY = 40.0
W_WORKLOAD = 1.0
W_URGENCY = 15.0


class TaskAuction:
    def __init__(self, robot_id: str,
                 announce: Callable[[dict], None],
                 bid: Callable[[dict], None],
                 award: Callable[[dict], None],
                 path_length_m: Callable,      # (cell_a, cell_b) -> metres or -1
                 logger=None):
        self.me = robot_id
        self._announce = announce
        self._bid = bid
        self._award = award
        self._path_len = path_length_m
        self.log = logger

        self.open_auctions: Dict[str, dict] = {}   # task_id -> {task,bids,t0}
        self.my_task: Optional[Task] = None
        self.assignments: Dict[str, str] = {}      # task_id -> robot_id
        self.task_db: Dict[str, Task] = {}
        self.completed: Set[str] = set()
        self._seq = 0

        self.stats = {"bids_placed": 0, "tasks_won": 0, "tasks_completed": 0,
                      "reannounced": 0, "claim_collisions": 0}

    # ------------------------------------------------------------- inbound
    def on_announce(self, task: Task, now: float = 0.0) -> None:
        tid = task.task_id
        if (tid in self.open_auctions or tid in self.completed
                or self.assignments.get(tid)):
            return
        if self.my_task is not None and self.my_task.task_id == tid:
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
        a = self.open_auctions.get(msg["task_id"])
        if a is not None:
            a["bids"][msg["robot_id"]] = float(msg["bid"])

    def on_award(self, msg: dict) -> None:
        tid, winner = msg["task_id"], msg["winner_id"]
        self.open_auctions.pop(tid, None)
        self.assignments[tid] = winner

        # Claim collision: both of us think we won (divergent bid sets caused
        # by packet loss). Lower robot_id keeps it; I release immediately.
        if (self.my_task is not None and self.my_task.task_id == tid
                and winner != self.me and winner < self.me):
            self.stats["claim_collisions"] += 1
            if self.log:
                self.log(f"claim collision on {tid}, yielding to {winner}")
            self.my_task = None

    # ------------------------------------------------------------- bidding
    def compute_bid(self, task: Task) -> Optional[float]:
        """bid = 1 / (1 + cost). Returns None if I must not bid.

        cost = W_TRAVEL   * travel_time
             + W_BATTERY  * battery_risk
             + W_WORKLOAD * queued_work
             + W_URGENCY  * (1 - urgency_match)

        battery_risk is INFINITE below the floor. A robot that strands itself
        mid-aisle becomes a permanent obstacle for the whole fleet, which is
        far more expensive than any assignment inefficiency. That makes this a
        hard constraint expressed as "do not bid", not a penalty term.
        """
        self._last_est = 0.0
        self._last_batt_after = 0.0

        if self.my_task is not None:
            return None                       # single-task robots, by design
        if self.state_status == "FAULT":
            return None

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
        workload = 0.0 if self.my_task is None else 20.0
        urgency = 1.0 if (task.priority >= 3 and self.state_status == "IDLE") else 0.5

        cost = (W_TRAVEL * travel_s + W_BATTERY * risk
                + W_WORKLOAD * workload + W_URGENCY * (1.0 - urgency))

        self._last_est = travel_s
        self._last_batt_after = batt_after
        return round(1.0 / (1.0 + cost), 6)

    # Injected each tick by the agent node so compute_bid stays pure-ish.
    current_cell = (0, 0)
    battery_pct = 100.0
    nominal_speed = 0.4
    state_status = "IDLE"

    # --------------------------------------------------------------- resolve
    def tick(self, now: float, alive_ids: Set[str],
             peers_last_seen: Dict[str, float]) -> None:
        self._resolve_auctions(now)
        self._monitor_assignees(now, peers_last_seen, alive_ids)

    def _resolve_auctions(self, now: float) -> None:
        for tid, a in list(self.open_auctions.items()):
            if (now - a["t0"]) < BID_WINDOW_S:
                continue
            self.open_auctions.pop(tid)
            bids = a["bids"]
            if not bids:
                # Nobody could take it (all busy / low battery). Put it back.
                self.stats["reannounced"] += 1
                self._announce(self._task_to_dict(a["task"]))
                continue

            # DETERMINISTIC: highest bid, tie broken by lexicographic robot_id.
            # Every robot sorts the same dict and gets the same answer.
            winner = min(bids.keys(), key=lambda r: (-bids[r], r))
            self.assignments[tid] = winner
            if winner == self.me:
                self.my_task = a["task"]
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
        for tid, assignee in list(self.assignments.items()):
            if assignee == self.me or tid in self.completed:
                continue
            seen = last_seen.get(assignee, 0.0)
            if (now - seen) > TASK_RELEASE_S:
                self.assignments.pop(tid, None)
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
                        self.log(f"re-announcing {tid} (assignee {assignee} lost)")

    def on_complete(self, task_id: str) -> None:
        """Apply a distributed completion event from the winning robot."""
        self.open_auctions.pop(task_id, None)
        self.assignments.pop(task_id, None)
        self.completed.add(task_id)
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

    def release_current(self) -> None:
        """Give up my task (low battery, fault). It returns to auction."""
        if self.my_task is None:
            return
        task = self.my_task
        self.my_task = None
        self.assignments.pop(task.task_id, None)
        self._announce(self._task_to_dict(task))

    @staticmethod
    def _task_to_dict(t: Task) -> dict:
        return dict(task_id=t.task_id, pickup_row=t.pickup[0],
                    pickup_col=t.pickup[1], dropoff_row=t.dropoff[0],
                    dropoff_col=t.dropoff[1], priority=t.priority,
                    created_at=t.created_at, deadline=t.deadline)
