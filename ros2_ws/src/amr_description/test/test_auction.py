"""Auctioneer-free task allocation."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.models import Task
from amr_fleet.core.tasks import NO_BID_RETRY_S, TASK_RELEASE_S, TaskAuction


def mk(rid, cell=(0, 0), batt=100.0):
    a = TaskAuction(rid, lambda d: None, lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: abs(p[0] - q[0]) + abs(p[1] - q[1]))
    a.current_cell = cell
    a.battery_pct = batt
    a.nominal_speed = 0.4
    a.state_status = "IDLE"
    return a


TASK = Task("T-1", pickup=(5, 5), dropoff=(10, 10), priority=1)


def test_nearer_robot_bids_higher():
    near = mk("robot_1", (4, 4)).compute_bid(TASK)
    far = mk("robot_2", (30, 30)).compute_bid(TASK)
    assert near > far


def test_low_battery_robot_refuses_to_bid():
    """HARD constraint. A robot that would strand itself must not bid at all -
    it would become a permanent obstacle for the whole fleet."""
    assert mk("robot_1", (4, 4), batt=16.0).compute_bid(TASK) is None


def test_busy_robot_does_not_bid():
    a = mk("robot_1")
    a.my_task = TASK
    assert a.compute_bid(Task("T-2", (1, 1), (2, 2))) is None


def test_unreachable_task_not_bid():
    a = mk("robot_1")
    a._path_len = lambda p, q: -1.0
    assert a.compute_bid(TASK) is None


def test_all_robots_pick_the_same_winner():
    """The heart of the auctioneer-free design: identical bid sets must give
    identical winners on every robot, in any iteration order."""
    bids = {"robot_1": 0.20, "robot_2": 0.55, "robot_3": 0.31}
    winners = set()
    for order in ([("robot_1", 0.20), ("robot_2", 0.55), ("robot_3", 0.31)],
                  [("robot_3", 0.31), ("robot_1", 0.20), ("robot_2", 0.55)],
                  [("robot_2", 0.55), ("robot_3", 0.31), ("robot_1", 0.20)]):
        d = dict(order)
        winners.add(min(d.keys(), key=lambda r: (-d[r], r)))
    assert winners == {"robot_2"}


def test_tie_broken_by_lowest_id():
    d = {"robot_3": 0.5, "robot_1": 0.5, "robot_2": 0.5}
    assert min(d.keys(), key=lambda r: (-d[r], r)) == "robot_1"


def test_claim_collision_lower_id_keeps_task():
    a = mk("robot_2")
    a.my_task = TASK
    a.on_award({"task_id": "T-1", "winner_id": "robot_1"})
    assert a.my_task is None, "higher id must yield on a claim collision"


def test_claim_collision_higher_id_yields_not_me():
    a = mk("robot_1")
    a.my_task = TASK
    a.on_award({"task_id": "T-1", "winner_id": "robot_2"})
    assert a.my_task is not None, "lower id must KEEP the task"


def test_failed_task_reannouncement_elects_self_when_lowest_alive_id():
    sent = []
    a = TaskAuction("robot_1", lambda d: sent.append(d), lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: 1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    a.tick(TASK_RELEASE_S + 5.0, {"robot_3"}, {"robot_2": 0.0})
    assert len(sent) == 1
    assert sent[0]["task_id"] == TASK.task_id


def test_failed_task_reannouncement_not_duplicated_by_higher_id():
    sent = []
    a = TaskAuction("robot_3", lambda d: sent.append(d), lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: 1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    a.tick(TASK_RELEASE_S + 5.0, {"robot_1"}, {"robot_2": 0.0})
    assert sent == []


def test_brief_dead_blip_does_not_steal_a_task():
    """Webots run: heartbeats stalled 0.2-4.4 s under CPU load, the assignee
    went DEAD for a moment and its loaded task was re-auctioned to a peer."""
    sent = []
    a = TaskAuction("robot_1", lambda d: sent.append(d), lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: 1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    for t in (10.0, 11.0, 12.0, 13.0):               # DEAD but heard 1.5 s ago
        a.tick(t, {"robot_3"}, {"robot_2": t - 1.5})
    a.tick(13.5, {"robot_2", "robot_3"}, {"robot_2": 13.5})   # recovered
    for t in (14.0, 15.0, 16.0, 17.0):               # a second, separate blip
        a.tick(t, {"robot_3"}, {"robot_2": t - 1.5})
    assert sent == [] and a.assignments[TASK.task_id] == "robot_2"


def test_assignee_unavailable_past_release_is_reannounced():
    sent = []
    a = TaskAuction("robot_1", lambda d: sent.append(d), lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: 1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    # TASK_RELEASE_S is 20 s now (sih-57's mock saw living robots DEAD for
    # 10-16 s under load): the fault must persist past it to free the task.
    for t in range(10, 10 + int(TASK_RELEASE_S) + 3):  # faulted, broadcasting
        a.tick(float(t), {"robot_3"}, {"robot_2": float(t)})
    assert len(sent) == 1 and sent[0]["task_id"] == TASK.task_id


def test_holder_reasserts_ownership_when_its_task_is_reannounced():
    awards = []
    a = TaskAuction("robot_2", lambda d: None, lambda d: None, awards.append,
                    path_length_m=lambda p, q: 1.0)
    a.my_task = TASK
    a.on_announce(TASK, now=50.0)
    assert awards and awards[0]["task_id"] == "T-1"
    assert awards[0]["winner_id"] == "robot_2"
    assert TASK.task_id not in a.open_auctions and a.my_task is TASK

    peer = mk("robot_1")
    peer.on_announce(TASK, now=50.0)                 # peer opens the auction...
    peer.on_award(awards[0])                         # ...the re-assert cancels it
    assert TASK.task_id not in peer.open_auctions
    assert peer.assignments[TASK.task_id] == "robot_2" and peer.my_task is None


def _fleet_auctions():
    """Three auctions on a shared in-memory bus (announce + bid + award)."""
    fleet = {}
    log = []

    def bus(kind, sender):
        def send(d):
            log.append((kind, sender, d))
            for rid, a in fleet.items():
                if kind == "announce":
                    a.on_announce(Task(d["task_id"], (d["pickup_row"], d["pickup_col"]),
                                       (d["dropoff_row"], d["dropoff_col"]),
                                       d.get("priority", 0)), now=fleet_now[0])
                elif kind == "bid" and rid != sender:
                    a.on_bid(d)
                elif kind == "award":
                    a.on_award(d)
        return send

    fleet_now = [0.0]
    for rid in ("robot_1", "robot_2", "robot_3"):
        a = TaskAuction(rid, bus("announce", rid), bus("bid", rid), bus("award", rid),
                        path_length_m=lambda p, q: abs(p[0] - q[0]) + abs(p[1] - q[1]))
        a.nominal_speed = 0.4
        a.state_status = "IDLE"
        fleet[rid] = a
    return fleet, fleet_now, log


def test_unbid_task_not_stranded_when_lowest_id_missed_the_announce():
    """Mock run: announced during a DEAD storm, nobody could bid, and the
    only robot allowed to retry (lowest id) never heard it, so every other
    robot dropped it and it stayed PENDING forever."""
    fleet, now, log = _fleet_auctions()
    alive = {"robot_1", "robot_2", "robot_3"}
    busy = Task("T-busy", (1, 1), (2, 2))
    fleet["robot_1"].my_task = busy                  # busy, and misses the announce
    for rid in ("robot_2", "robot_3"):
        fleet[rid]._path_len = lambda p, q: -1.0     # unreachable right now
        fleet[rid].on_announce(TASK, now=0.0)
    for rid in ("robot_2", "robot_3"):               # peers recover a moment later
        fleet[rid]._path_len = lambda p, q: abs(p[0] - q[0]) + abs(p[1] - q[1])
    t = 0.0
    while t < 30.0 and TASK.task_id not in fleet["robot_2"].assignments:
        t = round(t + 0.1, 1)
        now[0] = t
        for a in fleet.values():
            a.tick(t, alive, {r: t for r in alive})
    holders = [rid for rid, a in fleet.items()
               if a.my_task is not None and a.my_task.task_id == TASK.task_id]
    assert len(holders) == 1, f"task stranded or double-held: {holders} at t={t}"
    assert t <= 2 * NO_BID_RETRY_S + 1.0


def test_unbid_retry_is_one_announce_per_period_not_a_storm():
    fleet, now, log = _fleet_auctions()
    alive = {"robot_1", "robot_2", "robot_3"}
    for a in fleet.values():
        a._path_len = lambda p, q: -1.0              # nobody can ever bid
    fleet["robot_1"]._announce(dict(task_id="T-1", pickup_row=5, pickup_col=5,
                                    dropoff_row=10, dropoff_col=10, priority=1))
    log.clear()
    t = 0.0
    while t < 60.0:
        t = round(t + 0.1, 1)
        now[0] = t
        for a in fleet.values():
            a.tick(t, alive, {r: t for r in alive})
    announces = [s for k, s, _ in log if k == "announce"]
    assert set(announces) == {"robot_1"}, "only the lowest live id retries"
    assert 60.0 / NO_BID_RETRY_S - 2 <= len(announces) <= 60.0 / NO_BID_RETRY_S + 1


def test_waiting_robot_bids_on_retry_once_it_becomes_free():
    fleet, now, log = _fleet_auctions()
    alive = {"robot_1", "robot_2", "robot_3"}
    for a in fleet.values():
        a.my_task = Task("T-busy-" + a.me, (1, 1), (2, 2))
    fleet["robot_1"]._announce(dict(task_id="T-1", pickup_row=5, pickup_col=5,
                                    dropoff_row=10, dropoff_col=10, priority=1))
    t = 0.0
    while t < 3.0:
        t = round(t + 0.1, 1)
        now[0] = t
        for a in fleet.values():
            a.tick(t, alive, {r: t for r in alive})
    fleet["robot_3"].my_task = None                  # robot_3 finishes its job
    while t < 20.0 and fleet["robot_3"].my_task is None:
        t = round(t + 0.1, 1)
        now[0] = t
        for a in fleet.values():
            a.tick(t, alive, {r: t for r in alive})
    assert fleet["robot_3"].my_task is not None
    assert fleet["robot_3"].my_task.task_id == "T-1"


def test_cancel_arriving_after_assignee_lost_reannounce_converges():
    """sih-57 field race: the holder's cancel was processed by a peer AFTER
    that peer had already re-announced the task (holder looked DEAD under
    load). The resurrection must die out: a robot that re-wins the task drops
    it the moment the delayed cancel arrives, and nobody announces it again."""
    fleet, now, log = _fleet_auctions()
    alive = {"robot_1", "robot_2", "robot_3"}
    # robot_2 won the task, then stalls silently (never ticks forward).
    for a in fleet.values():
        a.task_db[TASK.task_id] = TASK
        a.assignments[TASK.task_id] = "robot_2"
    fleet["robot_2"].my_task = TASK
    # robot_2 is silent long past release; robot_1 re-announces (lost assignee).
    t = 30.0
    now[0] = t
    fleet["robot_1"].tick(t, {"robot_1", "robot_3"}, {"robot_2": 0.0})
    assert any(k == "announce" for k, s, d in log), "re-announce expected"
    # The resurrection is won before any cancel is seen. (How the auction
    # resolves is covered elsewhere; this test pins the race outcome, so the
    # win is applied directly.)
    fleet["robot_3"].my_task = TASK
    for a in fleet.values():
        a.assignments[TASK.task_id] = "robot_3"
    # The delayed cancel (webapp aborted it before any pickup) now reaches all.
    for a in fleet.values():
        a.on_cancel(TASK.task_id, now=t)
    assert fleet["robot_3"].my_task is None, "late winner must drop on cancel"
    log.clear()
    for step in range(120):                    # well past every retry period
        t = round(t + 0.5, 1)
        now[0] = t
        for a in fleet.values():
            a.tick(t, alive, {r: t for r in alive})
    assert not any(k == "announce" for k, s, d in log), "never resurrected"
    assert all(TASK.task_id not in a.assignments for a in fleet.values())


def test_completion_is_seen_by_all_robots():
    a = mk("robot_2")
    a.assignments[TASK.task_id] = "robot_1"
    a.on_complete(TASK.task_id)
    assert TASK.task_id in a.completed
    assert TASK.task_id not in a.assignments


def test_charge_hold_robot_does_not_bid():
    """Low-battery robots charge at the dock instead of taking new work."""
    a = mk("robot_1", (4, 4), batt=90.0)
    a.charge_hold = True
    assert a.compute_bid(TASK) is None


def test_docked_robot_counts_as_idle_for_urgency():
    urgent = Task("T-9", pickup=(5, 5), dropoff=(10, 10), priority=3)
    idle = mk("robot_1", (4, 4))
    charging = mk("robot_1", (4, 4))
    charging.state_status = "CHARGING"
    assert charging.compute_bid(urgent) == idle.compute_bid(urgent)


# ====================================================== T1 / R4(3): no urgency
def test_returning_closer_robot_outbids_the_idle_one():
    """The measured R4(3) violation: a priority-3 task, an IDLE robot 4 m
    away and a robot still driving home (MOVING) 2 m away. The old
    IDLE/CHARGING urgency term handed it to the idle robot; with the term
    deleted, the closer robot wins regardless of status."""
    urgent = Task("T-9", pickup=(5, 5), dropoff=(10, 10), priority=3)
    idle_far = mk("robot_1", (5, 1))          # 4 m from the pickup
    returning_near = mk("robot_2", (5, 3))    # 2 m from the pickup
    returning_near.state_status = "MOVING"
    assert returning_near.compute_bid(urgent) > idle_far.compute_bid(urgent)


def test_mid_dwell_robot_bids_with_its_remaining_dwell():
    a = mk("robot_1", (4, 4))
    b = mk("robot_2", (4, 4))
    b.remaining_dwell_s = 2.0
    assert a.compute_bid(TASK) > b.compute_bid(TASK)


# =============================================== T2: joint batch assignment ==
def _two_robot_bus(costs_1, costs_2):
    """Two auctions on a synchronous bus with per-robot path-length tables."""
    fleet, log = {}, []

    def bus(kind, sender):
        def send(d):
            log.append((kind, sender, d))
            for rid, a in fleet.items():
                if kind == "announce":
                    a.on_announce(Task(d["task_id"],
                                       (d["pickup_row"], d["pickup_col"]),
                                       (d["dropoff_row"], d["dropoff_col"]),
                                       d.get("priority", 0)), now=now[0])
                elif kind == "bid" and rid != sender:
                    a.on_bid(d)
                elif kind == "award":
                    a.on_award(d)
        return send

    now = [0.0]
    for rid, costs in (("robot_1", costs_1), ("robot_2", costs_2)):
        a = TaskAuction(rid, bus("announce", rid), bus("bid", rid),
                        bus("award", rid),
                        path_length_m=lambda p, q, c=costs: c.get((p, q), 0.0))
        a.nominal_speed = 1.0           # cost in seconds == metres
        a.state_status = "IDLE"
        fleet[rid] = a
    return fleet, now


def test_batch_resolves_the_cross_assignment_cost_optimally():
    """The measured cross-assignment case (R4(1)): robot_1 is best at BOTH
    tasks. Per-task greedy gives robot_1 task A and leaves B to a 20 m
    detour by robot_2 (total 25 m); the min-sum batch gives r1->B, r2->A
    (total 13 m)."""
    cur1, cur2 = (0, 0), (9, 9)
    pA, pB, dA, dB = (0, 5), (0, 6), (1, 5), (1, 6)
    costs_1 = {(cur1, pA): 5.0, (cur1, pB): 6.0, (pA, dA): 0.0, (pB, dB): 0.0}
    costs_2 = {(cur2, pA): 7.0, (cur2, pB): 20.0, (pA, dA): 0.0, (pB, dB): 0.0}
    fleet, now = _two_robot_bus(costs_1, costs_2)
    fleet["robot_1"].current_cell = cur1
    fleet["robot_2"].current_cell = cur2
    tA = Task("T-A", pickup=pA, dropoff=dA, priority=1)
    tB = Task("T-B", pickup=pB, dropoff=dB, priority=1)
    for a in fleet.values():
        a.on_announce(tA, now=0.0)
    now[0] = 0.3
    for a in fleet.values():
        a.on_announce(tB, now=0.3)    # closes within BATCH_WINDOW_S of T-A
    alive = set(fleet)
    for t in (0.5, 0.7, 0.9, 1.1):
        now[0] = t
        for a in fleet.values():
            a.tick(t, alive, {r: t for r in alive})
    assert fleet["robot_1"].my_task is not None
    assert fleet["robot_1"].my_task.task_id == "T-B"
    assert fleet["robot_2"].my_task is not None
    assert fleet["robot_2"].my_task.task_id == "T-A"
    for a in fleet.values():
        assert a.assignments["T-A"] == "robot_2"
        assert a.assignments["T-B"] == "robot_1"


# ========================================= T4 / R4(4): peer-state collisions ==
def test_loaded_holder_wins_a_claim_collision_regardless_of_id():
    a = mk("robot_1")                    # LOWEST id, but not loaded
    a.my_task = TASK
    lost = a.on_peer_task("robot_2", TASK.task_id, peer_class=3, now=5.0,
                          my_class=2)
    assert lost and a.my_task is None
    assert a.assignments[TASK.task_id] == "robot_2"


def test_loaded_me_keeps_the_task_against_a_lower_id_claim():
    a = mk("robot_2")
    a.my_task = TASK
    lost = a.on_peer_task("robot_1", TASK.task_id, peer_class=2, now=5.0,
                          my_class=3)
    assert not lost and a.my_task is TASK
    assert a.assignments[TASK.task_id] == "robot_2"


def test_equal_class_collision_falls_back_to_lowest_id():
    a = mk("robot_2")
    a.my_task = TASK
    assert a.on_peer_task("robot_1", TASK.task_id, 2, 5.0, my_class=2)
    b = mk("robot_1")
    b.my_task = TASK
    assert not b.on_peer_task("robot_2", TASK.task_id, 2, 5.0, my_class=2)


def test_orphaned_task_is_reannounced_after_orphan_s():
    """Assignee alive but broadcasting a DIFFERENT task for 3 s."""
    from amr_fleet.core.tasks import ORPHAN_S
    sent = []
    a = TaskAuction("robot_1", lambda d: sent.append(d), lambda d: None,
                    lambda d: None, path_length_m=lambda p, q: -1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    alive = {"robot_2", "robot_3"}
    t = 10.0
    while t < 10.0 + ORPHAN_S + 1.0 and not sent:
        a.on_peer_task("robot_2", "T-other", 2, t, my_class=0)
        a.tick(t, alive, {r: t for r in alive})
        t = round(t + 0.1, 1)
    assert sent and sent[0]["task_id"] == TASK.task_id
    assert t - 10.0 >= ORPHAN_S - 1e-9, "must not fire before ORPHAN_S"


def test_loaded_task_is_not_reannounced_before_holder_gone():
    from amr_fleet.core.tasks import T_GONE_S
    sent = []
    a = TaskAuction("robot_1", lambda d: sent.append(d), lambda d: None,
                    lambda d: None, path_length_m=lambda p, q: -1.0)
    a.task_db[TASK.task_id] = TASK
    a.assignments[TASK.task_id] = "robot_2"
    a.on_peer_task("robot_2", TASK.task_id, 3, 9.0, my_class=0)  # LOADED
    # robot_2 goes silent for 25 s (> task_release_s, < T_GONE_S): keep it.
    for t in range(10, 35):
        a.tick(float(t), {"robot_3"}, {"robot_2": 9.0})
    assert sent == [], "a LOADED task must wait for GONE (30 s)"
    for t in range(35, 45):                 # past GONE
        a.tick(float(t), {"robot_3"}, {"robot_2": 9.0})
    assert len(sent) == 1
    assert T_GONE_S == 30.0


# ================================================= 200-trial lossy probe =====
def _lossy_probe(loss: float, trials: int, seed: int = 1):
    """3 distributed auctions; bids, awards and states each dropped with
    probability `loss` per receiver (announce is COORD_QOS RELIABLE +
    TRANSIENT_LOCAL, so it always arrives). Peer task claims flow through
    on_peer_task at 10 Hz, as fleet_agent_node.cb_peer_state does.
    Returns (unassigned_trials, worst duplicate-ownership seconds)."""
    import random
    rng = random.Random(seed)
    unassigned, worst_dup = 0, 0.0
    for trial in range(trials):
        fleet = {}
        inbox = {r: [] for r in ("robot_1", "robot_2", "robot_3")}

        def bus(kind, sender):
            def send(d, kind=kind, sender=sender):
                for rid in fleet:
                    if rid == sender:
                        if kind != "bid":       # DDS loopback
                            inbox[rid].append((kind, dict(d)))
                        continue
                    if kind != "announce" and rng.random() < loss:
                        continue
                    inbox[rid].append((kind, dict(d)))
            return send

        for rid in ("robot_1", "robot_2", "robot_3"):
            a = TaskAuction(rid, bus("announce", rid), bus("bid", rid),
                            bus("award", rid),
                            path_length_m=lambda p, q: abs(p[0] - q[0])
                            + abs(p[1] - q[1]))
            a.current_cell = (0, 0)
            a.nominal_speed = 0.4
            a.state_status = "IDLE"
            fleet[rid] = a
        task = Task(f"P-{trial}", pickup=(5, 5), dropoff=(9, 9), priority=1)
        for a in fleet.values():                 # reliable announce
            a.on_announce(task, now=0.0)
        alive = set(fleet)
        dup_run = 0.0
        t = 0.0
        while t < 15.0:
            t = round(t + 0.1, 1)
            for rid, a in fleet.items():
                for kind, d in inbox[rid]:
                    if kind == "announce":
                        a.on_announce(Task(d["task_id"],
                                           (d["pickup_row"], d["pickup_col"]),
                                           (d["dropoff_row"], d["dropoff_col"]),
                                           d.get("priority", 0)), now=t)
                    elif kind == "bid":
                        a.on_bid(d)
                    else:
                        a.on_award(d)
                inbox[rid] = []
            # 10 Hz peer state (lossy), as cb_peer_state feeds on_peer_task
            for rid, a in fleet.items():
                for peer, b in fleet.items():
                    if peer == rid or rng.random() < loss:
                        continue
                    tid = (b.my_task.task_id if b.my_task is not None else "")
                    lostc = a.on_peer_task(peer, tid, 2 if tid else 0, t,
                                           my_class=2 if a.my_task else 0)
                    assert not (lostc and a.my_task is not None)
                for a2 in (a,):
                    a2.tick(t, alive - {rid}, {r: t for r in alive})
            holders = [r for r, a in fleet.items()
                       if a.my_task is not None
                       and a.my_task.task_id == task.task_id]
            dup_run = dup_run + 0.1 if len(holders) >= 2 else 0.0
            worst_dup = max(worst_dup, dup_run)
        holders = [r for r, a in fleet.items() if a.my_task is not None]
        if not holders:
            unassigned += 1
    return unassigned, worst_dup


def test_probe_200_trials_at_20_and_40_percent_loss():
    """AC9: 0 trials end unassigned; duplicate ownership <= 1.0 s. (The
    pre-increment-3 code measured 86 unassigned and 136 duplicate trials.)"""
    for loss in (0.2, 0.4):
        unassigned, worst_dup = _lossy_probe(loss, 100, seed=int(loss * 10))
        assert unassigned == 0, f"{unassigned} unassigned at loss={loss}"
        assert worst_dup <= 1.0, f"dup {worst_dup:.1f}s at loss={loss}"


# ======================================================== R6: cancel suite ===
def test_cancel_refused_when_loaded():
    a = mk("robot_1")
    a.my_task = TASK
    assert a.on_cancel(TASK.task_id, 5.0, my_phase="DROPOFF") is False
    assert a.my_task is TASK
    assert TASK.task_id not in a.cancelled


def test_cancel_before_pickup_drops_my_task():
    a = mk("robot_1")
    a.my_task = TASK
    a.assignments[TASK.task_id] = "robot_1"
    assert a.on_cancel(TASK.task_id, 5.0, my_phase="PICKUP") is True
    assert a.my_task is None and TASK.task_id in a.cancelled
    assert TASK.task_id not in a.assignments


def test_cancel_never_resurrected_by_any_path():
    """The six resurrection paths: no-bid retry, dead-assignee re-announce,
    late announce, late award, holder re-assert, on_release — plus
    on_peer_task and the freed-robot pull."""
    sent_announce, sent_award = [], []
    a = TaskAuction("robot_1", sent_announce.append, lambda d: None,
                    sent_award.append,
                    path_length_m=lambda p, q: abs(p[0] - q[0]) + abs(p[1] - q[1]))
    a.current_cell = (0, 0)
    a.nominal_speed = 0.4
    a.state_status = "IDLE"
    # (1) no-bid retry: an unbid task in retry-wait, then cancelled
    a._path_len = lambda p, q: -1.0
    a.on_announce(TASK, now=0.0)
    a.tick(1.0, {"robot_2"}, {"robot_2": 1.0})     # now in no_bid_wait
    sent_announce.clear()           # the pre-cancel retry is legitimate
    assert a.on_cancel(TASK.task_id, 1.5) is True
    a._path_len = lambda p, q: 1.0
    for t in (2.0, 7.0, 12.0, 30.0):
        a.tick(t, {"robot_2"}, {"robot_2": t})
    assert sent_announce == [], "no-bid retry resurrected a cancelled task"
    # (2) dead-assignee re-announce
    t2 = Task("T-2", (1, 1), (2, 2))
    a.task_db[t2.task_id] = t2
    a.assignments[t2.task_id] = "robot_2"
    a.on_cancel(t2.task_id, 31.0)
    for t in (60.0, 90.0):                          # robot_2 long gone
        a.tick(t, set(), {"robot_2": 0.0})
    assert sent_announce == []
    # (3) late announce
    a.on_announce(t2, now=91.0)
    assert t2.task_id not in a.open_auctions
    # (4) late award
    a.on_award(dict(task_id=t2.task_id, winner_id="robot_1"))
    assert t2.task_id not in a.assignments and a.my_task is None
    # (5) holder re-assert: a cancelled task is no longer mine, so a later
    # announce must trigger neither an award nor a new auction
    sent_award.clear()
    a.on_announce(t2, now=92.0)
    assert sent_award == [] and t2.task_id not in a.open_auctions
    # (6) on_release
    assert a.on_release(t2, "robot_2", now=93.0) is False
    assert t2.task_id not in a.open_auctions
    # on_peer_task ignores it
    assert a.on_peer_task("robot_2", t2.task_id, 2, 94.0, my_class=0) is False
    assert t2.task_id not in a.assignments
    # the freed-robot pull skips it
    a.note_freed(95.0)
    a.tick(96.0, {"robot_2"}, {"robot_2": 96.0})
    assert sent_announce == []


def test_freed_robot_pulls_the_most_urgent_open_task():
    sent = []
    a = TaskAuction("robot_1", sent.append, lambda d: None, lambda d: None,
                    path_length_m=lambda p, q: 1.0)
    low = Task("T-low", (1, 1), (2, 2), priority=0, created_at=100.0,
               deadline=325.0)
    old_urgent = Task("T-old", (3, 3), (4, 4), priority=2, created_at=10.0,
                      deadline=160.0)                 # long overdue
    a.task_db[low.task_id] = low
    a.task_db[old_urgent.task_id] = old_urgent
    a.note_freed(300.0)
    a.tick(300.0, {"robot_1"}, {})
    assert sent and sent[0]["task_id"] == "T-old"


def test_task_cancel_conversion_round_trip():
    """conversions TaskCancel helpers, duck-typed (no generated msg needed)."""
    import sys
    import types
    if "amr_description.msg" not in sys.modules:
        fake = types.ModuleType("amr_description")
        fake_msg = types.ModuleType("amr_description.msg")
        for name in ("Intent", "MapUpdate", "MotionPermit", "RobotState",
                     "Task", "Conflict", "TaskCancel"):
            setattr(fake_msg, name, type(name, (), {}))
        fake.msg = fake_msg
        sys.modules["amr_description"] = fake
        sys.modules["amr_description.msg"] = fake_msg
    from amr_fleet.nodes import conversions as cv

    class StubCancel:
        task_id = requester_id = reason = ""
    m = cv.task_cancel_to_ros("T-1", "webapp", "user abort", msg=StubCancel())
    d = cv.task_cancel_from_ros(m)
    assert d == dict(task_id="T-1", requester_id="webapp", reason="user abort")
