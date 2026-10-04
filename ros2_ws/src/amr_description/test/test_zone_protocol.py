"""Zone mutual exclusion, including under message loss."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.zone import ZoneArbiter, ZoneTimeout

# The two coordinator tests at the bottom name the pre-increment-2 zone
# inter_X1, so they load the frozen legacy grid (wiki/Traffic-Lanes.md).
LEGACY_GRID = (pathlib.Path(__file__).resolve().parent / "fixtures"
               / "warehouse_grid_legacy.yaml")


class Bus:
    """In-process broadcast bus.

    `concurrent=True` buffers requests and delivers them only when pump() is
    called, which models two robots requesting at the same instant. That is
    the case priority arbitration is FOR. With immediate delivery the first
    requester simply acquires the lock before the second one asks, which is
    also correct behaviour but tests a different path.
    """

    def __init__(self, drop=None, concurrent=False):
        self.nodes = {}
        self.drop = drop or (lambda kind, msg: False)
        self.sent = []
        self.concurrent = concurrent
        self.pending = []

    def register(self, rid):
        a = ZoneArbiter(rid,
                        lambda d, r=rid: self._req(r, d),
                        lambda d, r=rid: self._grant(r, d))
        self.nodes[rid] = a
        return a

    def _relevant(self, rid):
        return {r for r in self.nodes if r != rid}

    def _req(self, sender, d):
        self.sent.append(("REQ", d))
        if self.drop("REQ", d):
            return
        if self.concurrent:
            self.pending.append(("REQ", sender, d))
            return
        self._deliver_req(sender, d)

    def _deliver_req(self, sender, d):
        for rid, a in self.nodes.items():
            if rid != sender:
                a.on_request(d, i_need_zone=True,
                             relevant_peers=self._relevant(rid))

    def _grant(self, sender, d):
        self.sent.append(("GRANT", d))
        if self.drop("GRANT", d):
            return
        if self.concurrent:
            self.pending.append(("GRANT", sender, d))
            return
        self._deliver_grant(d)

    def _deliver_grant(self, d):
        for rid, a in self.nodes.items():
            if rid == d["requester_id"]:
                a.on_grant(d)

    def pump(self, rounds=3):
        for _ in range(rounds):
            batch, self.pending = self.pending, []
            for kind, sender, d in batch:
                if kind == "REQ":
                    self._deliver_req(sender, d)
                else:
                    self._deliver_grant(d)


def test_uncontested_zone_granted_immediately():
    bus = Bus()
    a = bus.register("robot_1")
    bus.register("robot_2")
    a.request("Z", 0.5, 0.0, 5.0)
    assert a.may_enter("Z", {"robot_2"})


def test_fifo_ignores_priority_on_concurrent_requests():
    """Increment 3: the key is (lamport, robot_id) PURE FIFO. A higher
    priority_score never orders or preempts a zone (the sih-57 stale-score
    reorder race); on a concurrent lamport tie the lower robot_id wins,
    however the scores compare."""
    bus = Bus(concurrent=True)
    a1 = bus.register("robot_1")
    a2 = bus.register("robot_2")
    a1.request("Z", 0.30, 0.0, 5.0)      # lower priority score
    a2.request("Z", 0.80, 0.0, 5.0)      # higher priority score
    bus.pump()
    e1 = a1.may_enter("Z", {"robot_2"})
    e2 = a2.may_enter("Z", {"robot_1"})
    assert e1 and not e2, f"FIFO/id order must win: e1={e1} e2={e2}"


def test_earlier_lamport_wins_whatever_the_scores_say():
    """A later request with a huge score queues behind an earlier one."""
    bus = Bus()
    a1 = bus.register("robot_1")
    a2 = bus.register("robot_2")
    a2.lamport = 50                       # robot_2 is causally later
    a1.request("Z", 0.0, 0.0, 5.0)        # delivered immediately
    a2.request("Z", 99999.0, 0.0, 5.0)
    assert a1.may_enter("Z", {"robot_2"})
    assert not a2.may_enter("Z", {"robot_1"})


def test_first_requester_keeps_lock_against_later_higher_priority():
    """A granted lock is NOT preempted by a later higher-priority request.

    Preemption would allow livelock: a high-priority robot could repeatedly
    snatch a zone from one that had already acquired it and started moving.
    """
    bus = Bus()
    a1 = bus.register("robot_1")
    a2 = bus.register("robot_2")
    a1.request("Z", 0.30, 0.0, 5.0)      # asks first, uncontested -> granted
    a2.request("Z", 0.95, 0.0, 5.0)      # much higher, but too late
    assert a1.may_enter("Z", {"robot_2"})
    assert not a2.may_enter("Z", {"robot_1"})


def test_mutual_exclusion_never_violated():
    """THE SAFETY INVARIANT: at most one holder, under EVERY ordering.

    This test found a real bug. Originally, a robot that granted the zone
    before wanting it could later outrank the holder and both would enter.
    """
    for concurrent in (False, True):
        for s1, s2 in [(0.5, 0.5), (0.9, 0.1), (0.1, 0.9), (0.7, 0.7),
                       (0.0, 1.0), (1.0, 0.0)]:
            bus = Bus(concurrent=concurrent)
            a1 = bus.register("robot_1")
            a2 = bus.register("robot_2")
            a1.request("Z", s1, 0.0, 5.0)
            a2.request("Z", s2, 0.0, 5.0)
            if concurrent:
                bus.pump()
            holders = sum(1 for a, peer in ((a1, "robot_2"), (a2, "robot_1"))
                          if a.may_enter("Z", {peer}))
            assert holders <= 1, (
                f"{holders} holders: scores {s1}/{s2} concurrent={concurrent}")


def test_mutual_exclusion_with_three_robots():
    """Three-way contention: still at most one holder."""
    for concurrent in (False, True):
        bus = Bus(concurrent=concurrent)
        arbs = {r: bus.register(r) for r in ("robot_1", "robot_2", "robot_3")}
        for i, (rid, a) in enumerate(arbs.items()):
            a.request("Z", 0.2 * (i + 1), 0.0, 5.0)
        if concurrent:
            bus.pump()
        holders = sum(1 for rid, a in arbs.items()
                      if a.may_enter("Z", {r for r in arbs if r != rid}))
        assert holders <= 1, f"{holders} holders with 3 robots"


def test_exact_tie_broken_by_id():
    bus = Bus(concurrent=True)
    a1 = bus.register("robot_1")
    a2 = bus.register("robot_2")
    a1.request("Z", 0.5, 0.0, 5.0)
    a2.request("Z", 0.5, 0.0, 5.0)
    bus.pump()
    assert a1.may_enter("Z", {"robot_2"})      # lower id wins
    assert not a2.may_enter("Z", {"robot_1"})


def test_release_grants_deferred_peers():
    """A deferred robot must be WOKEN by the release, not left polling."""
    bus = Bus(concurrent=True)
    a1 = bus.register("robot_1")
    a2 = bus.register("robot_2")
    a1.request("Z", 0.9, 0.0, 5.0)
    a2.request("Z", 0.2, 0.0, 5.0)
    bus.pump()
    assert a1.may_enter("Z", {"robot_2"})
    assert not a2.may_enter("Z", {"robot_1"})
    a1.release("Z")
    bus.pump()
    assert a2.may_enter("Z", {"robot_1"}), "release must grant deferred peers"


def test_dropped_grant_times_out_rather_than_hanging():
    """A lost grant must never wedge the fleet forever."""
    import time
    bus = Bus(drop=lambda kind, m: kind == "GRANT")
    a1 = bus.register("robot_1")
    bus.register("robot_2")
    a1.request("Z", 0.5, 0.0, 5.0)
    a1.req_time["Z"] = 0.0       # simulate 11 s elapsed
    try:
        a1.may_enter("Z", {"robot_2"}, now=11.0)
        assert False, "should have raised ZoneTimeout"
    except ZoneTimeout as e:
        assert e.zone_id == "Z"


def test_dead_peer_grant_not_required():
    """If robot_2 dies, robot_1 must still be able to enter."""
    bus = Bus()
    a1 = bus.register("robot_1")
    bus.register("robot_2")
    a1.request("Z", 0.5, 0.0, 5.0)
    a1.grants["Z"] = set()
    assert a1.may_enter("Z", set())      # empty relevant set -> proceed


def test_lamport_clock_advances_on_receipt():
    bus = Bus()
    a1 = bus.register("robot_1")
    a2 = bus.register("robot_2")
    a2.lamport = 100
    a2.request("Z", 0.5, 0.0, 5.0)
    assert a1.lamport > 100, "clock must advance past an observed timestamp"


def test_deferred_grant_uses_requesters_original_request_id():
    sent = []
    a = ZoneArbiter("robot_1", lambda d: None, lambda d: sent.append(d))
    a.state["Z"] = "HELD"
    a.req_id["Z"] = 99
    a.on_request({"robot_id": "robot_2", "zone_id": "Z", "lamport_ts": 1,
                  "priority_score": 0.2, "req_id": 7}, i_need_zone=True)
    a.release("Z")
    assert sent[-1]["requester_id"] == "robot_2"
    assert sent[-1]["req_id"] == 7


def test_zone_distance_is_measured_from_current_position_not_path_start():
    from amr_fleet.core.coordinator import FleetCoordinator
    from amr_fleet.core.gridmap import GridMap
    from amr_fleet.core.models import Pose2D

    grid = GridMap.from_yaml(str(LEGACY_GRID))
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*grid.cell_to_world((74, 45)), 0.0)
    c.path = [(74, 45), (74, 46), (74, 47), (74, 48), (74, 49),
              (74, 50), (74, 51), (74, 52), (74, 53), (74, 54)]
    c.intent.zones = ["inter_X1"]

    d1 = c._distance_to_zone_m("inter_X1")
    c.state.pose = Pose2D(*grid.cell_to_world((74, 49)), 0.0)
    d2 = c._distance_to_zone_m("inter_X1")
    assert d2 < d1


def test_passed_zone_is_released_even_while_still_in_intent():
    from amr_fleet.core.coordinator import FleetCoordinator
    from amr_fleet.core.gridmap import GridMap
    from amr_fleet.core.models import Pose2D

    grid = GridMap.from_yaml(str(LEGACY_GRID))
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.path = [(74, 49), (74, 50), (74, 51), (74, 52), (74, 53), (74, 54), (74, 55), (74, 56), (74, 57), (74, 58), (74, 59), (74, 60), (74, 61), (74, 62), (74, 63), (74, 64), (74, 65), (74, 66), (74, 67), (74, 68), (74, 69), (74, 70)]
    c.intent.zones = ["inter_X1"]
    c.state.pose = Pose2D(*grid.cell_to_world((74, 70)), 0.0)
    c.arbiter.state["inter_X1"] = "HELD"

    c._maybe_release_passed_zones()
    assert c.arbiter.state["inter_X1"] == "FREE"


# --------------------------------------- increment 3: FIFO + lamport grants
def test_reorder_race_new_request_overtaking_flushed_grant_no_double_hold():
    """The measured 0.15 s-jitter race: peer P flushes a grant to me (its
    timeout flushed its deferred queue), then re-requests with a fresh
    lamport - and the two messages arrive in EITHER order. Neither order may
    leave both of us holding."""
    relevant = {"robot_1", "robot_3"}      # ownership incomplete: no owner
    for grant_first in (False, True):
        me = ZoneArbiter("robot_2", lambda d: None, lambda d: None)
        me.request("SPINE", 0.5, 0.0, 1.0, ranked=True)
        grant = dict(granter_id="robot_1", requester_id="robot_2",
                     zone_id="SPINE", req_id=me.req_id["SPINE"],
                     granted=True, lamport_ts=7)
        newreq = dict(robot_id="robot_1", zone_id="SPINE", lamport_ts=9,
                      priority_score=0.1, req_id=123)
        if grant_first:
            me.on_grant(grant)
            me.on_request(newreq, i_need_zone=True, relevant_peers=relevant)
        else:
            me.on_request(newreq, i_need_zone=True, relevant_peers=relevant)
            me.on_grant(grant)
        # The flushed grant is overtaken/stale either way: I must NOT count
        # robot_1 as a granter.
        assert "robot_1" not in me.grants.get("SPINE", set()), grant_first
        assert not me.may_enter("SPINE", relevant, now=0.1)
        # Liveness: robot_1's NEWER grant (after it defers to my resend, or
        # releases) is valid, plus robot_3's.
        me.on_grant(dict(grant, lamport_ts=12))
        me.on_grant(dict(grant, granter_id="robot_3", lamport_ts=13))
        assert me.may_enter("SPINE", relevant, now=0.2)


def test_effective_owner_keeps_a_deliberately_flushed_grant():
    """When I already hold EVERY relevant grant I am the effective owner:
    the granter's later re-request queues behind me (FIFO: its new lamport
    is causally after its grant) and must NOT void my ownership."""
    me = ZoneArbiter("robot_2", lambda d: None, lambda d: None)
    me.request("SPINE", 0.5, 0.0, 1.0, ranked=True)
    me.on_grant(dict(granter_id="robot_1", requester_id="robot_2",
                     zone_id="SPINE", req_id=me.req_id["SPINE"],
                     granted=True, lamport_ts=7))
    me.on_request(dict(robot_id="robot_1", zone_id="SPINE", lamport_ts=9,
                       priority_score=9999.0, req_id=123),
                  i_need_zone=True, relevant_peers={"robot_1"})
    assert me.may_enter("SPINE", {"robot_1"}, now=0.1)


def test_ranked_request_resends_same_stamp_and_never_times_out():
    """A ranked waiter keeps its FIFO queue position: every resend carries
    the SAME req_id and lamport, there is no ZoneTimeout, and resends keep
    coming long past the legacy T_ZONE_S."""
    sent = []
    a = ZoneArbiter("robot_1", sent.append, lambda d: None)
    a.request("SPINE", 0.5, 0.0, 5.0, now=0.0, ranked=True)
    for t in (1.1, 2.2, 3.3, 11.0, 25.0, 60.0):
        assert not a.may_enter("SPINE", {"robot_2"}, now=t)
    reqs = [d for d in sent if d["zone_id"] == "SPINE"]
    assert len(reqs) >= 6, "must keep resending"
    assert len({d["req_id"] for d in reqs}) == 1, "same req_id forever"
    assert len({d["lamport_ts"] for d in reqs}) == 1, "same lamport forever"
    assert a.stats["timeouts"] == 0


def test_unranked_zone_keeps_the_legacy_timeout():
    a = ZoneArbiter("robot_1", lambda d: None, lambda d: None)
    a.request("legacy_Z", 0.5, 0.0, 5.0, now=0.0)
    try:
        a.may_enter("legacy_Z", {"robot_2"}, now=11.0)
        assert False, "unranked zones must still raise ZoneTimeout"
    except ZoneTimeout:
        pass


def test_silent_peer_still_blocks_a_ranked_zone():
    """A required granter that has gone quiet is NOT dropped (zone.py
    drop_peer is for boot change / FAULT / GONE only): with it in the
    relevant set, may_enter stays False for as long as it stays silent."""
    a = ZoneArbiter("robot_1", lambda d: None, lambda d: None)
    a.request("SPINE", 0.5, 0.0, 5.0, now=0.0, ranked=True)
    for t in (5.0, 15.0, 29.0):
        assert not a.may_enter("SPINE", {"robot_silent"}, now=t)
    # Only the caller's GONE decision (drop from the relevant set) frees it.
    assert a.may_enter("SPINE", set(), now=30.5)
