"""Zone mutual exclusion, including under message loss."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.zone import ZoneArbiter, ZoneTimeout


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


def test_higher_priority_wins_on_concurrent_requests():
    """Priority arbitrates CONCURRENT contention - both request before either
    hears the other."""
    bus = Bus(concurrent=True)
    a1 = bus.register("robot_1")
    a2 = bus.register("robot_2")
    a1.request("Z", 0.30, 0.0, 5.0)      # lower priority
    a2.request("Z", 0.80, 0.0, 5.0)      # higher priority
    bus.pump()
    e1 = a1.may_enter("Z", {"robot_2"})
    e2 = a2.may_enter("Z", {"robot_1"})
    assert e2 and not e1, f"robot_2 should win: e1={e1} e2={e2}"


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

    grid = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1] / "config" / "warehouse_grid.yaml"))
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

    grid = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1] / "config" / "warehouse_grid.yaml"))
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.path = [(74, 49), (74, 50), (74, 51), (74, 52), (74, 53), (74, 54), (74, 55), (74, 56), (74, 57), (74, 58), (74, 59), (74, 60), (74, 61), (74, 62), (74, 63), (74, 64), (74, 65), (74, 66), (74, 67), (74, 68), (74, 69), (74, 70)]
    c.intent.zones = ["inter_X1"]
    c.state.pose = Pose2D(*grid.cell_to_world((74, 70)), 0.0)
    c.arbiter.state["inter_X1"] = "HELD"

    c._maybe_release_passed_zones()
    assert c.arbiter.state["inter_X1"] == "FREE"
