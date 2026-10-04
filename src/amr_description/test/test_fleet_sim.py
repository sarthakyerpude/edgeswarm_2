"""Smoke tests for the offline fleet simulator (test/fleet_sim.py).

These only check that the simulator runs and reports well-formed metrics, and
that one robot alone completes a task. The traffic-rule acceptance thresholds
(Plan-P2P-Coordination increment 2) are set by the integrator once the
traffic layer lands; they are present below as non-strict xfails.

The fault / noise knobs (heartbeat, link and node faults; localization
bias and jitter) are smoke-tested for effect, not for fleet outcomes.

Set FLEET_SIM_LONG=1 to also run the 8-minute 3-robot stream and the
sih-57 stress case (~20 s wall each on an idle host).
"""
import math
import os
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fleet_sim as fs  # noqa: E402

NUMERIC = ["deliveries", "pickups", "contacts", "contact_ticks",
           "deadlocks_detected", "deadlocks_resolved", "reroutes",
           "retreats_started", "retreats_completed", "node_yield_retreats",
           "zone_timeouts", "hold_lease_releases", "replan_failed",
           "refuges_in_spine", "refuges_in_spine_gap",
           "spine_head_on_encounters",
           "near_misses", "mutual_yield_events", "lane_passes",
           "makeway_started", "makeway_failed"]
NUMERIC_F = ["duplicate_owner_s", "duplicate_owner_max_s",
             "three_stuck_s", "three_stuck_max_s"]


def _check_well_formed(m, n_robots, duration):
    assert m["n_robots"] == n_robots
    assert m["duration_s"] == duration
    for k in NUMERIC:
        assert isinstance(m[k], int) and m[k] >= 0, k
    for k in NUMERIC_F:
        assert isinstance(m[k], float) and m[k] >= 0.0, k
    cyc = m["task_cycle_s"]
    assert cyc["n"] == len(cyc["per_task"])
    if cyc["n"]:
        assert cyc["mean_s"] <= cyc["max_s"]
    if n_robots > 1:
        assert m["min_rect_gap_m"] is not None
        assert m["min_rect_gap_m"] <= m["min_separation_m"]
    assert m["retreats_completed"] <= m["retreats_started"]
    assert m["deadlocks_resolved"] <= m["deadlocks_detected"]
    assert m["deliveries"] <= m["pickups"]
    ids = {f"robot_{i + 1}" for i in range(n_robots)}
    for k in ("max_wait_s", "max_stall_s", "max_go_stall_s", "distance_m",
              "permit_actions",
              "deliveries_per_robot", "rack_scrape_s"):
        assert set(m[k]) == ids, k
    for rid in ids:
        assert 0.0 <= m["max_wait_s"][rid] <= duration + 1e-6
        assert m["distance_m"][rid] >= 0.0
        ticks = sum(m["permit_actions"][rid].values())
        assert ticks >= round(duration / fs.DT)
    for table in (m["zone_double_occupancy"], m["plan_zone_double_occupancy"]):
        for v in table.values():
            assert v["events"] <= v["ticks"]
    assert set(m["plan_zone_double_occupancy"]) == set(fs.PLAN_ZONES)
    if n_robots > 1:
        assert m["min_separation_m"] is not None and m["min_separation_m"] > 0.0
        assert m["intent_visibility"] is None or 0.0 <= m["intent_visibility"] <= 1.0
    for r in m["refuges"]:
        assert len(r["cell"]) == 2 and isinstance(r["in_spine_gap"], bool)
    summary = fs.summary(m)
    assert "events" not in summary and "deliveries" in summary


def test_single_robot_completes_pickup_and_dropoff():
    """Alone, spawn (-3,-4) -> L1 -> D2 is ~22 m: well under 150 s."""
    m = fs.run(fs.single_robot(), 150.0)
    _check_well_formed(m, 1, 150.0)
    kinds = [(e["kind"], e["t"]) for e in m["events"]]
    assert [k for k, _ in kinds] == ["award", "pickup", "delivery"], kinds
    t_delivery = kinds[2][1]
    # per-task cycle metric: award -> delivery
    assert m["task_cycle_s"]["n"] == 1
    assert abs(m["task_cycle_s"]["mean_s"] - t_delivery) < 1.0
    assert t_delivery < 120.0, kinds
    assert m["deliveries"] == 1 and m["contacts"] == 0
    assert m["deadlocks_detected"] == 0
    assert m["rack_scrape_s"]["robot_1"] == 0.0
    # The follower never sits permitted-but-motionless (spin-in-place bug).
    assert m["max_go_stall_s"]["robot_1"] < 5.0


def test_single_robot_on_legacy_grid_completes():
    m = fs.run(fs.single_robot(grid_yaml=str(fs.LEGACY_YAML)), 150.0)
    assert m["deliveries"] == 1


def test_three_robot_stream_runs_and_metrics_are_well_formed():
    sc = fs.random_stream(seed=3, interval_s=5.0)
    m = fs.run(sc, 40.0)
    _check_well_formed(m, 3, 40.0)
    assert m["tasks_announced"] >= 3
    assert m["bus_sent"]["state"] > 0 and m["bus_sent"]["intent"] > 0


def test_head_on_scenario_runs_and_robots_move():
    m = fs.run(fs.head_on(), 30.0)
    _check_well_formed(m, 2, 30.0)
    assert m["distance_m"]["robot_1"] > 2.0
    assert m["distance_m"]["robot_2"] > 2.0
    assert any(e["kind"] == "pickup" and e["robot"] == "robot_2"
               for e in m["events"]), "B loads at L1 first"


def test_message_loss_latency_and_skew_knobs():
    sc = fs.random_stream(seed=1, interval_s=5.0, state_loss=0.3,
                          intent_loss=0.3, latency_s=0.15, jitter_s=0.05,
                          clock_offsets={"robot_2": 0.8}, loc_noise_m=0.05)
    m = fs.run(sc, 20.0)
    _check_well_formed(m, 3, 20.0)
    assert m["bus_dropped"]["state"] > 0 and m["bus_dropped"]["intent"] > 0
    assert m["bus_dropped"]["zone_request"] == 0


def test_heartbeat_fault_makes_the_peer_dead_but_not_the_sender_blind():
    """robot_2's state is unheard for 9 s: after peer_dead_s (5 s) both
    peers mark it DEAD; robot_2 itself still hears everyone."""
    sc = fs.random_stream(seed=1, interval_s=5.0,
                          faults=[fs.Fault("robot_2", 2.0, 11.0, "heartbeat")])
    m = fs.run(sc, 13.0)
    _check_well_formed(m, 3, 13.0)
    assert m["bus_fault_dropped"]["state"] > 0
    assert m["bus_fault_dropped"]["intent"] == 0
    for obs in ("robot_1", "robot_3"):
        assert 2.5 <= m["peer_dead_s"].get(f"{obs}<-robot_2", 0.0) <= 4.5
    assert not any(k.startswith("robot_2<-") for k in m["peer_dead_s"])
    assert m["peer_failures_detected"] >= 2


def test_link_fault_replays_reliable_traffic_and_node_stall_stops_robot():
    link = [fs.Fault("robot_1", 2.0, 6.0, "link")]
    m = fs.run(fs.random_stream(seed=1, interval_s=5.0, faults=link), 9.0)
    assert m["bus_fault_dropped"]["state"] > 0
    assert m["bus_replayed"]["intent"] > 0          # KEEP_LAST(10) replay
    m2 = fs.run(fs.random_stream(seed=1, interval_s=5.0, faults=link,
                                 link_replay=False), 9.0)
    assert sum(m2["bus_replayed"].values()) == 0
    assert m2["bus_fault_dropped"]["intent"] > 0
    # Node stall at full speed: the gate stops it 0.5 s after the last permit.
    sim = fs.FleetSim(fs.single_robot(
        faults=[fs.Fault("robot_1", 5.0, 10.0, "node")]))
    sim.run(5.0)            # mid-straight at the 0.6 m/s cruise (turns at ~7.5 s)
    a = sim.agents[0]
    assert a.v > 0.2, "cruising before the stall"
    x0, y0 = a.x, a.y
    sim.run(5.0)
    # gate timeout 0.5 s at the 0.6 m/s cruise + braking (2.0 m/s^2) ~ 0.39 m
    assert math.hypot(a.x - x0, a.y - y0) < 0.5
    assert abs(a.v) < 1e-6
    m3 = sim.run(4.0)
    assert m3["node_stalled_s"]["robot_1"] == 5.0
    assert m3["distance_m"]["robot_1"] > math.hypot(a.x - x0, a.y - y0)
    assert abs(a.v) > 0.05, "resumes after the stall"


def test_localization_noise_separates_belief_from_truth():
    m = fs.run(fs.single_robot(loc_noise_m=0.1, loc_jitter_m=0.03,
                               loc_heading_noise_rad=0.02, seed=4), 60.0)
    assert 0.03 < m["loc_error_mean_m"] < 0.3
    assert m["loc_error_max_m"] >= m["loc_error_mean_m"]
    assert m["pickups"] == 1


def test_runs_are_deterministic_for_a_seed():
    a = fs.run(fs.random_stream(seed=5, interval_s=5.0), 25.0)
    b = fs.run(fs.random_stream(seed=5, interval_s=5.0), 25.0)
    assert a["distance_m"] == b["distance_m"]
    assert a["events"] == b["events"]


# ------------------------------------------------ distributed auction (R4) --
def test_distributed_auction_assigns_over_the_bus():
    sc = fs.random_stream(seed=3, interval_s=5.0)
    assert sc.distributed_auction
    m = fs.run(sc, 40.0)
    awards = [e for e in m["events"] if e["kind"] == "award"]
    assert awards, "no award ever happened over the bus"
    assert m["duplicate_owner_s"] == 0.0
    assert m["bus_sent"]["task_announce"] > 0
    assert m["bus_sent"]["task_bid"] > 0
    assert m["bus_sent"]["task_award"] > 0


def test_three_at_once_assignment_is_cost_optimal():
    """R4(1)/AC7: the batch assignment is MIN-SUM over the true route
    lengths, and the west robot is never sent to the east pickup."""
    from itertools import permutations
    sim = fs.FleetSim(fs.three_at_once())
    m = sim.run(30.0)
    owner = {e["task"]: e["robot"] for e in m["events"] if e["kind"] == "award"}
    assert set(owner) == {"A-1", "A-2", "A-3"}, owner
    assert owner["A-2"] != "robot_1", "west robot crossed to the east pickup"
    assert m["duplicate_owner_s"] == 0.0
    # brute-force the min-sum assignment over real route lengths
    plen = sim.agents[0].coord.path_length_m_between
    spawn = {a.id: a.grid.world_to_cell(a.spec.x, a.spec.y)
             for a in sim.agents}
    tasks = {t.task_id: t for _, t in sim.sc.announced}
    rids = sorted(spawn)
    tids = sorted(tasks)

    def total(perm):
        return sum(plen(spawn[r], tasks[t].pickup)
                   + plen(tasks[t].pickup, tasks[t].dropoff)
                   for r, t in zip(perm, tids))
    best = min(permutations(rids), key=total)
    chosen = tuple(owner[t] for t in tids)
    assert abs(total(chosen) - total(best)) < 1e-6, (chosen, best)


def test_converge_and_three_way_scenarios_run():
    m = fs.run(fs.converge(), 15.0)
    _check_well_formed(m, 3, 15.0)
    assert sum(1 for e in m["events"] if e["kind"] == "award") == 3
    names = set()
    for v in range(8):
        sc = fs.three_way(variant=v)
        names.add(sc.name)
        picks = [spec.tasks[0][1].pickup for spec in sc.robots]
        assert sorted(picks) == sorted([fs.L1, fs.L2, fs.L3])
    assert len(names) == 8


# -------------------------------------------------------- R6 cancel in sim --
def test_cancel_before_pickup_aborts_and_frees_the_pickup():
    sim = fs.FleetSim(fs.random_stream(seed=2, interval_s=5.0))
    sim.run(7.0)                       # T-0001 announced at 5.0, awarded 5.6
    assert any(e["kind"] == "award" and e["task"] == "T-0001"
               for e in sim.events)
    sim.cancel_task("T-0001")
    m = sim.run(10.0)
    assert any(e["kind"] == "cancel_aborted" for e in m["events"])
    assert not any(e["kind"] == "delivery" and e["task"] == "T-0001"
                   for e in m["events"])
    for a in sim.agents:
        assert a.coord.state.task_id != "T-0001"
        assert "T-0001" in a.auction.cancelled
        assert "T-0001" not in a.auction.completed, "abort counted as complete"
    assert "T-0001" not in sim.live_tasks, "pickup dedup entry not freed"


def test_cancel_after_pickup_is_refused_and_task_delivers():
    sim = fs.FleetSim(fs.single_robot())
    a = sim.agents[0]
    while sim.t < 90.0 and a._task_phase != "DROPOFF":
        sim.step()                                   # run until loaded
    assert a._task_phase == "DROPOFF", "precondition: loaded"
    sim.cancel_task("S-1")
    m = sim.run(60.0)
    assert any(e["kind"] == "cancel_refused" for e in m["events"])
    assert m["deliveries"] == 1, "a loaded task must still deliver"


def test_rect_gap_ground_truth_metric():
    # side-by-side, 0.8 m apart, both heading north: wheel faces 0.205 m out
    g = fs.rect_gap((0.0, 0.0, math.pi / 2), (0.8, 0.0, math.pi / 2))
    assert abs(g - (0.8 - 2 * fs.HB_HALF_W)) < 1e-6
    # nose-to-nose 0.5 m apart
    g2 = fs.rect_gap((0.0, 0.0, 0.0), (0.5, 0.0, math.pi))
    assert abs(g2 - (0.5 - 2 * fs.HB_HALF_L)) < 1e-6
    # overlap
    assert fs.rect_gap((0.0, 0.0, 0.0), (0.3, 0.0, 0.0)) == 0.0


# ------------------------------------------------- acceptance placeholders --
# Increment-2 ACCEPT criteria (wiki/Plan-P2P-Coordination.md), simulated.
# Non-strict xfail until the integrator sets the thresholds.

@pytest.mark.xfail(strict=False,
                   reason="increment-2 acceptance; thresholds owned by integrator")
def test_accept_head_on_completes_without_long_stops():
    m = fs.run(fs.head_on(), 240.0)
    done = {e["robot"] for e in m["events"] if e["kind"] == "delivery"}
    assert done == {"robot_1", "robot_2"}
    assert m["contacts"] == 0 and m["min_separation_m"] >= 0.50
    assert max(m["max_wait_s"].values()) <= 20.0


@pytest.mark.skipif(not os.environ.get("FLEET_SIM_LONG"),
                    reason="8-minute stream; set FLEET_SIM_LONG=1")
@pytest.mark.xfail(strict=False,
                   reason="AC1/AC3/AC4 depend on the LANES+priority layers")
def test_accept_three_robot_stream():
    """Merged-spec AC1 (rect contacts), AC3 (throughput/cycles), AC4 (waits)."""
    m = fs.run(fs.random_stream(seed=7), 480.0)
    assert m["contacts"] == 0 and m["min_rect_gap_m"] >= 0.03
    assert max(m["max_wait_s"].values()) <= 30.0
    assert not any(abs(w - 20.0) < 0.5 for w in m["max_wait_s"].values()), \
        "the fixed ACQ_WAIT_TIMEOUT_S cap signature is back"
    assert not any(v["events"] for v in m["plan_zone_double_occupancy"].values())
    assert m["deliveries"] >= 14          # AC3, LANES (SPINE baseline was 10)
    cyc = m["task_cycle_s"]
    assert cyc["mean_s"] is not None and cyc["mean_s"] <= 80.0
    if cyc["second_half_mean_s"] is not None:
        assert cyc["second_half_mean_s"] <= 1.15 * cyc["first_half_mean_s"]
    assert m["duplicate_owner_max_s"] <= 1.0
    assert math.isfinite(m["min_separation_m"])


@pytest.mark.skipif(not os.environ.get("FLEET_SIM_LONG"),
                    reason="8-minute stress run; set FLEET_SIM_LONG=1")
@pytest.mark.xfail(strict=False,
                   reason="AC6 depends on the zones-survive-silence layer")
def test_accept_stress_no_double_hold():
    """AC6: 40 % state+intent loss and 10-16 s heartbeat drops (stress())."""
    m = fs.run(fs.stress(seed=7), 480.0)
    assert m["peer_failures_detected"] > 0            # the faults did bite
    assert m["arbiter_double_hold_ticks"] == {}
    assert not any(v["events"] for v in m["plan_zone_double_occupancy"].values())
    assert m["contacts"] == 0
    assert m["deliveries"] >= 11
    assert m["duplicate_owner_max_s"] <= 1.0


@pytest.mark.skipif(not os.environ.get("FLEET_SIM_LONG"),
                    reason="2-minute scenario pair; set FLEET_SIM_LONG=1")
@pytest.mark.xfail(strict=False,
                   reason="AC7 depends on the make-way/priority layer")
def test_accept_converge_and_three_at_once():
    """AC7: the user's 3-stuck case. All pickups within 120 s, three-stuck
    time <= 5 s, no mutual yields."""
    for sc in (fs.converge(), fs.three_at_once()):
        m = fs.run(sc, 125.0)
        picked = {e["task"] for e in m["events"] if e["kind"] == "pickup"}
        assert len(picked) == 3, (sc.name, sorted(picked))
        last_pickup = max(e["t"] for e in m["events"] if e["kind"] == "pickup")
        assert last_pickup <= 120.0, (sc.name, last_pickup)
        assert m["three_stuck_s"] <= 5.0, sc.name
        assert m["mutual_yield_events"] == 0, sc.name
        assert m["contacts"] == 0, sc.name
