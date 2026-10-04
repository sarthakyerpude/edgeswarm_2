"""Strict directional lanes - the U-TURN GATE (traffic.uturn_gate) and its
execution wiring in the coordinator (_uturn_permit / uturn_active).

THE RULES under test: a U-turn may cut across into the opposite lane
ANYWHERE, but only after checking the space; if the check fails, wait in
your own lane or drive on to where the turn fits. One predicate serves
planning and execution; all inputs are the broadcast peer states.

The grid here is a synthetic two-lane vertical corridor with the planner's
contract functions STUBBED as instance attributes (lane_band_at /
in_crossing_zone), so these tests pin MY gate semantics independently of the
planner agent's in-flight gridmap internals:

    rows 0/29 and cols 0/23 are walls; free cols 1..22 (2.2 m wide)
    SB lane = cols 1..11, NB lane = cols 12..22  (divider at col 11.5)
    world x = (col+0.5)*0.1, y = (row+0.5)*0.1
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import traffic
from amr_fleet.core.coordinator import (UTURN_BLOCKED_REPLAN_S,
                                        FleetCoordinator)
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Intent, Pose2D, RobotState

NOW = 500.0
H, W = 30, 24


def make_grid(cross_rows=None):
    occ = []
    for r in range(H):
        if r in (0, H - 1):
            occ.append("#" * W)
        else:
            occ.append("#" + "." * (W - 2) + "#")
    g = GridMap(occ, 0.1, (0.0, 0.0), {})

    cr = cross_rows or ()

    def in_crossing_zone(row, col):
        return bool(cr) and cr[0] <= row <= cr[1] and 1 <= col <= W - 2

    def lane_band_at(row, col):
        if not (1 <= row <= H - 2 and 1 <= col <= W - 2):
            return None
        return ("corr", "NB" if col >= 12 else "SB")

    g.lane_band_at = lane_band_at          # instance attrs shadow the class
    g.in_crossing_zone = in_crossing_zone
    return g


def _peer(rid, cell, theta=-math.pi / 2, v=0.0, sigma=0.05, grid=None):
    x, y = grid.cell_to_world(cell)
    st = RobotState(robot_id=rid, seq=1, stamp=NOW, pose=Pose2D(x, y, theta))
    st.v = v
    st.loc_sigma_lat = sigma
    st.intent = Intent()
    return st


GRID = make_grid()
TURN_CELLS = [(12, c) for c in (11, 10, 9, 8, 7)]   # divider -> SB centre
MY_POSE = (*GRID.cell_to_world((12, 16)), math.pi / 2, 0.05)
MY_BAND = ("corr", "NB")
TGT_BAND = ("corr", "SB")


def gate(peers=(), pose=MY_POSE, cells=TURN_CELLS, grid=GRID):
    if not isinstance(peers, dict):
        peers = {p.robot_id: p for p in peers}
    return traffic.uturn_gate(NOW, pose, MY_BAND, TGT_BAND, cells,
                              peers, grid)


# ------------------------------------------------------------- the predicate
def test_clear_with_no_traffic():
    ok, reason = gate()
    assert ok and reason == "clear"


def test_a_approaching_mover_in_target_band_blocks():
    p = _peer("robot_1", (22, 7), v=1.0, grid=GRID)   # 1.0 m upstream, SB
    ok, reason = gate([p])
    assert not ok
    assert "approach" in reason and "robot_1" in reason


def test_a_stationary_peer_in_target_band_outside_margin_is_clear():
    p = _peer("robot_1", (22, 7), v=0.0, grid=GRID)   # parked 1.0 m upstream
    ok, reason = gate([p])
    assert ok, reason


def test_a_peer_already_past_the_turn_is_clear():
    p = _peer("robot_1", (4, 7), v=1.0, grid=GRID)    # 0.8 m DOWNstream (SB)
    ok, reason = gate([p])
    assert ok, reason


def test_a_threat_scales_with_my_turn_duration():
    occ = ["#" * W if r in (0, 69) else "#" + "." * (W - 2) + "#"
           for r in range(70)]
    tall = GridMap(occ, 0.1, (0.0, 0.0), {})
    tall.lane_band_at = (lambda row, col:
                         (("corr", "NB" if col >= 12 else "SB")
                          if 1 <= row <= 68 and 1 <= col <= W - 2 else None))
    tall.in_crossing_zone = lambda row, col: False
    # 4.6 m upstream at cruise: inside stop(0.55) + v*UTURN_DURATION_S(4.0)
    # + body(0.6) = 5.15 m of threat.
    p = _peer("robot_1", (58, 7), v=1.0, grid=tall)
    ok, reason = traffic.uturn_gate(NOW, MY_POSE, MY_BAND, TGT_BAND,
                                    TURN_CELLS, {"robot_1": p}, tall)
    assert not ok and "approach" in reason
    p2 = _peer("robot_1", (65, 7), v=1.0, grid=tall)  # 5.3 m: past the threat
    ok, reason = traffic.uturn_gate(NOW, MY_POSE, MY_BAND, TGT_BAND,
                                    TURN_CELLS, {"robot_1": p2}, tall)
    assert ok, reason


def test_b_occupant_of_turn_cells_blocks():
    p = _peer("robot_2", (12, 9), v=0.0, grid=GRID)
    ok, reason = gate([p])
    assert not ok
    assert "occupied" in reason and "robot_2" in reason


def test_c_close_follower_in_my_band_blocks():
    p = _peer("robot_3", (8, 16), theta=math.pi / 2, v=0.0, grid=GRID)
    ok, reason = gate([p])                 # 0.4 m behind me, my lane
    assert not ok
    assert "follower" in reason and "robot_3" in reason


def test_c_far_follower_is_clear_but_fast_one_is_not():
    slow = _peer("robot_3", (2, 16), theta=math.pi / 2, v=0.0, grid=GRID)
    ok, reason = gate([slow])              # 1.0 m behind, stationary
    assert ok, reason
    fast = _peer("robot_3", (2, 16), theta=math.pi / 2, v=1.0, grid=GRID)
    ok, reason = gate([fast])              # 1.0 m behind at cruise
    assert not ok and "follower" in reason


def test_c_leader_ahead_in_my_band_is_clear():
    p = _peer("robot_3", (20, 16), theta=math.pi / 2, v=0.0, grid=GRID)
    ok, reason = gate([p])
    assert ok, reason


def test_d_static_fit_rejects_undrivable_turn_cells():
    cells = TURN_CELLS + [(12, 2)]         # 0.15 m clearance: not drivable
    ok, reason = gate(cells=cells)
    assert not ok
    assert "static fit" in reason and "(12,2)" in reason


def test_d_sigma_margin_scales_the_pivot_fit():
    # The PIVOT (widest turn cell) must fit the swept disc + sigma margin.
    # cols 6..7 offer at most 0.65 m: enough at sigma 0.05 (need 0.357),
    # not at sigma 0.3 (need 0.286 + 1.414*0.3 = 0.71).
    cells = [(12, 7), (12, 6)]
    ok, reason = gate(cells=cells,
                      pose=(*GRID.cell_to_world((12, 16)), math.pi / 2, 0.05))
    assert ok, reason
    ok, reason = gate(cells=cells,
                      pose=(*GRID.cell_to_world((12, 16)), math.pi / 2, 0.3))
    assert not ok and "pivot" in reason
    # A wide crossing (col 11 offers 1.05 m) fits even the sigma-0.3 sweep:
    # the engineered turnarounds stay usable to a degraded robot.
    ok, reason = gate(pose=(*GRID.cell_to_world((12, 16)), math.pi / 2, 0.3))
    assert ok, reason


def test_gate_is_deterministic_and_input_shape_tolerant():
    p = _peer("robot_1", (22, 7), v=1.0, grid=GRID)
    r1 = gate({"robot_1": p})
    r2 = gate([p])                         # iterable instead of dict
    assert r1 == r2 == gate({"robot_1": p})
    # Pose2D-shaped my_pose works too (sigma then floors to the clean floor).
    pose = Pose2D(*GRID.cell_to_world((12, 16)), math.pi / 2)
    ok, reason = traffic.uturn_gate(NOW, pose, MY_BAND, TGT_BAND, TURN_CELLS,
                                    {}, GRID)
    assert ok, reason


def test_empty_turn_cells_never_pass():
    ok, reason = gate(cells=[])
    assert not ok and "no turn cells" in reason


# --------------------------------------------------------- coordinator wiring
def _coord(grid, cell=(12, 16), theta=math.pi / 2):
    c = FleetCoordinator("robot_9", grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*grid.cell_to_world(cell), theta)
    c.state.loc_sigma_lat = 0.05
    return c


def _uturn_path():
    return ([(r, 16) for r in range(5, 13)]
            + [(12, c) for c in range(15, 6, -1)]
            + [(r, 7) for r in range(11, 1, -1)])


def test_path_uturn_detection_finds_the_crossing_window():
    c = _coord(GRID, cell=(5, 16))
    c.path = _uturn_path()
    info = c._path_uturn(0)
    assert info is not None
    assert info["my_band"] == ("corr", "NB")
    assert info["target"] == ("corr", "SB")
    assert c.path[info["k0"]] == (12, 12)          # last own-lane cell
    assert set(info["cells"]) == set(TURN_CELLS)   # divider -> first SB step


def test_straight_path_has_no_uturn_and_uturn_active_defaults_false():
    c = _coord(GRID, cell=(5, 16))
    assert c.uturn_active is False                 # the audit/webapp contract
    c.path = [(r, 16) for r in range(5, 25)]
    assert c._path_uturn(0) is None
    assert c._uturn_permit(NOW) is None
    assert c.uturn_active is False


def test_gate_failure_stops_me_in_my_own_lane_with_the_blocker_named():
    c = _coord(GRID, cell=(12, 13))                # 0.1 m short of the turn
    c.path = _uturn_path()
    peer = _peer("robot_1", (22, 7), v=1.0, grid=GRID)
    assert c.registry.update(peer, now=NOW)
    p = c._uturn_permit(NOW)
    assert p is not None and p.action == "STOP"
    assert p.reason.startswith("U-turn gate:")
    assert p.blocking_robot == "robot_1"
    assert c._acq_waiting is True                  # structural, bounded wait
    assert c.uturn_active is False


def test_gate_pass_engages_uturn_active_and_does_not_restrict():
    c = _coord(GRID, cell=(12, 13))
    c.path = _uturn_path()
    assert c._uturn_permit(NOW) is None            # nobody around
    assert c.uturn_active is True


def test_committed_turn_always_finishes_even_if_the_gate_flips():
    c = _coord(GRID, cell=(12, 9))                 # astride the divider
    c.path = _uturn_path()
    peer = _peer("robot_1", (22, 7), v=1.0, grid=GRID)
    assert c.registry.update(peer, now=NOW)
    assert c._uturn_permit(NOW) is None            # no mid-crossing freeze
    assert c.uturn_active is True


def test_blocked_turn_replans_to_where_the_turn_fits():
    c = _coord(GRID, cell=(12, 13))
    c.path = _uturn_path()
    c.goal_cell = (2, 7)
    peer = _peer("robot_1", (22, 7), v=1.0, grid=GRID)
    assert c.registry.update(peer, now=NOW)
    c._uturn_block_t0 = NOW - UTURN_BLOCKED_REPLAN_S - 0.5
    replans_before = c.replan_count
    p = c._uturn_permit(NOW)
    assert c.replan_count == replans_before + 1, (
        "after UTURN_BLOCKED_REPLAN_S the robot must drive on to where the "
        "turn fits, through the ordinary replan machinery")
    assert p is None and c._uturn_block_t0 is None
    assert c.path and c.path[-1] == (2, 7)


def test_full_tick_resets_uturn_active_each_tick():
    c = _coord(GRID, cell=(12, 13))
    c.goal_cell = (2, 7)
    c.path = _uturn_path()
    c.tick(NOW)
    assert c.uturn_active is True                  # engaging a clear turn
    c.path = [(r, 16) for r in range(12, 25)]      # reversal gone
    c.goal_cell = (24, 16)
    c.tick(NOW + 0.1)
    assert c.uturn_active is False
