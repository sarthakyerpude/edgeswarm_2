"""Broadcast intent: undriven suffix, re-timed from now, skew-free on receipt."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import astar
from amr_fleet.core.coordinator import (FleetCoordinator, INTENT_MAX_CELLS,
                                        INTENT_MAX_HORIZON_S,
                                        INTENT_MIN_SPEED_MPS)
from amr_fleet.core.gridmap import GridMap, Zone
from amr_fleet.core.models import Intent, Pose2D

ROW = 10


def _setup(n_cells=111, zones=None):
    occ = ["." * 130 for _ in range(20)]
    grid = GridMap(occ, 0.1, (0.0, 0.0), zones or {})
    c = FleetCoordinator("robot_1", grid, lambda d: None, lambda d: None)
    c.path = [(ROW, col) for col in range(n_cells)]
    c.intent = astar.build_intent(grid, c.path, c.v_nominal, 0.0)
    c.intent.zones = grid.zones_on_path(c.path)
    return c, grid


def _at(c, grid, col):
    c.state.pose = Pose2D(*grid.cell_to_world((ROW, col)), 0.0)


def test_suffix_starts_at_current_index():
    c, grid = _setup()
    _at(c, grid, 20)
    it = c.current_intent(now=5.0)
    assert it.cells[0] == (ROW, 20)
    assert (ROW, 19) not in it.cells
    assert it.cells == c.path[20:20 + len(it.cells)]


def test_after_a_wait_every_window_starts_at_or_after_now():
    c, grid = _setup()
    _at(c, grid, 0)
    now = 10.0                         # intent was built at t=0
    it = c.current_intent(now)
    assert it.cells
    assert all(t >= now for t in it.t_enter)
    assert all(b > now for b in it.t_exit)
    assert all(b >= a for a, b in zip(it.t_enter, it.t_exit))
    # The stale replan-time windows would all lie in the past.
    assert c.intent.t_exit[len(it.cells) - 1] < now


def test_header_skew_gives_identical_local_windows():
    c, grid = _setup()
    _at(c, grid, 0)
    rx_time = 1000.0
    sender_now = 50.0
    it = c.current_intent(sender_now)
    skewed = Intent(cells=list(it.cells),
                    t_enter=[t + 2.0 for t in it.t_enter],
                    t_exit=[t + 2.0 for t in it.t_exit], zones=list(it.zones))
    a = it.to_receiver_clock(sender_now, rx_time)
    b = skewed.to_receiver_clock(sender_now + 2.0, rx_time)
    assert a.cells == b.cells
    for x, y in zip(a.t_enter + a.t_exit, b.t_enter + b.t_exit):
        assert abs(x - y) < 1e-9
    # Windows land relative to receipt: current cell occupied from rx_time.
    assert abs(a.t_enter[0] - rx_time) < 1e-9


def test_long_path_capped_at_60_cells_or_8_seconds():
    c, grid = _setup(n_cells=111)
    _at(c, grid, 0)
    c._speed_ema = 1.0                 # fast: the cell cap binds
    fast = c.current_intent(0.0)
    assert len(fast.cells) <= INTENT_MAX_CELLS
    assert len(fast.cells) == INTENT_MAX_CELLS
    assert fast.cells == c.path[:INTENT_MAX_CELLS]

    c._speed_ema = 0.0                 # stopped: floor speed, time cap binds
    slow = c.current_intent(0.0)
    assert 1 <= len(slow.cells) < INTENT_MAX_CELLS
    cell_time = grid.resolution / INTENT_MIN_SPEED_MPS
    assert (len(slow.cells) - 1) * cell_time <= INTENT_MAX_HORIZON_S + 1e-9


def test_speed_ema_tracks_measured_speed_not_nominal():
    c, grid = _setup()
    _at(c, grid, 0)
    c.state.v = -0.2                   # reversing counts as |v|
    for k in range(200):
        c._update_speed_ema(0.1 * k)
    assert abs(c._speed_ema - 0.2) < 0.01
    it = c.current_intent(20.0)
    step = it.t_exit[5] - it.t_exit[4]
    assert abs(step - grid.resolution / 0.2) < 1e-3     # not 0.1 / 0.4


def test_zones_cover_remaining_route_but_drop_passed_ones():
    zones = {"PASSED": Zone("PASSED", "intersection",
                            {(ROW, c) for c in range(2, 5)}),
             "AHEAD": Zone("AHEAD", "intersection",
                           {(ROW, c) for c in range(100, 105)})}
    c, grid = _setup(n_cells=111, zones=zones)
    assert c.intent.zones == ["PASSED", "AHEAD"]
    _at(c, grid, 10)
    c._speed_ema = 0.0                 # short suffix that ends before AHEAD
    it = c.current_intent(0.0)
    assert (ROW, 100) not in it.cells
    assert it.zones == ["AHEAD"]
    c.arbiter.state["PASSED"] = "HELD"  # never hide a zone I still hold
    assert c.current_intent(0.0).zones == ["PASSED", "AHEAD"]


def test_idle_robot_broadcasts_empty_intent():
    c, grid = _setup()
    c.path = []
    assert c.current_intent(3.0).cells == []


def test_congestion_costs_skip_passed_cells():
    it = Intent(cells=[(1, 1), (1, 2), (1, 3)],
                t_enter=[0.0, 5.0, 30.0], t_exit=[2.0, 7.0, 32.0])
    costs = astar.congestion_costs([it], now=4.0)
    assert (1, 1) not in costs         # window already over
    assert (1, 2) in costs             # inside the horizon
    assert (1, 3) not in costs         # beyond the 10 s horizon
    # Positional signature and defaults are unchanged.
    assert astar.congestion_costs([it], 4.0, 30.0, 1.0) == {(1, 2): 1.0,
                                                           (1, 3): 1.0}
