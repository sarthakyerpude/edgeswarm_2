"""Give-way / make-way / inversion (increment 3, R1).

The user's case: all three robots stuck at one spot 'trying to figure out
the path'. Priority now gives a total order; the lower robot holds short
(give-way) or physically clears the corridor (make-way), and the measured
pathologies (first-leg approach lock, J_N goal livelock, mutual-yield flip,
no-target stall) each have a pinned rule here.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import astar, priority as prio, traffic      # noqa: E402
from amr_fleet.core.coordinator import (                         # noqa: E402
    FleetCoordinator, MW_CLEAR_CORRIDOR_M, MW_FIRST_LEG_M, MW_GOAL_CLEAR_M,
    MW_INVERT_S, ROW_PEER_LOOK_M)
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Intent, Pose2D, RobotState, WAITING

NOW = 100.0
GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))


def _open_grid():
    return GridMap(["." * 60] * 60, 0.1, (-3.0, -3.0), {})


def _coord(grid, rid, x, y, theta=0.0):
    c = FleetCoordinator(rid, grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(x, y, theta)
    return c


def _peer(grid, rid, x, y, theta, score, cells=(), v=0.0, status="MOVING",
          waiting_for="", waiting_time=0.0, goal=None, zones=()):
    st = RobotState(robot_id=rid, seq=1, stamp=NOW, pose=Pose2D(x, y, theta),
                    v=v, status=status, waiting_for=waiting_for,
                    waiting_time=waiting_time, priority_score=score)
    st.intent = Intent(cells=list(cells), t_enter=[NOW] * len(cells),
                       t_exit=[NOW + 60] * len(cells), zones=list(zones))
    if goal is not None:
        st.intent.goal = Pose2D(goal[0], goal[1], 0.0)
    return st


def _col_cells(grid, x, y0, y1):
    step = grid.resolution if y1 >= y0 else -grid.resolution
    out, y = [], y0
    while (y <= y1) if step > 0 else (y >= y1):
        out.append(grid.world_to_cell(x, y))
        y += step
    return out


# ------------------------------------------------------------------ give-way
def test_give_way_layer_is_off_by_default():
    """Measured: with hold-short ON, 6-seed mean was 10.9-12.1 deliveries
    with up to 256 s waits; OFF it is 15.3 with 10.4 s. The crossings here
    are already ordered by zones/lanes/envelope/goal-queue/make-way."""
    assert FleetCoordinator.give_way_enabled is False


def test_give_way_stops_only_the_lower_robot():
    g = _open_grid()
    crossing = _col_cells(g, 1.0, -1.5, 1.5)
    # LOW me (goal only -> L=1000) vs HIGH crossing peer (L=3000): STOP.
    me = _coord(g, "robot_2", 0.6, 0.0)
    me.give_way_enabled = True
    assert me.set_goal(g.world_to_cell(2.5, 0.0), now=NOW)
    me.on_peer_state(_peer(g, "robot_1", 1.0, -1.5, math.pi / 2, 3000.0,
                           crossing, v=0.3), NOW)
    p = me.tick(NOW)
    assert p.action in ("STOP", "SLOW"), p
    assert p.blocking_robot == "robot_1", p
    assert "give way" in p.reason or "safety" in p.reason, p
    # HIGH me vs LOW crossing peer: the give-way layer never fires.
    hi = _coord(g, "robot_2", 0.6, 0.0)
    hi.give_way_enabled = True
    assert hi.set_goal(g.world_to_cell(2.5, 0.0), now=NOW)
    hi.on_peer_state(_peer(g, "robot_1", 1.0, -1.5, math.pi / 2, 500.0,
                           crossing, v=0.3), NOW)
    hi._cmp_L = 3000.0
    assert hi._give_way_permit(NOW) is None


def test_same_direction_following_never_triggers_give_way():
    g = _open_grid()
    ahead = _col_cells(g, 0.0, 0.0, 0.0)  # dummy; build along +x instead
    ahead = [g.world_to_cell(x / 10.0, 0.0) for x in range(8, 28)]
    me = _coord(g, "robot_2", 0.0, 0.0)
    me.give_way_enabled = True
    assert me.set_goal(g.world_to_cell(2.5, 0.0), now=NOW)
    me.on_peer_state(_peer(g, "robot_1", 0.8, 0.0, 0.0, 3000.0, ahead,
                           v=0.3), NOW)
    me._cmp_L = 1000.0
    assert me._give_way_permit(NOW) is None, \
        "car-following is the envelope's job"


def test_three_robot_total_order_top_never_yields():
    g = _open_grid()
    crossing = _col_cells(g, 1.0, -1.5, 1.5)
    top = _coord(g, "robot_3", 0.6, 0.0)
    top.give_way_enabled = True
    assert top.set_goal(g.world_to_cell(2.5, 0.0), now=NOW)
    top.on_peer_state(_peer(g, "robot_1", 1.0, -1.5, math.pi / 2, 3000.0,
                            crossing, v=0.3), NOW)
    top.on_peer_state(_peer(g, "robot_2", 1.0, 1.5, -math.pi / 2, 2000.0,
                            list(reversed(crossing)), v=0.3), NOW)
    top._cmp_L = 4000.0
    assert top._give_way_permit(NOW) is None
    assert prio.pick_victim({"robot_1": 3000.0, "robot_2": 2000.0,
                             "robot_3": 4000.0}) == "robot_2"


def test_encounter_freeze_keeps_the_first_decision():
    g = _open_grid()
    me = _coord(g, "robot_2", 0.0, 0.0)
    me._cmp_L = 1000.0
    st = _peer(g, "robot_1", 1.0, 0.0, 0.0, 3000.0)
    assert me._peer_outranks_me(st, NOW)
    st.priority_score = 500.0             # its broadcast L dropped mid-pass
    assert me._peer_outranks_me(st, NOW + 1.0), "frozen while within 2 m"
    st.pose = Pose2D(5.0, 0.0, 0.0)        # separated: freeze expires
    assert not me._peer_outranks_me(st, NOW + 2.0)


# ------------------------------------------------------------------ make-way
def _stuck_pair(g):
    """HIGH robot_1 WAITING on me (robot_2); I stand on its corridor."""
    corridor = _col_cells(g, 0.0, -1.2, 1.5)
    me = _coord(g, "robot_2", 0.0, 0.0, math.pi / 2)
    st = _peer(g, "robot_1", 0.0, -1.2, math.pi / 2, 3000.0, corridor,
               status=WAITING, waiting_for="robot_2", waiting_time=1.0,
               goal=(0.0, 2.0))
    me.on_peer_state(st, NOW)
    return me, st


def test_make_way_moves_the_blocker_clear_of_corridor_and_goal():
    g = _open_grid()
    me, st = _stuck_pair(g)
    p = me.tick(NOW)
    assert me._retreat is not None and me._retreat.get("make_way")
    assert me.state.status == "YIELDING" and me.state.waiting_for == ""
    assert p.action in ("GO", "SLOW") and p.speed_scale > 0.0
    tx, ty = g.cell_to_world(me._retreat["cell"])
    # Target clear of the beneficiary's corridor BY DISTANCE and of its goal.
    for cell in st.intent.cells:
        cx, cy = g.cell_to_world(cell)
        assert math.hypot(tx - cx, ty - cy) >= MW_CLEAR_CORRIDOR_M - 1e-6
    assert math.hypot(tx - 0.0, ty - 2.0) >= MW_GOAL_CLEAR_M - 1e-6
    assert math.hypot(tx - st.pose.x, ty - st.pose.y) >= 0.7


def test_make_way_first_leg_never_approaches_the_higher_robot():
    g = _open_grid()
    me, st = _stuck_pair(g)
    me.tick(NOW)
    assert me._retreat is not None and me.path
    d0 = math.hypot(me.state.pose.x - st.pose.x,
                    me.state.pose.y - st.pose.y)
    cum, px, py = 0.0, me.state.pose.x, me.state.pose.y
    for cell in me.path[1:]:
        cx, cy = g.cell_to_world(cell)
        cum += math.hypot(cx - px, cy - py)
        px, py = cx, cy
        if cum > MW_FIRST_LEG_M:
            break
        assert math.hypot(cx - st.pose.x, cy - st.pose.y) >= d0 - 1e-6, \
            "the measured lock: a first leg toward the waiter re-blocks it"


def test_zone_wait_skip_no_make_way_when_the_wait_is_for_my_zone():
    """Measured +3 deliveries: when the peer waits for a ranked zone I hold,
    vacating ground changes nothing - the zone protocol resolves it."""
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((60, 60)), math.pi / 2)
    c.arbiter.state["SPINE_M"] = "HELD"
    st = _peer(GRID, "robot_2", *GRID.cell_to_world((40, 60)), math.pi / 2,
               4000.0, [(50, 60), (55, 60), (60, 60)],
               status=WAITING, waiting_for="robot_1", waiting_time=2.0,
               zones=["SPINE_M"])
    c.on_peer_state(st, NOW)
    assert c._maybe_make_way(NOW) is None
    assert c._retreat is None


def test_inversion_higher_robot_makes_way_after_mutual_stall():
    """The design-1 no-target trace: the lower robot is boxed in and cannot
    yield. After MW_INVERT_S of mutual waiting the HIGHER robot moves
    instead of stalling past 60 s."""
    g = _open_grid()
    corridor = _col_cells(g, 0.0, -1.2, 1.0)
    me = _coord(g, "robot_1", 0.0, 0.0, -math.pi / 2)
    me._cmp_L = 3000.0
    me.state.priority_score = 3000.0
    me.state.waiting_for = "robot_2"
    me.state.waiting_time = MW_INVERT_S + 0.5
    st = _peer(g, "robot_2", 0.0, -1.2, math.pi / 2, 1000.0, corridor,
               status=WAITING, waiting_for="robot_1",
               waiting_time=MW_INVERT_S + 0.5)
    me.on_peer_state(st, NOW)
    assert me._maybe_make_way(NOW) is not None or me._retreat is not None
    assert me._retreat is not None and me._retreat.get("make_way")
    assert me._retreat["beneficiary"] == "robot_2"


def test_make_way_failure_counts_toward_deadlock_handoff():
    """No valid target (walled-in): the attempt fails, is retried at 1 Hz,
    and after MW_FAILS_TO_DEADLOCK the pair is left to the deadlock layer."""
    rows = ["#" * 20] + ["#" + "." * 18 + "#"] * 1 + ["#" * 20] * 18
    g = GridMap(rows, 0.1, (0.0, 0.0), {})          # a 1-cell-high slot
    me = _coord(g, "robot_2", 0.95, 0.15)
    corridor = [g.world_to_cell(x / 10.0, 0.15) for x in range(2, 18)]
    st = _peer(g, "robot_1", 0.25, 0.15, 0.0, 3000.0, corridor,
               status=WAITING, waiting_for="robot_2", waiting_time=1.0)
    me.on_peer_state(st, NOW)
    for k in range(6):
        me._maybe_make_way(NOW + 1.1 * k)
    assert me._retreat is None
    assert me.makeway_fail_count >= 1
    assert me._mw_fails.get("robot_1", 0) >= 3 or me.makeway_fail_count >= 3
