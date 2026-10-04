"""Yielding robots must never park on another robot's destination.

webots_userrestart7 (2026-10-04), 0 deliveries in 8 min: robot_3 made way
for robot_1 and parked on (2.55, -1.95) - robot_2's pickup. robot_2 (not a
higher-priority robot, so make-way ignored its goal) sat in the spine's
south throat waiting on the robot standing on its goal, and every later
retreat released after the 2 s minimum hold because robot_2's ~1 m intent
horizon never reached the refuge.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core.coordinator import (MW_GOAL_CLEAR_M, RETREAT_HOLD_MIN_S,
                                        FleetCoordinator)
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import Intent, Pose2D, RobotState

GRID = GridMap.from_yaml(str(pathlib.Path(__file__).resolve().parents[1]
                             / "config" / "warehouse_grid.yaml"))
NOW = 1000.0
PICKUP_R2 = (2.55, -1.95)


def _coord(rid, xy, theta):
    c = FleetCoordinator(rid, GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(xy[0], xy[1], theta)
    c.state.loc_sigma_lat = 0.1
    return c


def _peer(rid, xy, theta, goal=None, cells=(), score=0.0):
    st = RobotState(robot_id=rid, seq=1, stamp=NOW,
                    pose=Pose2D(xy[0], xy[1], theta))
    st.status = "WAITING"
    st.priority_score = score
    st.loc_sigma_lat = 0.1
    st.intent = Intent()
    st.intent.cells = [GRID.world_to_cell(*p) for p in cells]
    if goal is not None:
        st.intent.goal = Pose2D(goal[0], goal[1], 0.0)
    return st


def _knot():
    """The live geometry: robot_3 at the throat, robot_2 in the NB lane
    heading for its pickup south-east of the spine, robot_1 to the west."""
    c = _coord("robot_3", (0.69, -2.68), math.pi)
    r2 = _peer("robot_2", (0.49, -1.70), 1.39, goal=PICKUP_R2,
               cells=[(0.45, -1.65), (0.35, -1.65), (0.25, -1.65)])
    r1 = _peer("robot_1", (-1.62, -2.52), 0.19, goal=(-2.45, -1.95),
               cells=[(-1.75, -2.55), (-1.95, -2.45)], score=3000.0)
    for st in (r1, r2):
        assert c.registry.update(st, now=NOW)
    return c, r1, r2


def test_refuge_is_never_near_a_peer_goal():
    c, _, _ = _knot()
    for blockers in (["robot_2"], ["robot_1"], ["robot_1", "robot_2"]):
        refuge = c._find_refuge(NOW, blockers)
        if refuge is None:
            continue
        x, y = GRID.cell_to_world(refuge)
        assert math.hypot(x - PICKUP_R2[0], y - PICKUP_R2[1]) >= MW_GOAL_CLEAR_M - 1e-6, (
            f"refuge {refuge} ({x:.2f},{y:.2f}) parks on robot_2's pickup")


def test_make_way_target_avoids_lower_priority_peer_goal():
    c, r1, _ = _knot()
    target = c._find_make_way_target(r1, NOW)     # making way for robot_1
    if target is not None:
        x, y = GRID.cell_to_world(target)
        assert math.hypot(x - PICKUP_R2[0], y - PICKUP_R2[1]) >= MW_GOAL_CLEAR_M - 1e-6


def test_retreat_hold_waits_while_blocker_goal_is_beside_me():
    """Parked beside robot_2's goal (only possible via a fallback), the hold
    must not release after the minimum just because robot_2's short intent
    is still far away."""
    c, _, r2 = _knot()
    c.state.pose = Pose2D(PICKUP_R2[0] + 0.3, PICKUP_R2[1], 0.0)
    c._retreat = dict(cell=GRID.world_to_cell(*PICKUP_R2), saved_goal=None,
                      blockers=["robot_2"], phase="HOLD", t0=NOW - 5.0,
                      hold_t0=NOW - RETREAT_HOLD_MIN_S - 1.0)
    assert not c._retreat_cleared(NOW)
    # Control: once robot_2 is headed elsewhere the hold releases.
    moved_on = _peer("robot_2", (0.49, -1.70), 1.39, goal=(-3.0, 3.75),
                     cells=[(0.45, -1.55), (0.45, -1.45)])
    moved_on.seq, moved_on.stamp = 2, NOW + 0.1
    assert c.registry.update(moved_on, now=NOW + 0.1)
    c.registry.update_intent("robot_2", moved_on.intent, 2, now=NOW + 0.1,
                             stamp=NOW + 0.1)
    assert c._retreat_cleared(NOW + 0.1)
