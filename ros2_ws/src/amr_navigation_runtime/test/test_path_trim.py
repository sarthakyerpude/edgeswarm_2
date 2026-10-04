"""fleet_goal_bridge route trimming (the 'Resulting plan has 0 poses' freeze)."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_navigation_runtime.path_trim import REJOIN_M, trim_to_robot

# A straight 8 m route along +x in 0.1 m steps, like a coordination path.
ROUTE = [(0.1 * i, 0.0) for i in range(81)]


def test_fresh_route_starting_at_robot_is_unchanged():
    assert trim_to_robot(ROUTE, (0.0, 0.0)) == (0, False)


def test_route_resent_after_robot_drove_4m_starts_at_robot():
    """The freeze: robot 4 m along (beyond RPP's 2 m search window)."""
    start, prepend = trim_to_robot(ROUTE, (4.02, 0.03))
    assert start == 40 and not prepend
    assert len(ROUTE) - start == 41          # the passed 4 m are dropped


def test_robot_off_route_gets_its_own_pose_prepended():
    start, prepend = trim_to_robot(ROUTE, (2.0, 0.8))
    assert start == 20 and prepend


def test_rejoin_threshold():
    assert not trim_to_robot(ROUTE, (1.0, REJOIN_M - 0.01))[1]
    assert trim_to_robot(ROUTE, (1.0, REJOIN_M + 0.01))[1]


def test_robot_at_the_end_keeps_the_final_pose():
    """Docking/final approach: the final (dock) pose must survive trimming."""
    start, _ = trim_to_robot(ROUTE, (8.05, 0.0))
    assert start == len(ROUTE) - 1


def test_unknown_robot_pose_sends_route_as_is():
    assert trim_to_robot(ROUTE, None) == (0, False)
    assert trim_to_robot([], (1.0, 1.0)) == (0, False)


def test_ties_pick_the_earliest_pose():
    loop = [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)]   # route revisits a point
    assert trim_to_robot(loop, (0.0, 0.0))[0] == 0
