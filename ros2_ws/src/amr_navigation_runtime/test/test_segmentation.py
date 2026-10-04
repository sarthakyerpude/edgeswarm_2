"""fleet_goal_bridge rectilinear segmentation (user directive 2026-10-04:
straight segments + in-place spins at turn points, no arc turns).

Needs the ROS environment sourced (fleet_goal_bridge imports rclpy), same as
running the bridge tests:
    python3 -m pytest -q amr_navigation_runtime/test/test_segmentation.py
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_navigation_runtime.fleet_goal_bridge import (TURN_SPLIT_RAD,
                                                      split_rectilinear)

N, E, W, S = math.pi / 2, 0.0, math.pi, -math.pi / 2


def _xy(seg):
    return [(round(x, 2), round(y, 2)) for x, y, _ in seg]


def test_straight_route_is_one_segment():
    pts = [(0.0, 0.1 * i) for i in range(10)]
    segs = split_rectilinear(pts)
    assert len(segs) == 1
    assert all(abs(yaw - N) < 1e-9 for _, _, yaw in segs[0][:-1])


def test_square_left_turn_splits_at_the_corner_with_spin_yaw():
    pts = ([(0.0, 0.1 * i) for i in range(6)]          # N to (0.0, 0.5)
           + [(-0.1 * i, 0.5) for i in range(1, 6)])   # W to (-0.5, 0.5)
    segs = split_rectilinear(pts)
    assert len(segs) == 2
    # segment 1 ends AT the corner, its final yaw is the NEXT leg's heading
    assert _xy(segs[0])[-1] == (0.0, 0.5)
    assert abs(abs(segs[0][-1][2]) - W) < 1e-9
    # segment 2 starts at the corner and runs straight west
    assert _xy(segs[1])[0] == (0.0, 0.5)
    assert all(abs(abs(yaw) - W) < 1e-9 for _, _, yaw in segs[1])


def test_final_route_yaw_is_preserved_for_docking():
    pts = [(0.0, 0.1 * i) for i in range(5)]
    segs = split_rectilinear(pts, final_yaw=1.234)
    assert abs(segs[-1][-1][2] - 1.234) < 1e-9


def test_45_degree_staircase_stays_one_smooth_segment():
    """Lane-merge tapers (alternating 45 deg jogs) keep smooth motion:
    every step heading stays within TURN_SPLIT_RAD of the first."""
    pts, x, y = [(0.0, 0.0)], 0.0, 0.0
    for i in range(8):
        if i % 2:
            x += 0.1
        else:
            x += 0.1
            y += 0.1
        pts.append((x, y))
    segs = split_rectilinear(pts)
    assert len(segs) == 1
    assert max(abs(h - pts_heading) for _, _, h in segs[0]
               for pts_heading in [segs[0][0][2]]) < TURN_SPLIT_RAD


def test_decomposed_45_plus_45_corner_still_splits():
    """A 90 deg turn written as two 45s one cell apart is the same physical
    swing: the cumulative-reference rule splits it at the second 45."""
    pts = ([(0.0, 0.1 * i) for i in range(5)]          # N
           + [(-0.1, 0.5)]                             # NW jog
           + [(-0.1 - 0.1 * i, 0.5) for i in range(1, 5)])   # W run
    segs = split_rectilinear(pts)
    assert len(segs) == 2
    assert abs(abs(segs[0][-1][2]) - W) < 1e-9


def test_dead_end_u_turn_becomes_two_spins():
    pts = ([(0.0, 0.1 * i) for i in range(5)]          # N
           + [(-0.1, 0.4), (-0.2, 0.4)]                # short W stub
           + [(-0.2, 0.4 - 0.1 * i) for i in range(1, 5)])   # S run
    segs = split_rectilinear(pts)
    assert len(segs) == 3
    assert abs(abs(segs[0][-1][2]) - W) < 1e-9         # spin 1: N -> W
    assert abs(segs[1][-1][2] - S) < 1e-9              # spin 2: W -> S


def test_single_pose_and_duplicates_are_safe():
    assert split_rectilinear([]) == []
    segs = split_rectilinear([(1.0, 2.0)], final_yaw=0.5)
    assert segs == [[(1.0, 2.0, 0.5)]]
    segs = split_rectilinear([(0.0, 0.0), (0.0, 0.0), (0.0, 0.1)])
    assert len(segs) == 1 and len(segs[0]) == 2
