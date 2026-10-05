"""Start a coordination route where the robot actually is.

Why this exists: Nav2 Jazzy's RegulatedPurePursuit looks for the robot only
within the FIRST ``max_robot_pose_search_dist`` metres of a new plan (default:
half the local costmap, 2 m here) and keeps poses no farther than the costmap
extent. The fleet agent publishes its whole route from where it was PLANNED.
If that route is (re)sent after the robot has driven more than ~2 m along it -
e.g. the bridge's retry after the progress checker aborted a goal while the
robot waited at a STOP permit - the controller finds no pose near the robot,
throws "Resulting plan has 0 poses in it", aborts, and the bridge re-sends the
same stale route forever (609 aborts in the 2026-10-04 Webots run).

So every goal handed to Nav2 starts at the route pose nearest the robot, and
if the robot has drifted off the route, at the robot itself.

Pure functions only - no ROS imports - so they are unit-testable anywhere.
"""
import math
from typing import Optional, Sequence, Tuple

REJOIN_M = 0.30      # farther than this from the route: start at the robot


def trim_to_robot(points: Sequence[Tuple[float, float]],
                  robot_xy: Optional[Tuple[float, float]],
                  rejoin_m: float = REJOIN_M) -> Tuple[int, bool]:
    """(start_index, prepend_robot_pose) for a route of (x, y) points.

    start_index : first route pose to keep - the one nearest the robot
                  (earliest on ties, so a fresh route starting at the robot
                  is passed through unchanged).
    prepend     : the robot is more than rejoin_m from that pose, so the
                  goal should begin at the robot's own position.
    Unknown robot pose -> (0, False): send the route as published.
    """
    if not points or robot_xy is None:
        return 0, False
    rx, ry = robot_xy
    best_i, best_d = 0, math.inf
    for i, (x, y) in enumerate(points):
        d = math.hypot(x - rx, y - ry)
        if d < best_d - 1e-9:
            best_i, best_d = i, d
    return best_i, best_d > rejoin_m
