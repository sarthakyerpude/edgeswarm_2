"""Repro driver for the mid-spine corridor pin (webots_final.log incident).

Not collected by pytest (leading underscore). Run:
    PYTHONHASHSEED=0 python3 test/_repro_pin.py [duration_s]
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fleet_sim as fs  # noqa: E402

from amr_fleet.core.models import Task  # noqa: E402


def _task(tid, pickup, dropoff, priority=0):
    return Task(task_id=tid, pickup=tuple(pickup), dropoff=tuple(dropoff),
                priority=priority, created_at=fs.T0)


def mid_spine_pin(seed=0, live=True, **kw):
    """robot_2 starts mid-corridor inside SPINE_M (heading north to L3 via
    J_AW: order SP_AW<SPINE_S<SPINE_M<SPINE_N<J_AW) while robots 1 and 3
    sit INSIDE SPINE_S (where the live run's REPLAN FAILED bursts place
    them) with tasks at the north pickups. force_sigma_lat 0.2 drops the
    LANES gate (SPINE fallback), exactly like the live run; live=True adds
    the measured bus conditions (loss, latency, robot_2 heartbeat dropouts
    that produced the 'robot_2 silent 5.6s' partition stops)."""
    bx, by = fs._cell_xy((52, 60))        # inside SPINE_M, mid corridor
    s1x, s1y = fs._cell_xy((31, 58))      # inside SPINE_S, west side (live)
    s3x, s3y = fs._cell_xy((32, 62))      # inside SPINE_S, east side (live)
    robots = [
        fs.RobotSpec("robot_1", s1x, s1y, math.pi / 2, dock=False,
                     tasks=[(0.0, _task("PIN-1", fs.L1, fs.D1))]),
        fs.RobotSpec("robot_2", bx, by, math.pi / 2, dock=False,
                     tasks=[(0.0, _task("PIN-2", fs.L3, fs.D3))]),
        fs.RobotSpec("robot_3", s3x, s3y, math.pi / 2, dock=False,
                     tasks=[(0.5, _task("PIN-3", fs.L2, fs.D2))]),
    ]
    knobs = dict(force_sigma_lat=0.2)
    if live:
        knobs.update(state_loss=0.3, latency_s=0.03, jitter_s=0.05,
                     faults=[fs.Fault("robot_2", t, t + 6.0, "heartbeat")
                             for t in range(10, 230, 40)])
    knobs.update(kw)
    return fs.Scenario(name="mid_spine_pin", robots=robots, seed=seed,
                       keep_log=True, **knobs)


def spine_wedge(seed=0, **kw):
    """The webots_final.log wedge, distilled: a compressed northbound queue
    inside the spine at fallback sigma 0.25. robot_2 leads just south of the
    SPINE_N boundary; robot_3 is jammed 0.5 m behind it, close enough that
    its K_SIG*sigma-inflated hitbox rasterises INTO SPINE_N, so robot_2
    (holding SPINE_N) stands at the acquisition point 'acquiring SPINE_N'
    with blocking robot_3 - who cannot move because robot_2 is ahead."""
    p2 = fs._cell_xy((59, 60))            # near the SPINE_M -> N boundary
    p3 = fs._cell_xy((54, 60))            # jammed 0.5 m behind, inflated in
    p1 = fs._cell_xy((25, 60))            # south throat, outside the zones
    robots = [
        fs.RobotSpec("robot_2", p2[0], p2[1], math.pi / 2, dock=False,
                     tasks=[(0.0, _task("WG-2", fs.L3, fs.D3))]),
        fs.RobotSpec("robot_3", p3[0], p3[1], math.pi / 2, dock=False,
                     tasks=[(0.0, _task("WG-3", fs.L2, fs.D2))]),
        fs.RobotSpec("robot_1", p1[0], p1[1], math.pi / 2, dock=False,
                     tasks=[(0.0, _task("WG-1", fs.L1, fs.D1))]),
    ]
    knobs = dict(force_sigma_lat=0.3)
    knobs.update(kw)
    return fs.Scenario(name="spine_wedge", robots=robots, seed=seed,
                       keep_log=True, **knobs)


SCENARIOS = {"pin": mid_spine_pin, "wedge": spine_wedge}

if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
    which = sys.argv[2] if len(sys.argv) > 2 else "pin"
    m = fs.run(SCENARIOS[which](), dur)
    print("deliveries:", m["deliveries"])
    print("pickups:", m["pickups"])
    print("contacts:", m["contacts"])
    print("replan_failed:", m["replan_failed"])
    print("deadlocks:", m["deadlocks_detected"], "/", m["deadlocks_resolved"])
    print("retreats:", m["retreats_started"], "/", m["retreats_completed"])
    print("hold_lease_releases:", m["hold_lease_releases"])
    print("max_wait_s:", m["max_wait_s"])
    print("three_stuck_s:", m["three_stuck_s"],
          "max:", m["three_stuck_max_s"])
    print("deliveries_per_robot:", m["deliveries_per_robot"])
