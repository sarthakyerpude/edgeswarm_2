"""AMCL re-seed must only ever add evidence (loc_validate2, robot_3, t=163).

Sigma spiked 0.02 -> 1.31 m on an aliased pose; AMCL's recovery injection
fired correctly, but the LOST handler re-seeded at "0.0 m since the last
confident fix" - the diverged pose itself - with a tight covariance, killing
the injected hypotheses. The robot then stopped and stayed 3.58 m wrong for
~7 min. Re-seed now waits for the injected cloud and needs real odometry.
"""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
fan = pytest.importorskip("amr_fleet.nodes.fleet_agent_node")
decide = fan.FleetAgentNode.reseed_decision

# The node defaults: min odometry 0.3 m, hold 8 s, max odometry 5 m.
ARGS = dict(min_odom_m=0.3, hold_s=8.0, max_odom_m=5.0)


def test_no_reseed_with_zero_odometry_ever():
    """The robot_3 case: 0.0 m since the fix = re-writing the diverged pose."""
    for lost_for in (0.0, 2.1, 8.0, 60.0):
        assert decide(0.0, lost_for, **ARGS) == "wait"


def test_waits_for_the_injected_cloud_first():
    assert decide(1.0, 2.1, **ARGS) == "wait"
    assert decide(1.0, 7.9, **ARGS) == "wait"
    assert decide(1.0, 8.0, **ARGS) == "seed"


def test_needs_real_odometry_evidence():
    assert decide(0.29, 20.0, **ARGS) == "wait"
    assert decide(0.30, 20.0, **ARGS) == "seed"


def test_too_far_to_dead_reckon_wins():
    assert decide(5.1, 0.0, **ARGS) == "too_far"
    assert decide(5.1, 30.0, **ARGS) == "too_far"
