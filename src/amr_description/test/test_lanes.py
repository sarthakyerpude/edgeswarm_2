"""LANES mode (increment 4, R3/R7): directed lanes in the 1.6 m spine band.

NB keeps cols 63-65 (x=+0.45 centre), SB keeps 54-56 (x=-0.45), disjoint, so
two robots pass wheel-to-wheel with 0.49 m of air. The gate
(traffic.select_mode) grants LANES with hysteresis: enter when everyone near
the band localizes at sigma <= 0.18 for 3 s, leave at >= 0.30 sustained 5 s
(ghost/LOC_LOST drops at once); the pass itself is judged per-pair by the
envelope's lateral-gap rule with the live sigmas.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import astar, traffic                        # noqa: E402
from amr_fleet.core.coordinator import FleetCoordinator          # noqa: E402
from amr_fleet.core.gridmap import GridMap                       # noqa: E402
from amr_fleet.core.models import Pose2D, RobotState             # noqa: E402

YAML = pathlib.Path(__file__).resolve().parents[1] / "config" / "warehouse_grid.yaml"
GRID = GridMap.from_yaml(str(YAML))
NOW = 100.0
L1 = (88, 25)
NB_COLS = {63, 64, 65}
SB_COLS = {54, 55, 56}


def _ok_state(rid="robot_1", cell=(40, 60), sigma=0.0):
    st = RobotState(robot_id=rid, seq=1, stamp=NOW)
    st.pose = Pose2D(*GRID.cell_to_world(cell), 0.0)
    st.loc_sigma_lat = sigma
    return st


# ------------------------------------------------------------------- yaml
def test_lane_band_and_pockets_parsed_from_yaml():
    band = GRID.lanes
    assert band is not None and band.rect == (25, 52, 83, 67)
    assert band.axis == "v"
    assert (band.centre_pos, band.centre_neg) == (64, 55)
    assert band.divider == 59.5
    assert set(GRID.pockets) == {(66, 80), (74, 80), (44, 80), (52, 80),
                                 (44, 40), (52, 40)}
    assert GRID.traffic.mode == "LANES"


# ------------------------------------------------------------- lane routing
def test_nb_and_sb_lanes_are_disjoint_through_every_rack_gap():
    nb = astar.astar(GRID, (22, 60), (88, 60), mode="LANES")
    sb = astar.astar(GRID, (88, 60), (22, 60), mode="LANES")
    assert nb and sb
    nb_cols = {c for r, c in nb if 30 <= r <= 80}
    sb_cols = {c for r, c in sb if 30 <= r <= 80}
    assert nb_cols <= NB_COLS, nb_cols
    assert sb_cols <= SB_COLS, sb_cols
    assert not (nb_cols & sb_cols)
    # ...and both actually traverse every rack-gap row of the band.
    for gap in (33, 38, 55, 60, 77, 82):
        assert any(r == gap for r, _ in nb)
        assert any(r == gap for r, _ in sb)


def test_without_lane_mode_routing_is_unchanged():
    """Legacy (reservations on): no mode == SPINE (no lanes). Road model:
    EVERY mode plans the lanes - the SPINE fallback stays in lane too
    (strict directional lanes, user order 2026-10-04: no mode may ever
    route a robot into the opposite lane)."""
    legacy = GridMap.from_yaml(str(YAML))
    legacy.traffic.reservations = True
    a = astar.astar(legacy, (22, 60), (88, 60))
    b = astar.astar(legacy, (22, 60), (88, 60), mode="SPINE")
    assert a == b
    assert astar.astar(GRID, (22, 60), (88, 60)) == \
        astar.astar(GRID, (22, 60), (88, 60), mode="LANES")
    assert astar.astar(GRID, (22, 60), (88, 60), mode="SPINE") == \
        astar.astar(GRID, (22, 60), (88, 60), mode="LANES")


# ---------------------------------------------------------------- the gate
def test_gate_holds_lanes_only_after_3s_enter_threshold():
    gate = {"ok_since": None}
    me = _ok_state(sigma=0.05)
    assert traffic.select_mode(GRID, me, {}, [], 0.0, gate) == "SPINE"
    assert traffic.select_mode(GRID, me, {}, [], 2.9, gate) == "SPINE"
    assert traffic.select_mode(GRID, me, {}, [], 3.0, gate) == "LANES"
    # Entering requires sigma <= LANE_SIGMA_MAX sustained; above it the
    # enter timer re-arms.
    gate2 = {"ok_since": None}
    bad = _ok_state(sigma=traffic.LANE_SIGMA_MAX + 0.001)
    assert traffic.select_mode(GRID, bad, {}, [], 0.0, gate2) == "SPINE"
    assert traffic.select_mode(GRID, bad, {}, [], 5.0, gate2) == "SPINE"
    bad.loc_sigma_lat = traffic.LANE_SIGMA_MAX
    assert traffic.select_mode(GRID, bad, {}, [], 5.1, gate2) == "SPINE"
    assert traffic.select_mode(GRID, bad, {}, [], 8.1, gate2) == "LANES"


def test_gate_hysteresis_sticky_between_enter_and_exit_thresholds():
    """webots_final4 fix: real AMCL sigma breathes 0.05-0.155 between
    corrections; the old drop-at-once gate flapped LANES<->SPINE and parked
    the fleet in the capacity-1 SPINE fallback. Between LANE_SIGMA_MAX and
    LANE_SIGMA_EXIT the mode must be sticky; leaving needs >= LANE_SIGMA_EXIT
    sustained LANES_EXIT_HOLD_S; a ghost near the band still drops at once."""
    gate = {"ok_since": None}
    me = _ok_state(sigma=0.05)
    traffic.select_mode(GRID, me, {}, [], 0.0, gate)
    assert traffic.select_mode(GRID, me, {}, [], 3.0, gate) == "LANES"
    # the measured mid-correction excursion: sticky, no flap
    me.loc_sigma_lat = 0.155
    assert traffic.select_mode(GRID, me, {}, [], 3.1, gate) == "LANES"
    assert traffic.select_mode(GRID, me, {}, [], 30.0, gate) == "LANES"
    # at/above the exit threshold: leaves only after LANES_EXIT_HOLD_S
    me.loc_sigma_lat = traffic.LANE_SIGMA_EXIT
    assert traffic.select_mode(GRID, me, {}, [], 31.0, gate) == "LANES"
    assert traffic.select_mode(GRID, me, {}, [], 35.9, gate) == "LANES"
    assert traffic.select_mode(GRID, me, {}, [], 36.0, gate) == "SPINE"
    # recovery re-enters through the 3 s enter gate
    me.loc_sigma_lat = 0.05
    assert traffic.select_mode(GRID, me, {}, [], 36.1, gate) == "SPINE"
    assert traffic.select_mode(GRID, me, {}, [], 39.1, gate) == "LANES"
    # a brief exit-level excursion that recovers never drops the mode
    me.loc_sigma_lat = 0.31
    assert traffic.select_mode(GRID, me, {}, [], 40.0, gate) == "LANES"
    me.loc_sigma_lat = 0.06
    assert traffic.select_mode(GRID, me, {}, [], 42.0, gate) == "LANES"
    me.loc_sigma_lat = 0.31
    assert traffic.select_mode(GRID, me, {}, [], 44.0, gate) == "LANES"
    # ghost near the band. Road model (shipped): a SUSPECT peer is usually a
    # late state, bounded by the envelope's stale_reach - a SUSTAINED exit
    # reason like high sigma, never an instant geometry flip.
    gx = GRID.cell_to_world((60, 60))
    me.loc_sigma_lat = 0.06
    assert traffic.select_mode(GRID, me, {}, [gx], 44.1, gate) == "LANES"
    assert traffic.select_mode(GRID, me, {}, [gx], 48.0, gate) == "LANES"
    assert traffic.select_mode(GRID, me, {}, [gx], 49.2, gate) == "SPINE"
    # Legacy reservations: the ghost still drops the mode AT ONCE.
    legacy = GridMap.from_yaml(str(YAML))
    legacy.traffic.reservations = True
    lg = {"ok_since": None}
    traffic.select_mode(legacy, me, {}, [], 0.0, lg)
    assert traffic.select_mode(legacy, me, {}, [], 3.0, lg) == "LANES"
    assert traffic.select_mode(legacy, me, {}, [gx], 3.1, lg) == "SPINE"


def test_gate_requires_every_peer_near_the_band_to_localize_well():
    gate = {"ok_since": None}
    me = _ok_state(sigma=0.05)
    bad = _ok_state("robot_2", (60, 60), sigma=0.2)   # in the band
    for t in (0.0, 3.5):
        assert traffic.select_mode(GRID, me, {"robot_2": bad}, [],
                                   t, gate) == "SPINE"
    bad.loc_sigma_lat = 0.05
    traffic.select_mode(GRID, me, {"robot_2": bad}, [], 4.0, gate)
    assert traffic.select_mode(GRID, me, {"robot_2": bad}, [],
                               7.0, gate) == "LANES"


def test_gate_drops_on_a_ghost_near_the_band_and_without_lane_geometry():
    gate = {"ok_since": None}
    me = _ok_state(sigma=0.05)
    gx = GRID.cell_to_world((60, 60))
    assert traffic.select_mode(GRID, me, {}, [gx], 10.0, gate) == "SPINE"
    # No parsed band, or no planner support: SPINE regardless.
    bare = GridMap(["." * 20] * 20, 0.1, (0.0, 0.0), {})
    assert traffic.select_mode(bare, me, {}, [], 10.0,
                               {"ok_since": None}) == "SPINE"
    assert traffic.select_mode(GRID, me, {}, [], 10.0, {"ok_since": None},
                               lanes_supported=False) == "SPINE"


# --------------------------------------------------- mixed-mode occupancy
def test_robot_inside_spine_rect_counts_as_occupant_whatever_its_mode():
    """A LANES-mode robot never lists SPINE in its intent, but its body in
    the band must still block a SPINE-mode entrant (merged spec 5)."""
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world((30, 35)), 0.0)
    lane_robot = _ok_state("robot_2", (60, 63))       # NB lane, no zones
    c.on_peer_state(lane_robot, NOW)
    assert "robot_2" in c._required_granters("SPINE_M", NOW)
    occ = traffic.zone_occupants(GRID, "SPINE_M", c._peer_positions(NOW))
    assert "robot_2" in occ
    # ...and the robot itself answers SPINE-segment requests as a needer.
    c2 = FleetCoordinator("robot_2", GRID, lambda d: None, lambda d: None)
    c2.state.pose = Pose2D(*GRID.cell_to_world((60, 63)), math.pi / 2)
    assert "SPINE_M" in c2._inside_zones()


# ------------------------------------------- coordinator wiring in LANES
def _lanes_coordinator(cell):
    c = FleetCoordinator("robot_1", GRID, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*GRID.cell_to_world(cell), math.pi / 2)
    c._eff_mode = traffic.LANES_MODE          # gate already passed
    return c


def test_lanes_skips_the_wait_bay_and_the_spine_mutex():
    c = _lanes_coordinator((10, 30))
    assert c.set_goal(L1, now=NOW)
    bays = set(GRID.traffic.wait_bays.values())
    assert not any(cell in bays for cell in c.path), "no bay detour in LANES"
    req = traffic.required_zones(GRID, c.path, traffic.LANES_MODE)
    assert not any(z.startswith("SPINE") for z in req), req
    # SP_NW/SP_NE are SPINE-fallback-only since the R10 fix round: as
    # always-on mutexes each spur made half the two-abreast north aisle
    # single-file and a head-on across the two spurs could never resolve.
    # In LANES the junction J_N keeps its capacity-1 rank.
    assert "J_N" in set(req)
    assert "SP_NW" not in set(req)


def test_lanes_acquisition_point_is_in_lane_short_of_the_zone():
    c = _lanes_coordinator((10, 30))
    assert c.set_goal(L1, now=NOW)
    acq = traffic.acquisition_point(GRID, c.path, 0, (), traffic.LANES_MODE)
    k = traffic.first_new_zone_index(GRID, c.path, 0, (), traffic.LANES_MODE)
    assert acq is not None and k is not None
    assert GRID.zones["J_N"].contains(c.path[k])
    r, col = c.path[acq]
    assert GRID.lanes.in_band(r, col), "acquisition point stays in-lane"
    assert traffic.zones_at(GRID, c.path[acq], traffic.LANES_MODE) == []
    assert traffic.path_distance_m(GRID, c.path, acq, k) \
        >= traffic.ACQ_STANDOFF_M - 1e-9


def test_sb_through_traffic_requires_j_aw():
    """The SB lane (cols 55-57) crosses J_AW (rows 62-77, cols 52-59)."""
    sb = astar.astar(GRID, (88, 60), (22, 60), mode="LANES")
    req = traffic.required_zones(GRID, sb, traffic.LANES_MODE)
    assert "J_AW" in req
    assert not any(z.startswith("SPINE") for z in req), req
