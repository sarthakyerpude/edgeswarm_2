"""Strict directional lanes - OWN-LANE CONFINEMENT of recovery behaviours
and MOVE-WHEN-CLEAR (no wait-for edges for opposing-band bystanders).

THE RULES under test: robots use only the lane built for their direction;
recovery candidates (_find_refuge, _find_make_way_target, retreat targets,
wait bays, pockets) must be in a same-direction band, off-road (the south
open floor), or inside a crossing zone - NEVER an opposing band cell. The
bad-sigma opposing-pass yield shape: the lower robot reverses ALONG ITS OWN
LANE to the nearest pocket/junction/turnaround and holds. A stationary
opposing-band peer off my path creates no waiting_for and no deadlock
participation, while the safety envelope keeps seeing everyone.

Synthetic two-lane vertical corridor, planner contract functions stubbed as
instance attributes (independent of the planner agent's gridmap internals):

    rows 0/29 and cols 0/23 walls; free cols 1..22
    lane band rows 4..28: SB = cols 1..11, NB = cols 12..22
    rows 1..3  = off-road (south open floor)
    rows 25..28 = crossing zone (turnaround box; band still reported)
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from amr_fleet.core import traffic
from amr_fleet.core.coordinator import FleetCoordinator, STUCK_AFTER_S
from amr_fleet.core.gridmap import GridMap, TrafficConfig
from amr_fleet.core.models import Intent, MOVING, Pose2D, RobotState

NOW = 700.0
H, W = 30, 24
CROSS_R0, CROSS_R1 = 25, 28


def make_grid(reservations=False, pockets=((5, 16), (12, 10)),
              band_map=True):
    occ = []
    for r in range(H):
        if r in (0, H - 1):
            occ.append("#" * W)
        else:
            occ.append("#" + "." * (W - 2) + "#")
    g = GridMap(occ, 0.1, (0.0, 0.0), {},
                traffic=TrafficConfig(mode="SPINE",
                                      reservations=reservations))
    g.pockets = [tuple(p) for p in pockets]

    def in_crossing_zone(row, col):
        return CROSS_R0 <= row <= CROSS_R1 and 1 <= col <= W - 2

    def lane_band_at(row, col):
        if not (4 <= row <= H - 2 and 1 <= col <= W - 2):
            return None                     # rows 1..3: south open floor
        return ("corr", "NB" if col >= 12 else "SB")

    g.in_crossing_zone = in_crossing_zone
    g.lane_band_at = lane_band_at if band_map else None
    return g


def _coord(grid, rid="robot_5", cell=(12, 16), theta=math.pi / 2):
    c = FleetCoordinator(rid, grid, lambda d: None, lambda d: None)
    c.state.pose = Pose2D(*grid.cell_to_world(cell), theta)
    c.state.loc_sigma_lat = 0.05
    return c


def _rs(grid, rid, cell, theta=-math.pi / 2, v=0.0, sigma=0.05,
        status="IDLE", waiting_for="", score=0.0, stamp=NOW, seq=1):
    st = RobotState(robot_id=rid, seq=seq, stamp=stamp,
                    pose=Pose2D(*grid.cell_to_world(cell), theta))
    st.v = v
    st.loc_sigma_lat = sigma
    st.status = status
    st.waiting_for = waiting_for
    st.priority_score = score
    st.intent = Intent()
    return st


# ------------------------------------------------------------ the predicates
def test_cell_lane_legal_semantics():
    g = make_grid()
    assert traffic.cell_lane_legal(g, (12, 16), "NB")       # my own lane
    assert not traffic.cell_lane_legal(g, (12, 7), "NB")    # opposing lane
    assert traffic.cell_lane_legal(g, (12, 7), "SB")
    assert not traffic.cell_lane_legal(g, (12, 16), "SB")
    assert traffic.cell_lane_legal(g, (2, 7), "NB")         # open floor
    assert traffic.cell_lane_legal(g, (26, 7), "NB")        # crossing zone


def test_confined_no_go_is_exactly_the_foreign_band_cells():
    g = make_grid()
    bad = traffic.confined_no_go(g, "NB")
    assert (12, 7) in bad and (20, 3) in bad
    assert (12, 16) not in bad                              # own lane
    assert (2, 7) not in bad                                # open floor
    assert (26, 7) not in bad                               # crossing zone
    # No band map (planner support absent/stubbed off): no confinement.
    assert traffic.confined_no_go(make_grid(band_map=False), "NB") == set()


def test_confinement_gating_by_mode_and_reservations():
    on = _coord(make_grid(reservations=False))
    assert on._lane_confinement_active()                    # road model
    off = _coord(make_grid(reservations=True))
    assert not off._lane_confinement_active()               # SPINE mutex law
    off._eff_mode = traffic.LANES_MODE
    assert off._lane_confinement_active()
    no_map = _coord(make_grid(reservations=False, band_map=False))
    assert not no_map._lane_confinement_active()
    assert no_map._recovery_no_go() == set()


# ------------------------------------------------- refuge / retreat targets
def test_find_refuge_never_picks_the_opposing_pocket():
    g = make_grid()                         # pockets: own (5,16), opp (12,10)
    c = _coord(g)                           # NB robot at (12,16)
    blocker = _rs(g, "robot_9", (24, 7), v=0.3, status="MOVING")
    assert c.registry.update(blocker, now=NOW)
    refuge = c._find_refuge(NOW, blockers=["robot_9"])
    assert refuge == (5, 16), (
        f"refuge {refuge}: the NEARER pocket (12,10) sits in the opposing "
        f"lane and must never be a candidate")
    assert traffic.cell_lane_legal(g, refuge, "NB")


def test_find_refuge_uses_the_opposing_pocket_only_without_confinement():
    g = make_grid(reservations=True)        # SPINE mutex law: no confinement
    c = _coord(g)
    blocker = _rs(g, "robot_9", (24, 7), v=0.3, status="MOVING")
    assert c.registry.update(blocker, now=NOW)
    refuge = c._find_refuge(NOW, blockers=["robot_9"])
    assert refuge == (12, 10), (
        "control: without lane confinement the nearest pocket wins - this "
        "pins that the confinement (not something else) moved the choice")


def test_yield_shape_reverses_along_own_lane_to_the_pocket_behind():
    """The bad-sigma opposing-pass case: the lower robot's retreat target is
    the pocket BEHIND it in its OWN lane, and the whole retreat route stays
    lane-legal (reverse along the own lane - never cut the opposing one)."""
    g = make_grid()
    c = _coord(g)                           # NB at (12,16), sigma forced bad
    c.state.loc_sigma_lat = 0.3
    blocker = _rs(g, "robot_9", (24, 7), v=0.3, sigma=0.3, status="MOVING")
    assert c.registry.update(blocker, now=NOW)
    assert c._start_retreat(["robot_9"], NOW)
    assert c._retreat is not None
    cell = c._retreat["cell"]
    assert cell == (5, 16)                  # own-lane pocket
    assert cell[0] < 12, "the NB yielder must REVERSE (south), not advance"
    assert c.path, "the retreat must drive through the ordinary planner"
    assert all(traffic.cell_lane_legal(g, tuple(p), "NB") for p in c.path), (
        f"retreat route leaves the own lane: {c.path}")
    assert c._retreat["lane_dir"] == "NB", (
        "the yield must capture the own-lane direction for reverse_active")


def test_reverse_active_lifecycle():
    """reverse_active (audit/webapp contract name): True only while a
    retreat/make-way DRIVE travels OPPOSITE the yield's own-lane direction
    in that own half or off-road; forward travel and wrong-way motion in
    the opposing half stay False (the flag never masks a violation)."""
    g = make_grid()
    c = _coord(g)                           # NB robot at (12,16)
    assert c.reverse_active is False        # default after __init__
    c.path = [(r, 16) for r in range(12, 4, -1)]
    c._retreat = dict(cell=(5, 16), saved_goal=None, blockers=[],
                      phase="DRIVE", t0=NOW, hold_t0=None,
                      lane_dir="NB", route_zones=[])
    # Reversing south along the own NB half (the spine_wedge robot_3 shape).
    c.state.pose = Pose2D(*g.cell_to_world((12, 16)), -math.pi / 2)
    c.tick(NOW)
    assert c.reverse_active is True
    # Forward travel during the same drive: not a reverse.
    c.state.pose = Pose2D(*g.cell_to_world((12, 16)), math.pi / 2)
    c.tick(NOW + 0.1)
    assert c.reverse_active is False
    # The same heading in the OPPOSING half is wrong-way driving: never
    # masked by the flag.
    c.state.pose = Pose2D(*g.cell_to_world((12, 7)), -math.pi / 2)
    c.tick(NOW + 0.2)
    assert c.reverse_active is False
    # Off-road (south open floor) reverse during the drive still counts.
    c.state.pose = Pose2D(*g.cell_to_world((2, 16)), -math.pi / 2)
    c.tick(NOW + 0.3)
    assert c.reverse_active is True
    # Yield over: reset per tick, like uturn_active.
    c._retreat = None
    c.goal_cell = None
    c.path = []
    c.state.pose = Pose2D(*g.cell_to_world((12, 16)), -math.pi / 2)
    c.tick(NOW + 0.4)
    assert c.reverse_active is False


def test_find_refuge_never_parks_in_a_crossing_zone_or_the_opposing_lane():
    """No own-lane pocket reachable: the fallback refuge must still be
    lane-parkable - crossing zones (junction boxes / turnarounds) are for
    moving THROUGH, never for parking, and the opposing lane stays barred.
    (The planner integration measured _find_refuge parking at (65,65) INSIDE
    the aisle_a x spine junction box - this pins the rejection.)"""
    g = make_grid(pockets=((12, 10),))      # ONLY an opposing-lane pocket
    c = _coord(g)
    blocker = _rs(g, "robot_9", (22, 4), v=0.3, status="MOVING")
    assert c.registry.update(blocker, now=NOW)
    refuge = c._find_refuge(NOW, blockers=["robot_9"])
    assert refuge is not None
    assert refuge != (12, 10), "the opposing pocket is unreachable by rule"
    assert not traffic.in_crossing(g, refuge), (
        f"refuge {refuge} parks inside a crossing zone")
    assert traffic.cell_lane_parkable(g, refuge, "NB",
                                      c._pocket_grade_cells())


def test_designated_cells_are_exempt_but_unreachable_across_the_divider():
    """The designated tier-1 cells are re-admitted from the no-go set (the
    layout engineered them per direction) - but an opposing-side one stays
    out of reach because the ROUTE to it would cut the opposing lane."""
    g = make_grid()
    c = _coord(g)
    assert (12, 10) not in c._lane_no_go(), "designated cells are exempt"
    assert traffic.cell_lane_parkable(g, (12, 10), "NB",
                                      c._pocket_grade_cells())
    blocker = _rs(g, "robot_9", (24, 7), v=0.3, status="MOVING")
    assert c.registry.update(blocker, now=NOW)
    assert c._find_refuge(NOW, blockers=["robot_9"]) == (5, 16), (
        "every approach to the opposing-side pocket crosses the opposing "
        "lane, so the own-side pocket must still win")


# --------------------------------------------------------- make-way targets
def test_make_way_target_is_lane_legal():
    g = make_grid()
    c = _coord(g)
    ben = _rs(g, "robot_1", (20, 7), v=0.0, status="WAITING",
              waiting_for="robot_5", score=4000.0)
    assert c.registry.update(ben, now=NOW)
    t = c._find_make_way_target(ben, NOW)
    assert t is not None
    assert traffic.cell_lane_legal(g, t, "NB"), (
        f"make-way target {t} is an opposing-band cell")
    assert t not in traffic.confined_no_go(g, "NB")


# ------------------------------------------------------- move-when-clear (3)
def test_stationary_opposing_peer_creates_no_edge_and_no_deadlock_entry():
    """A parked opposing-band peer 0.9 m abeam, wholly off my path: the
    robot keeps its GO, gains NO waiting_for edge, and the deadlock layer
    sees nothing - while the safety envelope still lists the peer."""
    g = make_grid()
    c = _coord(g)                           # NB at (12,16)
    c.goal_cell = (22, 16)
    c.path = [(r, 16) for r in range(12, 23)]
    t, permit = NOW, None
    for k in range(13):                     # 6 s with the pose frozen
        t = NOW + 0.5 * k
        peer = _rs(g, "robot_7", (12, 7), v=0.0, stamp=t, seq=k + 1)
        assert c.registry.update(peer, now=t)
        assert c._opposing_bystander("robot_7", t) is True
        permit = c.tick(t)
        assert c.state.waiting_for == "", (
            f"wait-for edge against an opposing-band bystander at t={t}: "
            f"{permit.reason}")
    assert permit.action == "GO"
    assert c.state.status == MOVING
    assert c.deadlock.events == [], "no deadlock participation"
    assert (t - NOW) > STUCK_AFTER_S, "sanity: the stuck watchdog window ran"
    # Never filtered out of the SAFETY envelope itself: it stays all-seeing.
    assert "robot_7" in {p.robot_id for p in c._safety_peers(t)}


def test_same_lane_stationary_peer_still_creates_the_edge():
    """Control: a parked peer IN MY LANE ON MY PATH is a real blocker - the
    envelope stops me and the wait-for edge forms as before."""
    g = make_grid()
    c = _coord(g)
    c.goal_cell = (22, 16)
    c.path = [(r, 16) for r in range(12, 23)]
    peer = _rs(g, "robot_8", (16, 16), v=0.0)
    assert c.registry.update(peer, now=NOW)
    assert c._opposing_bystander("robot_8", NOW) is False
    permit = c.tick(NOW)
    assert permit.action == "STOP"
    assert permit.blocking_robot == "robot_8"
    assert c.state.waiting_for == "robot_8"
