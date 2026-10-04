"""
FleetCoordinator - the decision engine.

This class owns the per-tick sequence and produces a Permit. It imports only
other core modules: no rclpy, no ROS messages, no hardware. That is what lets
test/ exercise the complete coordination logic with plain pytest, and what
would let the same code run on a Jetson with no ROS installed.

TICK SEQUENCE
  1. expire stale map data
  2. recompute my own priority score
  3. detect conflicts against peer intents
  4. request / check the zone I need next
  5. run deadlock detection
  6. decide: GO / SLOW / STOP / YIELD / REROUTE
"""
import inspect
import math
from typing import Dict, List, Optional, Sequence, Set

from . import astar, peers as peers_mod, priority as priolib, safety, traffic
from .conflict import ConflictDetector
from .deadlock import DeadlockManager
from .gridmap import GridMap
from .models import (Conflict, Intent, Permit, Pose2D, RobotState, Task,
                     FAULT, IDLE, MOVING, WAITING, YIELDING)
from .peers import DEAD, FRESH, SUSPECT, PeerRegistry
from .priority import priority_score
from .zone import FREE, HELD, REQUESTING, ZoneArbiter, ZoneTimeout

try:                                    # HITBOX builder's module; optional
    from . import hitbox as hitbox_mod
except Exception:                       # pragma: no cover
    hitbox_mod = None

# Does the planner understand lane mode yet? (astar.py is the HITBOX
# builder's file; until its lane step costs land, the LANES gate stays shut
# and the fleet runs the shipped SPINE mutex.)
_ASTAR_HAS_MODE = "mode" in inspect.signature(astar.astar).parameters
# Overtake plumbing: can astar soften the wrong-way term over a borrow set?
_ASTAR_HAS_CROSSING_EXTRA = ("crossing_extra"
                             in inspect.signature(astar.astar).parameters)


def _ang_norm(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))

# Distance ahead of a zone at which we begin negotiating. Must be far enough to
# stop comfortably: v^2/(2a) at 0.5 m/s and 0.5 m/s^2 is 0.25 m, plus reaction
# and a margin.
ZONE_REQUEST_DISTANCE_M = 2.5
ZONE_ENTRY_DISTANCE_M = 0.6
MIN_SPEED_SCALE = 0.25

# Stuck-with-GO watchdog: a GO/SLOW permit with no physical movement for this
# long, with an alive peer this close, is downgraded to STOP/waiting_for so
# the deadlock layer can see sensor-level standoffs (collision-monitor holds).
STUCK_AFTER_S = 3.0
STUCK_MIN_MOVE_M = 0.05
STUCK_BLOCKER_RADIUS_M = 1.2
# Stuck-with-GO SELF-recovery (webots_strictlanes1, F1): a robot wedged
# against a WALL/rack corner holds GO forever - no alive peer within the
# blocker radius, so the downgrade above never fires, the deadlock graph has
# no edge from it, and the fleet's stuck machinery is blind to it (robot_1
# at (0.46, 4.39): 1868 phantom victim elections by its follower). After
# STUCK_SELF_RECOVER_S of no movement under GO with NOBODY to blame, replan
# with the next STUCK_BLOCK_AHEAD_M of path hard-blocked (the turn-shaping /
# forward-hazard terms then produce a different approach geometry); if no
# such route exists, STOP with no blocker: status becomes WAITING, and the
# deadlock timeout elects *me* - an actionable victim whose retreat runs.
STUCK_SELF_RECOVER_S = 5.0
STUCK_BLOCK_AHEAD_M = 0.6
STUCK_RECOVER_COOLDOWN_S = 10.0
# PHANTOM HOLD (webots_live_final residual): the robot is held with GO, no
# peer to blame, and the BELIEVED nose ray is clear - but the TRUE pose is
# pressed near a face the collision monitor sees (belief-truth drift hit
# 0.28 m live; design point sigma 0.30). The believed map cannot explain the
# hold, so after this longer patience the same blocked-ahead replan runs
# anyway: shifting the plan one row/col away from wherever the unseen
# obstruction is breaks the stop-retry loop (the rack-3 EB and rack-6 NB
# wedge clusters, 7 wedges live). Longer than every legitimate zero-motion
# GO spell (segment spins ~2 s, SLOW-scaled 180s ~8 s) and shorter than the
# bridge's 2-abort BackUp cycle (~20 s).
STUCK_PHANTOM_S = 12.0
# Self-recovery precondition: the hold must be EXPLAINED by static geometry -
# a wall/rack within the collision monitor's forward reach (0.45 m StopZone
# + 0.20 m nose) of the robot's nose cone. Without it, a follower hiccup in
# open ground would trigger spurious replans (and the synthetic frozen-pose
# tests would fire it).
STUCK_STATIC_AHEAD_M = 0.65

# Street-style OVERTAKE of a stationary blocker (user: "due to 1 robot other
# robots are stuck"). The robot DIRECTLY behind a leader that has been
# physically immobile for T_OVERTAKE_S - while broadcasting a non-waiting
# status (GO-but-held / idle-parked: a 'parked car'; a WAITING/YIELDING
# leader is queued on something and will move) - may borrow the OPPOSING
# lane alongside and past it, gated by traffic.overtake_gate (oncoming
# clearance over the whole manoeuvre, occupancy, swept-rect static fit) and
# planned by A* with the borrow window softened (crossing_extra) and the
# blocker's inflated body hard-blocked. While engaged, overtake_active is
# the audit/webapp contract flag (lifecycle exactly like uturn_active).
# T_OVERTAKE_S sits above every normal pause (2 s station dwell, junction
# queueing is WAITING and exempt) and BELOW the 8 s deadlock timeout, so the
# GATED overtake always pre-empts an ungated victim reroute.
T_OVERTAKE_S = 7.0
# The envelope's stationary-peer rule stops a follower up to STATIONARY_
# PATH_M (2.5 m) of PATH before the parked body, so the trigger must see a
# blocker from that standoff: 2.5 + body + margin.
OVERTAKE_RANGE_M = 3.5          # only a blocker this close ahead on my path
OVERTAKE_BEHIND_M = 0.3         # the borrow window starts this far behind me
# Borrow past the blocker: the PLANNING clearance around the parked body is
# the measured REROUTE_BLOCKER_CLEAR_M disc (0.88 m - below it a diagonal
# pass leaves < the stationary-rule rect gap and the envelope freezes the
# passer right after it commits, measured both in webots_final and in this
# fix's first cut at 0.7 m lateral: gap 0.21 m < 0.26 m), plus the ~0.9 m
# diagonal merge back across the lane spacing - all INSIDE the window,
# because along-steps in the opposing half outside it stay planner-forbidden.
OVERTAKE_CLEAR_BEYOND_M = 2.0
OVERTAKE_RETRY_S = 1.5          # gate/trigger re-evaluation rate limit
OVERTAKE_MAX_S = 25.0           # engage-to-commit bound (abort pre-commit)
OVERTAKE_COOLDOWN_S = 5.0       # per-blocker cooldown after abort/completion
# Rule 6 on the deadlock-victim REROUTE: a desperate reroute may never DRIVE
# ALONG the opposing lane (the planner's wrong-way toll is huge but finite,
# and with no legal alternative A* pays it - an UNGATED pass, which only the
# gated overtake may perform). Enforced as a post-plan check on the rerouted
# path, NOT by hard-blocking the opposing cells: a blanket block also forbade
# the rule-3 LATERAL crossings through the opposing half (always legal, how a
# band stays escapable) and measurably cost deliveries (seed 7: 20 -> 16).
YIELD_REROUTE_LANE_CONFINED = True
# A zone HELD but never physically entered is handed back after this long
# whenever peers are waiting on it (never-entered holds had no timeout at all).
ZONE_HOLD_LEASE_S = 12.0
# After this many CONSECUTIVE replan failures (no successful plan in between)
# the goal is treated as unreachable-for-now: the robot keeps the empty path
# (it holds under 'active goal has no valid route') and releases every ranked
# claim it is not physically inside, so a robot that cannot move can never
# pin a mover through stale zone claims (webots_final.log: robot_3's claims
# on SPINE_N outlived ~10 REPLAN FAILED while robot_2 waited on them).
REPLAN_FAIL_RELEASE_K = 3
# Inside-beats-stuck tiebreak (webots_final.log corridor wedge): a robot that
# HOLDS its next ranked segment and has stood at the acquisition point this
# long may ignore, in the entry-occupancy check only, a STOPPED peer whose
# believed CENTRE is outside that segment and inside territory I already hold
# or stand in (a queued/stuck neighbour whose K_SIG*sigma-inflated hitbox
# merely pokes over the boundary - at the SPINE-fallback sigma 0.2 that
# inflation is 0.4 m and wedged the holder forever). Mutual exclusion is
# unaffected (I hold the segment's mutex; the peer must acquire it from me
# before entering) and contact safety stays with the envelope, which keeps
# the full-sigma live-gap check. Bounded: applies only after the grace, so
# flowing traffic is untouched.
ACQ_INSIDE_TIEBREAK_S = 5.0
# Center-to-center clearance accounts for both AMR footprints (about 0.26 m
# circumscribed radius each) plus a small grid/localization margin.
PEER_OBSTACLE_CLEARANCE_M = 0.55
# Peers' lidars see MY body and broadcast it as a blockage. Cells within this
# radius of my own pose are never obstacles for me: if they were, A* rejects
# my start cell, every bid returns 'unreachable' and the robot idles forever.
SELF_CLEARANCE_M = 0.45
# Broadcast intent (current_intent): only the not-yet-driven suffix, capped so
# one Intent stays under ~1.4 kB (no IP fragmentation), re-timed from 'now'
# with the measured speed, because real speed is far below v_nominal.
INTENT_MAX_CELLS = 60
INTENT_MAX_HORIZON_S = 8.0
INTENT_MIN_SPEED_MPS = 0.15
SPEED_EMA_TAU_S = 2.0

# Deadlock-victim recovery. The warehouse north of the south corridor is a
# tree (one spine, dead-end spurs), so a victim can rarely route AROUND its
# blocker: a reroute only counts if the new path clears the blocker's body by
# REROUTE_BLOCKER_CLEAR_M and its next REROUTE_INTENT_HORIZON_S of path;
# otherwise the victim retreats to the nearest refuge cell that is off every
# peer's path, holds there until the blocker has passed, then resumes.
# 0.8 with the disc envelope; the rect envelope's stationary-peer rule stops
# at rect gap < ~0.39 m, and a diagonal pass at 0.8 m centre clearance leaves
# only ~0.24 m of true rect gap (measured) - the victim then froze under
# STOP right after 'rerouting'. 2*HB_R_CIRC (0.572) + the maximum stationary
# margin (0.391) needs ~0.97 m of centre clearance for a DIAGONAL pass.
# 0.97 was briefly shipped, but it exceeds the ~0.90 m of lateral clearance
# a 1.6 m aisle can physically offer (pocket row to the far wall strip), so
# EVERY in-aisle pass of a parked/yielding robot degraded to the retreat
# yo-yo (measured: passing_parked and all three aisle head-ons never
# completed; at exactly 0.90 the single-row far-wall corridor is a float
# knife-edge). A PARALLEL in-aisle pass needs 0.41 + 0.39 rect margin
# (0.46 with hysteresis) ~= 0.87 m of centre clearance; diagonal geometries
# that squeak through and still get enveloped are no longer frozen - the
# stationary-escape rule lets the robot drive on or the stale-reroute check
# turns it into a retreat within one window. 0.88 = the diagonal worst case
# against the stationary (v_close-free) threshold, 2*HB_R_CIRC 0.572 + 0.26
# + heading-sweep margin, and still below the 0.90 m a 1.6 m aisle offers.
REROUTE_BLOCKER_CLEAR_M = 0.88
REROUTE_INTENT_HORIZON_S = 3.0
REROUTE_INTENT_CLEAR_M = 0.45
RETREAT_SEARCH_M = 6.0
RETREAT_PEER_CLEAR_M = 0.9        # refuge distance from any peer's pose
RETREAT_INTENT_CLEAR_M = 0.95     # refuge distance from any peer's path
                                  # (> safety D_stop 0.67 + 2-sigma margin, so
                                  # the passing robot is not stopped by me)
RETREAT_PASS_CLEARANCE_M = 0.22   # static clearance to drive through a cell
RETREAT_CELL_CLEARANCE_M = 0.30   # static clearance to park in a cell
RETREAT_PEER_BODY_M = 0.55        # never search/plan through a peer's body
RETREAT_SPEED_SCALE = 0.6
RETREAT_MAX_DRIVE_S = 15.0
RETREAT_HOLD_MIN_S = 2.0
# Was 30.0: AC8 bounds every yield hold at 20 s, and the measured aisle-A-east
# livelock was serialized by 30.1 s holds that expired straight back into the
# same standoff (the corridor-clear refuge rule below is the real fix; the
# shorter cap just stops a stale hold from freezing throughput).
RETREAT_HOLD_MAX_S = 20.0
RETREAT_RELEASE_DIST_M = 1.2      # blocker this far away and off my area...
DEADLOCK_COOLDOWN_S = 4.0         # ...then no re-victimisation for this long
# Designated retreat targets (yaml traffic: retreat_cells, wait_bays, refuges)
# are searched this far along drivable cells before the local BFS fallback.
RETREAT_DESIGNATED_SEARCH_M = 14.0
# A refuge closer than this to where I already stand clears nothing.
RETREAT_MIN_MOVE_M = 0.5

# Traffic layer (core/traffic.py, ranked zones). Requests start when the
# acquisition point is this close along the path; the robot slows inside
# ACQ_SLOW_DISTANCE_M and stops within ACQ_STOP_TOL_M of it.
ACQ_REQUEST_DISTANCE_M = ZONE_REQUEST_DISTANCE_M
ACQ_SLOW_DISTANCE_M = 1.0      # was 0.6, re-sized for the 1.0 m/s cruise
ACQ_STOP_TOL_M = 0.15
# A south entry is routed through a wait bay if that costs at most this much
# extra path; the bay is then the acquisition point.
WAIT_BAY_MAX_DETOUR_M = 6.0
BAY_TAKEN_M = 1.0
# Waiting at an acquisition point is bounded by the rank order, so the
# deadlock TIMEOUT trigger is off for it - unless it lasts this long. Was
# 60.0: with the old score-ordered, timeout-and-re-request arbiter a waiter
# could lose its queue slot forever, and 60.1 s became the fleet's fixed max
# wait on every seed. With the FIFO no-timeout queue (zone.py) an acquisition
# wait is bounded by queue transit, so anything past 20 s is a real fault and
# becomes deadlock-eligible.
ACQ_WAIT_TIMEOUT_S = 20.0
# Boot race (measured: 12.2 s SP_NE double-hold from two simultaneous cold
# starts): an EMPTY peer registry makes an empty granter quorum, which
# may_enter treats as satisfied. Until the first heartbeat is heard, a ranked
# zone is not entered for this long after the coordinator's first tick; a
# genuinely alone robot proceeds once the grace expires.
BOOT_QUORUM_GRACE_S = 2.0
# Release gate sigma scale: my own hitbox is released at 1*sigma inflation
# (traffic applies K_SIG=2 on top, so 0.5 * 2 = 1), while ENTRY occupancy
# keeps the peers' full 2*sigma check. The wiring report named this knob for
# AC12's 1.5 s junction-restart bound (measured 5.0 s at forced sigma 0.2).
RELEASE_SIGMA_SCALE = 0.5
# No wait-for edge for an acquisition request younger than this (one grant
# round trip plus margin; deadlock.T_MIN_CYCLE_WAIT_S is 1.0 s too).
ACQ_EDGE_DELAY_S = 1.0
# The nearest-path-index can jump past a wait bay (the route doubles back
# beside it), so the bay counts as reached within this radius, and the
# robot keeps it as its acquisition point until it has acquired everything
# (sticky until the zones are acquired or the route is replanned).
BAY_REACHED_M = 0.5
# Path progress is tracked forward-only, like the follower's (Nav2 RPP
# prunes the plan behind the robot): the path index is the nearest cell in
# [last, last + AHEAD] cells, and jumps elsewhere (backwards included) only
# if some other cell is PATH_INDEX_JUMP_M closer. A global nearest search
# flips between the two legs of a route that doubles back 0.1 m beside
# itself (the wait-bay loop at the spine mouth); every flip onto the
# outbound leg moved the acquisition point 1-2 m ahead, gave GO, and
# ratcheted a robot that was still REQUESTING SPINE into it (fleet_sim seed
# 7 with 0.1 m loc noise; seed 21 with a backward-tolerant window).
PATH_INDEX_AHEAD = 12
PATH_INDEX_JUMP_M = 0.3
# A deadlock reroute must leave the victim able to move: its acquisition
# point (if it still needs zones) must be at least this far ahead.
REROUTE_MIN_FREE_RUN_M = 0.3
# A victim re-elected within this window without having moved this far since
# its last reroute gets no second reroute: it retreats (the reroute did not
# resolve anything, e.g. the envelope kept it stopped).
REROUTE_RETRY_WINDOW_S = 20.0
REROUTE_MIN_PROGRESS_M = 0.3

# ---------------------------------------------------------------------------
# Increments 3+4 (R1 priority / right-of-way / make-way, R3/R7 lanes).
# Strict partition: after this long with NO state from a peer, stop whenever
# its reach (frozen pose + v_max*age, capped) touches my next 1.5 m of path.
T_PART_S = 2.0
PART_LOOKAHEAD_M = 1.5
PART_BODY_M = 0.55                # two bodies' worth of standoff
T_SILENT_S = getattr(peers_mod, "T_SILENT_S", 5.0)
T_GONE_S = getattr(peers_mod, "T_GONE_S", 30.0)
# Goal queue: a peer this close to my goal makes me queue off it.
GOALQ_NEAR_M = 0.6
GOALQ_SLOW_M = 1.4
GOALQ_STOP_M = 1.0
# Give-way (hold-short): a HIGHER-priority peer's corridor (body + next
# ROW_PEER_LOOK_M of intent) crossing my next ROW_MY_LOOK_M within
# ROW_CORRIDOR_M at a heading difference above GW_HEADING_RAD stops me
# ROW_HOLD_STANDOFF_M short. Same-direction is car-following: envelope's job.
ROW_CORRIDOR_M = 0.75
ROW_PEER_LOOK_M = 3.0
ROW_MY_LOOK_M = 2.5
ROW_HOLD_STANDOFF_M = 0.35
GW_HEADING_RAD = 0.7853981634
# Hold short only for an IMMINENT pass: the mover's body within this far
# (along its own route) of the crossing. Holding for a crosser still 3 m
# out bred 8 s single-member timeouts and 2-cycles (measured: seeds 1/13
# gave 10 deliveries with it, 15-17 without give-way entirely).
GW_PEER_NEAR_M = 1.5
# Make-way (R1 'make space / take some steps back').
MW_TRIGGER_WAIT_S = 0.5
MW_MAX_M = 3.0
MW_FIRST_LEG_M = 0.6              # first leg never approaches a higher robot
MW_CLEAR_CORRIDOR_M = 0.9         # target clear of higher corridors BY DISTANCE
# A DESIGNATED pocket/bay/retreat cell is the layout's engineered two-abreast
# spot: in a 1.6 m aisle NOTHING is 0.9 m clear of an in-aisle corridor, so
# the make-way search against a mid-aisle blocker was structurally empty
# (measured: 'no valid target within 3.0m' on every attempt, AC8 20 % failed
# searches). A designated cell only needs the two-abreast lateral margin
# (0.45 m centre-to-centre > the 0.35 m rect-gap stop threshold at sigma 0.08).
MW_CLEAR_POCKET_M = 0.45
MW_GOAL_CLEAR_M = 1.0             # ...and of their goals (measured J_N livelock)
MW_PEER_BODY_M = 0.7
MW_DRIVE_CLEAR_M = 0.22
MW_PARK_CLEAR_M = 0.30
MW_HOLD_MIN_S = 0.5
MW_HOLD_MAX_S = 20.0
MW_RETRY_S = 1.0
MW_FAILS_TO_DEADLOCK = 3
MW_INVERT_S = 6.0                 # inversion: the HIGHER robot makes way
MW_COOLDOWN_S = 4.0
MW_BENEF_STATIONARY_S = 5.0       # stale-beneficiary release
MW_HELD_MIN_FOR_STALE_S = 2.0
# sih-57 live stall: the yielder parked 'clear' by its own pocket margin
# (MW_CLEAR_POCKET_M = 0.45) but INSIDE the beneficiary's stationary-peer
# safety rule, so the beneficiary froze on it and the stale-beneficiary
# release could not fire (st.waiting_for == me). If the beneficiary is
# stationary AND broadcasting waiting_for == me for this long during my HOLD,
# my spot is still in its way: re-target farther at once (with the
# beneficiary's stationary-rule clearance) instead of holding to the
# MW_HOLD_MAX_S = 20 s cap; with no farther target, end the hold as a failed
# make-way (resuming makes me a mover, which the stationary rule ignores).
MW_BLOCKED_RETARGET_S = 1.5
MW_AHEAD_COST = 10                # BFS rings: prefer lateral/backward targets
# Anti-flip: the first decision for a pair is frozen until separation or age.
ENCOUNTER_FREEZE_M = 2.0
ENCOUNTER_FREEZE_S = 15.0
T_DISAGREE_S = 3.0                # mutual waiting_for -> robot_id decides
# LANES gate (traffic.select_mode owns the thresholds; re-exported for tests).
LANE_SIGMA_MAX = traffic.LANE_SIGMA_MAX
LANE_SIGMA_EXIT = getattr(traffic, "LANE_SIGMA_EXIT", 0.30)
LANES_HOLD_S = traffic.LANES_HOLD_S
LANES_EXIT_HOLD_S = getattr(traffic, "LANES_EXIT_HOLD_S", 5.0)
LANES_RADIUS_M = traffic.LANES_RADIUS_M
# Lane-pass eligibility (the envelope's lateral-only pass rule,
# safety.envelope_permit lane_pass): both robots inside the band, heading
# within LANE_PASS_HEADING_RAD of the corridor axis, lateral offset from the
# OWN keep-right lane centre within LANE_KEEP_TOL_M, neither spinning or
# about to turn. Anything else keeps the full omnidirectional envelope.
LANE_PASS_HEADING_RAD = 0.5235987756   # 30 deg
LANE_KEEP_TOL_M = 0.18                 # |lateral error| from my lane centre
# Convoy entry (R10 direction-aware segments, the 'path lock' fix): a ranked
# corridor/spur zone whose every occupant AND claimant is a FRESH peer
# heading my way (within CONVOY_HEADING_RAD of my path's direction through
# the zone) may be entered while my FIFO request is still queued - the
# envelope spaces the convoy like car-following. Junctions never convoy.
CONVOY_HEADING_RAD = 0.7853981634      # 45 deg
# Don't-block-the-box (road model A2): a junction (zone kind 'intersection')
# is entered only when my next BOX_EXIT_CLEAR_M of path BEYOND it is clear
# of stopped robot hitboxes - otherwise wait at the standoff and leave the
# cross-flow free.
BOX_EXIT_CLEAR_M = 0.6
# Box-junction right-of-way (user order): Task.priority, then near-station
# (believed distance to the broadcast goal), then emergency battery, then
# the existing deterministic total order (level L, robot_id).
NEAR_STATION_M = 2.0
BATTERY_EMERGENCY_PCT = 15.0

# Strict directional lanes (R-lanes). The U-turn gate (traffic.uturn_gate)
# is evaluated against the first direction REVERSAL within this many path
# cells; while it fails the robot waits IN ITS OWN LANE at the stand-off,
# and after UTURN_BLOCKED_REPLAN_S of continuous failure it replans with
# these turn cells blocked ('drive on to where the turn fits' - the user
# rule), through the ordinary replan machinery.
UTURN_LOOKAHEAD_CELLS = 40
UTURN_WINDOW_INTO_TARGET = 12
UTURN_BLOCKED_REPLAN_S = 6.0


class FleetCoordinator:
    def __init__(self, robot_id: str, gridmap: GridMap,
                 send_zone_request, send_zone_grant,
                 v_nominal: float = 1.0,
                 mode: str = "proposed",
                 logger=None,
                 weights: Optional[dict] = None,
                 conflict_kwargs: Optional[dict] = None):
        self.me = robot_id
        self.grid = gridmap
        self.v_nominal = v_nominal
        # mode="baseline" runs the stop-and-wait comparison system. Same
        # sensing, same planning, same control - ONLY the coordination block
        # changes. Changing exactly one variable is what makes the experiment
        # in docs/TEST_PLAN.md scientifically valid.
        self.mode = mode
        self.log = logger or (lambda m: None)
        self.weights = weights

        self.registry = PeerRegistry(robot_id, logger=self.log)
        self.detector = ConflictDetector(**(conflict_kwargs or {}))
        self.arbiter = ZoneArbiter(robot_id, send_zone_request,
                                   send_zone_grant, logger=self.log)
        self.deadlock = DeadlockManager(robot_id, logger=self.log)
        # Rect-envelope hysteresis state (core/safety.SafetyMemory): one per
        # coordinator, passed into envelope_permit every tick. None only if
        # an older safety.py without the class is on the path.
        _mem_cls = getattr(safety, "SafetyMemory", None)
        self._safety_mem = _mem_cls() if _mem_cls is not None else None

        self.state = RobotState(robot_id=robot_id)
        self.intent = Intent()
        self.path: List = []
        self.goal_cell = None
        self.current_task: Optional[Task] = None

        self.last_conflicts: List[Conflict] = []
        self.replan_count = 0
        self._wait_started: Optional[float] = None
        self._wait_xy = None
        self._zone_penalties: Dict[str, float] = {}
        self._zone_penalty_until: Dict[str, float] = {}
        self._last_now: float = 0.0
        # Stuck-with-GO watchdog + never-entered-zone hold lease (physical
        # truth: a permit must be reconciled with actual motion).
        self._progress_xy = None
        self._progress_t: Optional[float] = None
        self._last_base_action = ""
        self._held_since: Dict[str, float] = {}
        # Deadlock-victim retreat state (see _yield_permit / _retreat_permit).
        self._retreat: Optional[dict] = None
        self._deadlock_cooldown_until = 0.0
        # EMA of my measured |v| (seeded with v_nominal), for intent timing.
        self._speed_ema = v_nominal
        self._speed_ema_t: Optional[float] = None
        # Unknown-plan peers need every zone near them (peers.py).
        self.registry.zones_near = getattr(gridmap, "zones_within", None)
        # Traffic layer. traffic_mode is the REQUESTED mode from the yaml;
        # the EFFECTIVE mode (self._eff_mode) comes from traffic.select_mode
        # every tick and starts at the safe SPINE fallback.
        tcfg = getattr(gridmap, "traffic", None)
        self.traffic_mode = getattr(tcfg, "mode", traffic.SPINE_MODE)
        self._eff_mode = traffic.SPINE_MODE
        self._mode_gate: Dict[str, Optional[float]] = {"ok_since": None}
        # Set by the NODE's task-lifecycle code (never inferred from
        # goal_cell): None | 'PICKUP' | 'DROPOFF' | 'REPOSITION'.
        self.task_phase: Optional[str] = None
        self.task_t0: Optional[float] = None
        # Quantized priority L: comparisons use my LAST-BROADCAST value.
        self._cmp_L: float = 0.0
        self._starved = False
        self._starve_anchor = None
        # Give-way / make-way state.
        self._encounter: Dict[str, dict] = {}       # peer -> frozen decision
        self._mutual_since: Dict[str, float] = {}   # disagreement watchdog
        self._mw_cooldown_until: Dict[str, float] = {}
        self._mw_fails: Dict[str, int] = {}
        self._mw_last_try: Dict[str, float] = {}
        self.makeway_count = 0
        self.makeway_fail_count = 0
        # Peers already handed to arbiter.drop_peer for being GONE.
        self._gone_dropped: Set[str] = set()
        self._acq_wait_since: Optional[float] = None
        self._acq_waiting = False
        # Zones where the inside-beats-stuck entry tiebreak is engaged: it
        # stays engaged (no fresh ACQ_INSIDE_TIEBREAK_S grace) until the
        # zone's filtered occupancy clears naturally, so the first GO cannot
        # reset the wait timer and re-wedge the robot one tick later.
        self._tiebreak_zones: Set[str] = set()
        self._bay_passed = False            # reset on every replan
        # Convoy entries already logged (log once per engagement, not 10 Hz).
        self._convoy_logged: Set[str] = set()
        self._bay_reached = False           # reset on every replan
        self._bay_detour = False            # current path detours via a bay
        self._last_reroute = None           # (t, x, y) of the last reroute
        self._progress_key = None           # identifies the tracked path
        self._progress_i = 0                # tracked path index
        self._t_boot: Optional[float] = None    # first tick (boot-race grace)
        self._replan_fail_streak = 0        # consecutive replan failures
        # Strict directional lanes (R-lanes). uturn_active is READ by the
        # trajectory audit and the webapp - the name is contract, do not
        # rename. The confinement no-go sets are static geometry, cached per
        # travel direction (and per grid: cleared never - grids are loaded
        # once per process).
        self.uturn_active = False
        self._uturn: Optional[dict] = None      # the engaged turn's cells
        self._uturn_block_t0: Optional[float] = None
        self._lane_nogo_cache: Dict[str, Set[tuple]] = {}
        # Stuck-with-GO self-recovery rate limit.
        self._stuck_recover_at = 0.0
        # Street overtake of a stationary blocker. overtake_active is READ
        # by the trajectory audit and the webapp - exact-name contract like
        # uturn_active (reset every tick, re-asserted while the manoeuvre is
        # engaging or executing).
        self.overtake_active = False
        self._overtake: Optional[dict] = None   # blocker/borrow/avoid/band
        self._peer_still_since: Dict[str, float] = {}
        self._ot_cooldown_until: Dict[str, float] = {}
        self._ot_last_try = 0.0
        self.overtake_count = 0
        # reverse_active is READ by the trajectory audit and the webapp -
        # exact name is contract. True while a retreat/make-way DRIVE is a
        # reverse along the robot's OWN lane (see _mark_reverse); the audit
        # uses it to tell the rule-legal own-lane reverse (yield shape) from
        # wrong-way driving. Reset every tick like uturn_active.
        self.reverse_active = False

    # ------------------------------------------------------------ inbound
    def on_peer_state(self, st: RobotState, now: float) -> None:
        previous = self.registry.peers.get(st.robot_id)
        restarted = (previous is not None and
                     st.boot_id != previous.boot_id)
        was_unavailable = (
            previous is not None and
            (not previous.alive or
             self.registry.health.get(st.robot_id) == DEAD))
        accepted = self.registry.update(st, now=now)
        if not accepted:
            return
        if restarted:
            # A restarted peer lost its in-memory zone state. Remove grants
            # from its previous process epoch so they cannot satisfy a new
            # acquisition after the peer rejoins.
            self.arbiter.drop_peer(st.robot_id)
        elif not st.alive:
            # Self-declared FAULT: its claims are reclaimable (zone.py rule:
            # drop only on boot change / FAULT / GONE). Zones its body still
            # occupies stay blocked by the occupancy rule.
            self.arbiter.drop_peer(st.robot_id)
        self._gone_dropped.discard(st.robot_id)
        if not st.alive and self.path and self._path_intersects_peer_obstacle():
            self.log(f"faulted peer {st.robot_id} blocks my route - replanning")
            self.replan(now=now)
        elif (was_unavailable and st.alive and self.goal_cell is not None and
              not self.path):
            self.log(f"recovered peer {st.robot_id}; retrying my blocked route")
            self.replan(now=now)

    def on_zone_request(self, msg: dict) -> None:
        zid = msg.get("zone_id", "")
        # Occupancy rule: a ranked zone my body is in is a zone I need.
        needed = zid in self.intent.zones or zid in self._inside_zones()
        # Pass the relevant-peer set so the arbiter can apply the ownership
        # check that keeps mutual exclusion safe. See zone.py::on_request.
        self.arbiter.on_request(
            msg, i_need_zone=needed,
            relevant_peers=self._required_granters(zid, self._last_now))

    def on_zone_grant(self, msg: dict) -> None:
        self.arbiter.on_grant(msg)

    def on_map_update(self, reporter: str, blocked, cleared,
                      confidence: float, expiry: float, now: float) -> None:
        blocked_before = self.grid.blocked_cells(now)
        if blocked:
            mine = self._self_cells()
            blocked = [c for c in blocked if c not in mine]
        self.grid.merge_remote_update(reporter, blocked, cleared,
                                      confidence, expiry, self.state.pose)
        reopened = bool(set(cleared) &
                        (blocked_before - self.grid.blocked_cells(now)))
        if self._path_invalidated(now):
            self.log("peer map update invalidated my path - replanning")
            self.replan(now=now)
        elif reopened and self.goal_cell is not None:
            # A recovered aisle may make a shorter route available even though
            # the current detour is still valid. Peers should converge back to
            # the new map after receiving the clearing evidence.
            self.log("peer map update cleared dynamic cells - replanning")
            self.replan(now=now)

    # -------------------------------------------------------------- planning
    def set_goal(self, goal_cell, now: Optional[float] = None) -> bool:
        if self._retreat is not None:
            # Mid-yield: finish clearing the way first, then head here.
            self._retreat["saved_goal"] = goal_cell
            return True
        self.goal_cell = goal_cell
        return self.replan(now=now)

    def clear_goal(self) -> None:
        if self._retreat is not None:
            # Task gone mid-yield: keep parking at the refuge, then idle there.
            self._retreat["saved_goal"] = None
            return
        self.goal_cell = None
        self.path = []
        self.intent = Intent()

    def replan(self, avoid_zones: Optional[Set[str]] = None,
               zone_penalty: float = 50.0, now: Optional[float] = None,
               extra_blocked: Optional[Set[tuple]] = None) -> bool:
        if self.goal_cell is None:
            self.path, self.intent = [], Intent()
            return False

        start = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        now = self._last_now if now is None else now

        blocked = self.grid.blocked_cells(now)
        blocked.update(self._peer_obstacle_cells())
        if extra_blocked:
            blocked.update(extra_blocked)
        if self._overtake is not None:
            # An engaged overtake keeps its shape across incidental replans:
            # the blocker's inflated body stays a hard obstacle and the
            # borrow window stays soft, so a mode flip or map update cannot
            # re-plan the robot straight back into the stationary leader.
            blocked.update(self._overtake["avoid"])
        blocked.difference_update(self._self_cells())

        extra = astar.congestion_costs(
            [p.intent for p in self.registry.alive().values()], now)
        for zid, pen in list(self._zone_penalties.items()):
            if self._zone_penalty_until.get(zid, float("inf")) < now:
                self._zone_penalties.pop(zid, None)
                self._zone_penalty_until.pop(zid, None)
                continue
            zone = self.grid.zones.get(zid)
            if zone:
                for c in zone.cells:
                    extra[c] = extra.get(c, 0.0) + pen
        for zid in (avoid_zones or set()):
            zone = self.grid.zones.get(zid)
            if zone:
                for c in zone.cells:
                    extra[c] = extra.get(c, 0.0) + zone_penalty

        kw = {"mode": self._eff_mode} if _ASTAR_HAS_MODE else {}
        if self._overtake is not None and _ASTAR_HAS_CROSSING_EXTRA:
            kw["crossing_extra"] = set(self._overtake["borrow"])
        path = astar.astar(self.grid, start, self.goal_cell,
                           blocked=blocked, extra_cost=extra, **kw)
        if not path:
            self.log(f"REPLAN FAILED {start} -> {self.goal_cell}")
            self.path, self.intent = [], Intent()
            self._replan_fail_streak += 1
            if self._replan_fail_streak == REPLAN_FAIL_RELEASE_K:
                # Goal unreachable-for-now: drop every ranked claim my body
                # is not inside, so my stale claims cannot pin a mover while
                # I hold with no route (the deferred queues flush to the
                # robots that CAN move). Inside holds are kept: traffic
                # rule 4, never release a segment my hitbox is still in.
                self._drop_stale_route_claims("persistent replan failure")
            return False

        self._replan_fail_streak = 0
        direct = path
        path = self._via_wait_bay(path, blocked, extra)
        self.path = path
        self._bay_passed = False
        self._bay_reached = False
        self._bay_detour = path is not direct
        self.intent = astar.build_intent(self.grid, path, self.v_nominal, now)
        # Intent.goal is on the wire and peers' make-way targets must clear
        # it (MW_GOAL_CLEAR_M): filled at EVERY replan (it was always zero).
        gx, gy = self.grid.cell_to_world(path[-1])
        self.intent.goal = Pose2D(gx, gy, 0.0)
        self.replan_count += 1
        return True

    def _via_wait_bay(self, path: List, blocked: Set[tuple],
                      extra: Dict[tuple, float]) -> List:
        """Route a south entry through a wait bay (its acquisition point).

        Applies when the path enters ranked zones from outside every ranked
        zone and I do not already hold all of them: the robot then waits for
        its zones off the lanes, at WB_W/WB_E, never on the spine axis where
        the robot leaving the spine would drive into it. The leg to the bay
        must itself enter no ranked zone. Cheapest bay within
        WAIT_BAY_MAX_DETOUR_M of extra path, else the path is unchanged.
        """
        if not self._reservations_on:
            return path             # ROAD MODEL: nothing to acquire, no bays
        if self._eff_mode == traffic.LANES_MODE:
            # LANES: the acquisition point for J_N/J_AW is in-lane,
            # ACQ_STANDOFF_M before the zone; the bay detour is SPINE-only.
            return path
        bays = list(getattr(getattr(self.grid, "traffic", None),
                            "wait_bays", {}).values())
        if not bays or len(path) < 2:
            return path
        mode = self._eff_mode
        required = traffic.required_zones(self.grid, path, mode)
        if not required or traffic.zones_at(self.grid, path[0], mode):
            return path
        # R10: the bay is the acquisition point for the FIRST window only
        # (the next segment(s)); deferred segments are acquired later at
        # in-corridor junction standoffs, never via a bay.
        window = traffic.zone_window(self.grid, path, mode)
        if all(self.arbiter.state.get(z) == HELD for z in window):
            return path
        k = traffic.first_new_zone_index(self.grid, path, 0, (), mode)
        if k is None or any(c in bays for c in path[:k]):
            return path
        base = astar.path_length_m(self.grid, path)
        best, best_len = None, base + WAIT_BAY_MAX_DETOUR_M
        for bay in bays:
            if bay in blocked or self._bay_taken(bay):
                continue
            leg1 = astar.astar(self.grid, path[0], bay,
                               blocked=blocked, extra_cost=extra)
            if not leg1 or traffic.required_zones(self.grid, leg1, mode):
                continue
            leg2 = astar.astar(self.grid, bay, path[-1],
                               blocked=blocked, extra_cost=extra)
            if not leg2:
                continue
            total = (astar.path_length_m(self.grid, leg1)
                     + astar.path_length_m(self.grid, leg2))
            if total <= best_len:
                best, best_len = leg1 + leg2[1:], total
        return best if best is not None else path

    def _bay_taken(self, bay) -> bool:
        """A wait bay holds one robot: taken if an alive peer stands within
        BAY_TAKEN_M of it or its broadcast route passes through it."""
        bx, by = self.grid.cell_to_world(bay)
        for rid, p in self.registry.alive().items():
            xy = self._peer_xy(rid, self._last_now)
            if xy and math.hypot(xy[0] - bx, xy[1] - by) <= BAY_TAKEN_M:
                return True
            if bay in p.intent.cells:
                return True
        return False

    def _peer_obstacle_cells(self) -> Set[tuple]:
        """Inflate faulted/dead peer poses by a conservative robot radius."""
        blocked = set()
        resolution = self.grid.resolution
        radius = PEER_OBSTACLE_CLEARANCE_M
        cells = int(radius / resolution) + 1
        for peer in self.registry.obstacles_from_dead():
            center = self.grid.world_to_cell(peer.pose.x, peer.pose.y)
            for dr in range(-cells, cells + 1):
                for dc in range(-cells, cells + 1):
                    if math.hypot(dr, dc) * resolution > radius:
                        continue
                    cell = (center[0] + dr, center[1] + dc)
                    if self.grid.is_static_free(cell):
                        blocked.add(cell)
        return blocked

    def _self_cells(self) -> Set[tuple]:
        """Cells covered by my own footprint (plus margin)."""
        cells = set()
        resolution = self.grid.resolution
        n = int(SELF_CLEARANCE_M / resolution) + 1
        center = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                if math.hypot(dr, dc) * resolution <= SELF_CLEARANCE_M:
                    cells.add((center[0] + dr, center[1] + dc))
        return cells

    def _path_intersects_peer_obstacle(self) -> bool:
        if not self.path:
            return False
        peer_cells = self._peer_obstacle_cells()
        return any(cell in peer_cells for cell in self.path)

    def _path_invalidated(self, now: Optional[float] = None) -> bool:
        if not self.path:
            return False
        blocked = self.grid.blocked_cells(self._last_now if now is None else now)
        return any(c in blocked for c in self.path)

    def distance_to_goal_m(self) -> float:
        if not self.path:
            return 0.0
        return astar.path_length_m(self.grid, self.path)

    def path_length_m_between(self, a, b) -> float:
        blocked = self.grid.blocked_cells(self._last_now)
        blocked.update(self._peer_obstacle_cells())
        blocked.difference_update(self._self_cells())
        p = astar.astar(self.grid, a, b, blocked=blocked)
        return astar.path_length_m(self.grid, p) if p else -1.0

    # ------------------------------------------------------------------ tick
    def tick(self, now: float) -> Permit:
        self._last_now = now
        if self._t_boot is None:
            self._t_boot = now
        self._acq_waiting = False           # set by _traffic_permit
        self.uturn_active = False           # re-asserted by _uturn_permit
        self.reverse_active = False         # re-asserted by _retreat_permit
        self.overtake_active = False        # re-asserted by _overtake_tick
        self._track_peer_motion(now)
        self._update_speed_ema(now)
        self.grid.expire(now)
        newly_dead = self.registry.tick(now)
        # ZONES SURVIVE SILENCE (sih-57 field report): a peer that merely
        # went quiet keeps every hold, grant and granter-role. Its zone
        # dependencies are dropped only once it is GONE (>= T_GONE_S) -
        # never at the registry's 1.5 s DEAD mark.
        for rid, seen in list(self.registry.last_seen.items()):
            if now - seen >= T_GONE_S:
                if rid not in self._gone_dropped:
                    self._gone_dropped.add(rid)
                    self.arbiter.drop_peer(rid)
                    self.log(f"peer {rid} GONE ({now - seen:.0f}s silent): "
                             f"zone dependencies released")
            else:
                self._gone_dropped.discard(rid)
        if newly_dead and self._path_intersects_peer_obstacle():
            self.log("failed peer blocks my route - replanning")
            self.replan(now=now)
        self._update_mode(now)

        if self.state.status == FAULT:
            self.arbiter.release_all()
            return Permit(action="STOP", speed_scale=0.0, reason="FAULT")

        # Reconcile arbiter state with the CURRENT plan every tick. The
        # traversal release below used to run only inside the GO branch, so a
        # stopped robot never released passed zones; and a replan that moved
        # the route leaks HELD/effective-owner zones forever without the
        # not-in-intent release (a replan always starts at the robot's own
        # cell, so a zone the robot is inside is always still in the intent).
        self._maybe_release_passed_zones()
        needed = set(self.intent.zones)
        inside = set(self._inside_zones())
        needed |= self._retreat_kept_zones(inside)
        for zid, st in list(self.arbiter.state.items()):
            if st != FREE and zid not in needed:
                if st == HELD and zid in inside:
                    continue        # traffic rule 4: never release while in it
                self.arbiter.release(zid)

        # Hold lease: a zone granted/HELD but never physically entered must
        # not be held forever while peers wait — the rear winner physically
        # blocked by a parked loser hands the aisle over so the loser can
        # drive through, then re-acquires (the livelock unbreaker).
        cur_cell = self.grid.world_to_cell(self.state.pose.x,
                                           self.state.pose.y)
        held_now = set(self.arbiter.held_zones())
        self._held_since = {z: t for z, t in self._held_since.items()
                            if z in held_now}
        for zid in held_now:
            if self.arbiter.state.get(zid) != HELD:
                continue            # backed off earlier in this loop
            zone = self.grid.zones.get(zid)
            if zone and cur_cell in zone.cells:
                self._held_since.pop(zid, None)
                continue
            t0 = self._held_since.setdefault(zid, now)
            if not (now - t0 > ZONE_HOLD_LEASE_S
                    and self.registry.peers_needing_zone(zid)):
                continue
            if traffic.is_ranked(self.grid, zid):
                # Ranked zones wait at acquisition points by design; only a
                # needer physically inside a zone I hold or request (an
                # occupant grandfathered in, which can never get past me) is
                # a real standoff. Back off ALL my un-entered ranked zones,
                # so re-acquisition restarts in rank order - and ALWAYS the
                # occupied ones, even below a zone I am committed inside:
                # keeping those was a measured livelock (head-on at L1, the
                # holder inside SPINE never released SP_NW to the robot
                # standing in it). Re-acquiring them later from inside the
                # higher zone is the logged RANK EXCEPTION path.
                if self._ranked_lease_standoff(zid, now):
                    self.log(f"hold-lease back-off {zid} (held "
                             f"{now - t0:.1f}s; a needer is inside my zones)")
                    self._back_off_ranked(inside)
                    occ_now = traffic.occupants_by_zone(
                        self.grid, self._peer_positions(now))
                    for z, st_z in list(self.arbiter.state.items()):
                        if (st_z == HELD and z not in inside
                                and traffic.is_ranked(self.grid, z)
                                and occ_now.get(z)):
                            self.log(f"hold-lease release {z} "
                                     f"(occupied by {sorted(occ_now[z])})")
                            self.arbiter.release(z)
                            self._held_since.pop(z, None)
                continue
            self.log(f"hold-lease release {zid} "
                     f"(held {now - t0:.1f}s without entering)")
            self.arbiter.release(zid)
            self._held_since.pop(zid, None)
            self._zone_penalties[zid] = 8.0
            self._zone_penalty_until[zid] = now + 30.0
            self.replan(now=now)

        # Request lease (inside-beats-stuck, webots_final.log pin): a ranked
        # claim stuck in REQUESTING also defers every later requester (FIFO),
        # so a claimant that cannot progress pins the robots that can. The
        # HELD lease above never covered it. Same standoff rule: a needer of
        # my requested zone physically inside a zone I hold or request can
        # never get past my claim, so after the lease I back ALL un-entered
        # ranked claims off (the release flushes my deferred queues - the
        # inside robot gets its grants). Bounded, not starvation: on the next
        # tick I re-request with a fresh lamport and queue BEHIND the robot
        # that was physically inside, which is exactly the right order.
        for zid, st_z in list(self.arbiter.state.items()):
            if st_z != REQUESTING or not traffic.is_ranked(self.grid, zid):
                continue
            if zid in inside:
                continue
            t0 = self.arbiter.req_time.get(zid, now)
            if now - t0 <= ZONE_HOLD_LEASE_S:
                continue
            if not self.registry.peers_needing_zone(zid):
                continue
            if self._ranked_lease_standoff(zid, now):
                self.log(f"request-lease back-off {zid} (requested "
                         f"{now - t0:.1f}s ago; a needer is inside my zones)")
                self._back_off_ranked(inside)
                break

        self._update_priority(now)
        peers = self.registry.alive()

        # Peer windows are re-timed on my clock at receipt, so compare them
        # with my own re-timed suffix, not self.intent (timed at replan).
        self.last_conflicts = self.detector.detect(
            self.state, self.current_intent(now), peers, now)

        # A task goal with no valid coordination route must never leave the
        # velocity gate permissive. This can happen immediately after a map
        # update or before a persistent route failure is returned to auction.
        # It applies to the baseline mode too: a comparison policy must not
        # turn a planning failure into permission to move.
        if self.goal_cell is not None and not self.path:
            # Inside-keeps-hold: hand back everything EXCEPT a HELD ranked
            # segment my hitbox is still physically in (releasing it invited
            # a peer into the ground my body occupies and churned the mutex;
            # traffic rule 4). Everything else is released so a route-less
            # robot can never pin a mover.
            self._drop_stale_route_claims("no valid route")
            permit = Permit(action="STOP", speed_scale=0.0,
                            reason="active goal has no valid route")
            self._track_waiting(permit, now)
            return permit

        if self.mode == "baseline":
            return self._baseline_permit(peers, now)

        if self._retreat is not None:
            # Yielding takes over before zone logic: the retreat route must
            # not acquire zones, and the yield must not re-trigger itself.
            permit = self._retreat_permit(now)
            if permit is not None:
                return permit

        permit = self._proposed_permit(peers, now)
        # Overtake lifecycle: progress / pre-commit abort / the contract flag.
        self._overtake_tick(now)

        # Stuck-with-GO watchdog: a GO/SLOW permit that produces no physical
        # movement while an alive peer sits within blocker radius means the
        # robot is held by the navigation safety layer (collision monitor)
        # behind that peer. Override to STOP with waiting_for set, so the
        # deadlock graph finally sees the edge. Anchor on the BASE permit
        # action (never on our own override), and release the override the
        # tick the blocker leaves or motion resumes.
        base_action = permit.action
        pose = self.state.pose
        if base_action in ("GO", "SLOW") and self.path:
            moved = (self._progress_xy is not None and
                     math.hypot(pose.x - self._progress_xy[0],
                                pose.y - self._progress_xy[1])
                     > STUCK_MIN_MOVE_M)
            if (self._progress_xy is None or moved
                    or self._last_base_action not in ("GO", "SLOW")):
                self._progress_xy = (pose.x, pose.y)
                self._progress_t = now
            elif now - self._progress_t > STUCK_AFTER_S:
                blocker, bdist = "", STUCK_BLOCKER_RADIUS_M
                for rid, p in self.registry.alive().items():
                    d = math.hypot(pose.x - p.pose.x, pose.y - p.pose.y)
                    # The collision monitor's stop boxes face the MOTION
                    # direction: a peer BEHIND me cannot be what holds my
                    # forward drive. Blaming it made a wedged/frozen leader
                    # name its own queued follower, go WAITING on it, and
                    # turn the pair into a fake 2-cycle - while the leader,
                    # now 'WAITING', also stopped qualifying as a parked car
                    # for the follower's overtake.
                    if ((p.pose.x - pose.x) * math.cos(pose.theta)
                            + (p.pose.y - pose.y)
                            * math.sin(pose.theta)) < 0.0:
                        continue
                    # Move-when-clear (strict lanes): a peer wholly in an
                    # opposing band off my path's cells is NOT a blocker -
                    # it must never gain a wait-for edge from me.
                    if d <= bdist and not self._opposing_bystander(rid, now):
                        blocker, bdist = rid, d
                if blocker:
                    permit = Permit(
                        action="STOP", speed_scale=0.0,
                        reason=f"no physical progress; blocked by {blocker}",
                        blocking_robot=blocker, zone_id=permit.zone_id)
                elif (now - self._progress_t > STUCK_SELF_RECOVER_S
                      and self.goal_cell is not None
                      and self._retreat is None
                      and (self._static_ahead_m() < STUCK_STATIC_AHEAD_M
                           or now - self._progress_t > STUCK_PHANTOM_S)):
                    # Seen obstruction (believed nose ray hits a face) after
                    # 5 s, or a PHANTOM hold (belief clear, physics stopped:
                    # belief-truth drift pressed the true body to a face the
                    # collision monitor sees) after 12 s.
                    permit = self._stuck_self_recover(now)
        else:
            self._progress_xy = None
            self._progress_t = None
        self._last_base_action = base_action

        # Street overtake: a STOP held by a long-stationary, non-waiting
        # leader in my lane may turn into a gated borrow of the opposing
        # lane (see the T_OVERTAKE_S block above).
        started = self._maybe_overtake(permit, now)
        if started is not None:
            permit = started

        # An acquisition-point wait is bounded by the rank order: keep cycle
        # detection, but not the no-cycle timeout (it would make every
        # robot queued at a wait bay a 'deadlock victim' after 8 s).
        if self._acq_waiting:
            if self._acq_wait_since is None:
                self._acq_wait_since = now
        elif self._acq_wait_since is not None:
            # The structural wait is over: a new wait (e.g. an envelope
            # stop) is timed from now, not from the start of the queueing.
            self._acq_wait_since = None
            self.deadlock.wait_start = None
        timeout_ok = ((self._acq_wait_since is None
                       or now - self._acq_wait_since > ACQ_WAIT_TIMEOUT_S)
                      # status still says WAITING from last tick, but this
                      # permit moves me: no timeout victim on the way out.
                      and permit.action not in ("GO", "SLOW"))
        dl = (self.deadlock.tick(self.state, peers, now,
                                 timeout_trigger=timeout_ok)
              if now >= self._deadlock_cooldown_until else None)
        if dl and dl.get("role") == "VICTIM":
            permit = self._yield_permit(dl, now)
        elif dl and dl.get("role") == "HOLD":
            permit.deadlock_detected = True
            permit.deadlock_cycle = dl.get("cycle", [])

        self._track_waiting(permit, now)
        return permit

    # ---------------------------------------------------------- priority
    @property
    def traffic_mode_effective(self) -> str:
        """'LANES' | 'SPINE' (contract: effective mode from
        traffic.select_mode; named this way because .mode already selects
        the proposed/baseline comparison system)."""
        return self._eff_mode

    def _update_mode(self, now: float) -> None:
        ghosts = []
        for rid, st in self.registry.peers.items():
            age = now - self.registry.last_seen.get(rid, st.rx_time)
            if age > self.registry.t_suspect:
                ghosts.append((st.pose.x, st.pose.y))
        mode = traffic.select_mode(self.grid, self.state,
                                   self.registry.alive(), ghosts, now,
                                   self._mode_gate,
                                   lanes_supported=_ASTAR_HAS_MODE)
        if mode != self._eff_mode:
            self.log(f"traffic mode {self._eff_mode} -> {mode}")
            self._eff_mode = mode
            if self.goal_cell is not None and self._retreat is None:
                self.replan(now=now)

    def _update_priority(self, now: float) -> None:
        """Broadcast priority = the quantized level L (R1/R5).

        Comparisons during THIS tick use the previously broadcast value
        (_cmp_L): both robots of a pair then compare the same two numbers,
        which is what makes the order a total one in practice (the continuous
        score's 0.271-vs-0.269 mutual HOLD came from each side using its own
        just-recomputed score against the other's broadcast one).
        """
        self._cmp_L = self.state.priority_score
        pose = self.state.pose
        # STARVED promotion: a continuous wait past STARVE_S, cleared only by
        # actually moving STARVE_CLEAR_M from where the promotion happened.
        if self._starved:
            ax, ay = self._starve_anchor
            if math.hypot(pose.x - ax, pose.y - ay) >= priolib.STARVE_CLEAR_M:
                self._starved = False
                self._starve_anchor = None
        elif self.state.waiting_time >= priolib.STARVE_S:
            self._starved = True
            self._starve_anchor = (pose.x, pose.y)

        ph = (self.task_phase or "").upper()
        if self._starved:
            cls = priolib.CLASS_STARVED
        elif ph in ("DROPOFF", "CARRYING"):
            cls = priolib.CLASS_CARRYING
        elif ph in ("PICKUP", "TO_PICKUP"):
            cls = priolib.CLASS_TO_PICKUP
        elif ph == "REPOSITION" or self.goal_cell is not None \
                or self._retreat is not None:
            cls = priolib.CLASS_REPOSITION
        else:
            cls = priolib.CLASS_IDLE

        urg = 0
        t = self.current_task
        if t is not None:
            dl = priolib.eff_deadline(t, now)
            created = getattr(t, "created_at", 0.0) or 0.0
            if math.isfinite(dl) and dl > created:
                age_frac = (now - created) / (dl - created)
            else:
                age_frac = 0.0
            urg = priolib.urgency(getattr(t, "priority", 0), age_frac)

        self.state.priority_score = priolib.level(
            cls, urg, self.state.waiting_time)

    def _peer_outranks_me(self, st: RobotState, now: float) -> bool:
        """st outranks me, with the encounter freeze (the first decision for
        a close pair is kept until separation/age) and the disagreement
        watchdog (mutual waiting_for for T_DISAGREE_S -> robot_id decides)."""
        rid = st.robot_id
        d = math.hypot(st.pose.x - self.state.pose.x,
                       st.pose.y - self.state.pose.y)
        fz = self._encounter.get(rid)
        if fz is not None and (now - fz["t0"] > ENCOUNTER_FREEZE_S
                               or d > ENCOUNTER_FREEZE_M):
            self._encounter.pop(rid, None)
            fz = None
        if fz is not None:
            result = fz["peer_wins"]
        else:
            result = priolib.outranks(st.priority_score, rid,
                                      self._cmp_L, self.me)
            if d <= ENCOUNTER_FREEZE_M:
                self._encounter[rid] = {"peer_wins": result, "t0": now}
        if st.waiting_for == self.me and self.state.waiting_for == rid:
            t0 = self._mutual_since.setdefault(rid, now)
            if now - t0 > T_DISAGREE_S:
                result = rid < self.me        # lower robot_id wins
        else:
            self._mutual_since.pop(rid, None)
        return result

    def _track_waiting(self, permit: Permit, now: float) -> None:
        if self._retreat is not None:
            # Yield/make-way status is owned by _mark_yielding: a GO while
            # driving to the refuge must not flip YIELDING back to MOVING
            # (a yielding robot must carry no wait-for edge either).
            return
        if permit.action in ("STOP", "YIELD"):
            if self._wait_started is None:
                self._wait_started = now
                self._wait_xy = (self.state.pose.x, self.state.pose.y)
            self.state.waiting_time = now - self._wait_started
            self.state.status = (YIELDING if permit.action == "YIELD" else WAITING)
            self.state.waiting_for = permit.blocking_robot or ""
        else:
            # A transient GO/REROUTE blip with NO real movement must not reset
            # the anti-starvation clock (a single-member deadlock 'resolution'
            # used to replan onto the same route and zero waiting_time every
            # 8 s, so the 15 s/20 s priority aging never engaged).
            pose = self.state.pose
            if (self._wait_started is not None and self.path
                    and self._wait_xy is not None
                    and math.hypot(pose.x - self._wait_xy[0],
                                   pose.y - self._wait_xy[1]) <= 0.15):
                self.state.waiting_time = now - self._wait_started
                return
            if self._wait_started is not None:
                self.deadlock.mark_resolved(now)
            self._wait_started = None
            self._wait_xy = None
            self.state.waiting_time = 0.0
            self.state.waiting_for = ""
            self.state.status = MOVING if self.path else IDLE

    # ------------------------------------------------------- the real system
    def _safety_peers(self, now: float) -> List[safety.PeerDisc]:
        out = []
        for rid, p in self.registry.alive().items():
            pose = self.registry.extrapolated_pose(rid, now) or p.pose
            out.append(safety.PeerDisc(
                robot_id=rid, x=pose.x, y=pose.y, theta=pose.theta,
                v=p.v, w=p.w, rx_time=p.rx_time,
                suspect=self.registry.freshness(rid, now) == SUSPECT,
                loc_sigma_lat=p.loc_sigma_lat))
        out.extend(self._ghost_discs(now))
        # STATE-DEAD but coordination-alive hole (measured stress contact at
        # the 1.0 m/s cruise): a peer whose robot_state has been lost for
        # longer than t_dead drops out of alive(), while its reliable zone
        # traffic keeps its TIER at FRESH (by design - heartbeat-only stalls
        # must not ghost its zone protocol), so silent_ghost_discs never
        # picks it up either and it is in NOBODY's envelope. Every peer with
        # a known pose but no fresh state must be a frozen ghost here.
        present = {p.robot_id for p in out}
        for rid, st in self.registry.peers.items():
            if rid in present or not st.alive:
                continue
            age = now - self.registry.last_seen.get(rid, st.rx_time)
            if age >= T_GONE_S:
                continue
            out.append(safety.PeerDisc(
                robot_id=rid, x=st.pose.x, y=st.pose.y, theta=st.pose.theta,
                v=0.0, w=0.0,
                rx_time=self.registry.last_seen.get(rid, st.rx_time),
                suspect=True, loc_sigma_lat=st.loc_sigma_lat,
                silent=True, v_last=getattr(st, "v", None)))
        return out

    def _ghost_discs(self, now: float) -> List[safety.PeerDisc]:
        """SILENT peers (T_SILENT_S..T_GONE_S quiet, alive flag still true)
        as frozen ghosts the envelope must respect: they may still be moving
        (the sih-57 contact at t=217.7 s). Uses safety.silent_ghost_discs
        when the HITBOX builder's version exists, else frozen suspect discs
        (the suspect inflation v_max*(now - rx_time) covers the reach)."""
        fn = getattr(safety, "silent_ghost_discs", None)
        if callable(fn):
            try:
                return list(fn(self.registry, now))
            except Exception:
                pass
        out = []
        for rid, st in self.registry.peers.items():
            if not st.alive:
                continue            # FAULT: a planner obstacle already
            age = now - self.registry.last_seen.get(rid, st.rx_time)
            if not (T_SILENT_S <= age < T_GONE_S):
                continue
            # rx_time clamped so the suspect inflation v_max*(now-rx_time)
            # equals the spec's capped reach (<= 1.0 m), not v_max*30 s.
            reach_age = min(age, 1.0 / safety.V_MAX_MPS)
            out.append(safety.PeerDisc(
                robot_id=rid, x=st.pose.x, y=st.pose.y, theta=st.pose.theta,
                v=0.0, w=0.0, rx_time=now - reach_age, suspect=True,
                loc_sigma_lat=max(0.2, float(st.loc_sigma_lat or 0.0)),
                silent=True))
        return out

    def _envelope(self, now: float) -> Optional[Permit]:
        """The safety envelope's verdict for my current path. Every permit
        that can move the robot must pass through most_restrictive with it.

        Wired to the wheel-inclusive rect envelope (safety.envelope_permit +
        this coordinator's SafetyMemory), the drop-in the HITBOX builder
        measured at 0 rect contacts over 24 noise runs; the legacy disc
        safety_permit is only the fallback for an older safety.py."""
        ep = getattr(safety, "envelope_permit", None)
        if ep is None or self._safety_mem is None:
            return safety.safety_permit(
                self.state, self.path, self._safety_peers(now), self.grid,
                now, path_index=(self._path_index_near_current()
                                 if self.path else None))
        pts = []
        if self.path:
            i0 = self._path_index_near_current()
            pts = [self.grid.cell_to_world(c)
                   for c in self.path[i0 + 1:i0 + 40]]
        try:
            return ep(self.state, pts, self._safety_peers(now),
                      self._safety_mem, now,
                      lane_pass=self._lane_pass_peers(now))
        except TypeError:       # an older safety.py without lane_pass
            return ep(self.state, pts, self._safety_peers(now),
                      self._safety_mem, now)

    # ------------------------------------------------- lane-pass eligibility
    def _band_axis_theta(self) -> Optional[float]:
        """Map-frame angle of the lane band's travel axis (axis 'v' runs
        along rows = world Y)."""
        band = traffic.lane_band(self.grid)
        if band is None:
            return None
        return math.pi / 2.0 if getattr(band, "axis", "v") == "v" else 0.0

    def _lane_centre_world(self, band, positive: bool) -> float:
        """World coordinate (x for axis 'v') of a lane centre line."""
        centre = band.centre_pos if positive else band.centre_neg
        o = self.grid.origin[0] if band.axis == "v" else self.grid.origin[1]
        return o + (float(centre) + 0.5) * self.grid.resolution

    def _lane_keeping(self, band, axis: float, x: float, y: float,
                      th: float, w: float) -> bool:
        """Inside the band, heading along the corridor, not rotating, and
        within LANE_KEEP_TOL_M of the OWN keep-right lane centre. The w
        bound is safety.LANE_PASS_W_MAX, not the 0.3 rad/s spin threshold:
        lane-keeping corrections at the 1.0 m/s cruise spike past 0.3 every
        few ticks and flickered the pass into a STOP/GO dither; the
        envelope's predicted-lateral-gap check (which includes both w's)
        is the actual guard."""
        w_max = getattr(safety, "LANE_PASS_W_MAX", 0.9)
        if abs(w) > w_max:
            return False
        r, c = self.grid.world_to_cell(x, y)
        if not band.in_band(r, c):
            return False
        d = _ang_norm(th - axis)
        if min(abs(d), math.pi - abs(d)) > LANE_PASS_HEADING_RAD:
            return False
        positive = abs(d) <= math.pi / 2.0        # travelling +axis
        lat = x if band.axis == "v" else y
        return abs(lat - self._lane_centre_world(band, positive)) \
            <= LANE_KEEP_TOL_M

    def _lane_pass_peers(self, now: float) -> Optional[Dict[str, float]]:
        """Peers the envelope may judge by the LATERAL lane gap only
        (safety.envelope_permit lane_pass): effective LANES mode, me and the
        peer both lane-keeping in the band, peer FRESH. The envelope itself
        still verifies the lateral gap against the live sigmas every tick -
        this is eligibility, not permission."""
        if self._eff_mode != traffic.LANES_MODE or hitbox_mod is None:
            return None
        band = traffic.lane_band(self.grid)
        axis = self._band_axis_theta()
        if band is None or axis is None or not hasattr(band, "in_band"):
            return None
        # A sharp turn within TURN_AHEAD_M of MY path keeps the full rule
        # (pure path geometry; my |w| is bounded in _lane_keeping instead,
        # because 10 Hz lane corrections flicker past SPIN_W_RAD_S).
        if self.path and hitbox_mod is not None:
            i0 = self._path_index_near_current()
            pts = [self.grid.cell_to_world(c) for c in self.path[i0:i0 + 12]]
            if hitbox_mod.is_spinning(0.0, pts):
                return None
        p = self.state.pose
        if not self._lane_keeping(band, axis, p.x, p.y, p.theta,
                                  self.state.w):
            return None
        out: Dict[str, float] = {}
        for rid, st in self.registry.alive().items():
            if self.registry.freshness(rid, now) != FRESH:
                continue
            pose = self.registry.extrapolated_pose(rid, now) or st.pose
            if self._lane_keeping(band, axis, pose.x, pose.y, pose.theta,
                                  getattr(st, "w", 0.0)):
                out[rid] = axis
        return out or None

    # ---------------------------------------------- convoy entry (R10 dir)
    def _path_heading_into(self, path: List, cur_i: int,
                           zone) -> Optional[float]:
        """Direction my path drives INTO a ranked zone: heading of the path
        step that first enters it (from cur_i on)."""
        k = next((i for i in range(cur_i, len(path))
                  if path[i] in zone.cells), None)
        if k is None:
            return None
        j = k - 1 if k > cur_i else k
        if j + 1 >= len(path):
            return None
        x0, y0 = self.grid.cell_to_world(path[j])
        x1, y1 = self.grid.cell_to_world(path[j + 1])
        if abs(x1 - x0) < 1e-9 and abs(y1 - y0) < 1e-9:
            return None
        return math.atan2(y1 - y0, x1 - x0)

    def _convoy_zones(self, path: List, cur_i: int,
                      occ: Dict[str, Set[str]], now: float) -> Set[str]:
        """Direction-aware segments (R10 fix: 'the corridor is a two-way
        street, not a lock'). Ranked NON-junction zones on my remaining path
        where every occupant AND every claimant (peers_needing_zone) is a
        FRESH, non-spinning peer heading the way my path drives through the
        zone, with at least one such peer physically inside the run (the
        convoy anchor). Such a zone may be entered while my own FIFO request
        is still queued: the leader's claim is a direction token, not a body
        lock, and the safety envelope spaces the convoy like car-following.

        Why no head-on can result: entry into an EMPTY zone still requires
        the grant (capacity-1 FIFO); the first entrant is therefore always
        the holder, every later entrant must match the direction of a body
        already inside, and an opposing robot fails the direction test AND
        is blocked by the physical-occupancy veto until the convoy has
        drained - at which point the FIFO queue (every convoy entrant
        requested before entering) hands the zone over in arrival order.
        One opposing REQUEST also closes the convoy to NEW entrants at once
        (the claimant test), so opposing traffic is never starved: this is
        the signal-style per-direction corridor token. Junctions (kind
        'intersection') never convoy - flows are arbitrated only where they
        cross."""
        if not path or cur_i >= len(path) or not self.registry.peers:
            return set()
        required = traffic.required_zones(self.grid, path[cur_i:],
                                          self._eff_mode)
        if not required:
            return set()
        inside_run = {rid for z in required for rid in occ.get(z, ())}
        out: Set[str] = set()
        for z in required:
            zone = self.grid.zones.get(z)
            if zone is None or getattr(zone, "kind", "") == "intersection":
                continue
            heading = self._path_heading_into(path, cur_i, zone)
            if heading is None:
                continue
            others = (set(occ.get(z, ()))
                      | set(self.registry.peers_needing_zone(z)))
            others.discard(self.me)
            if not others:
                continue
            ok, anchor = True, False
            for rid in others:
                st = self.registry.peers.get(rid)
                if st is None or self.registry.freshness(rid, now) != FRESH:
                    ok = False
                    break
                pose = self.registry.extrapolated_pose(rid, now) or st.pose
                # MOVING only: a stopped robot is not a convoy - it is a
                # queue (FIFO) or a wedge (inside-beats-stuck / make-way).
                if (abs(getattr(st, "v", 0.0)) < 0.05
                        or (hitbox_mod is not None
                            and abs(getattr(st, "w", 0.0))
                            > hitbox_mod.SPIN_W_RAD_S)
                        or abs(_ang_norm(pose.theta - heading))
                        > CONVOY_HEADING_RAD):
                    ok = False
                    break
                if rid in inside_run:
                    anchor = True
            if ok and anchor:
                out.add(z)
        self._convoy_logged &= out
        return out

    # --------------------------------------------- road model (reservations off)
    @property
    def _reservations_on(self) -> bool:
        return bool(getattr(getattr(self.grid, "traffic", None),
                            "reservations", True))

    def _row_key(self, task_priority: int, pose, goal, battery: float,
                 level: float, rid: str):
        """Right-of-way sort key at a box junction (user order, descending
        wins): (a) Task.priority; (b) near its station - believed distance
        to its broadcast goal <= NEAR_STATION_M; (c) emergency battery;
        then the existing deterministic total order (broadcast level L,
        which already encodes phase/aging; robot_id as final tiebreak,
        LOWER id wins so the key uses its negative sort position)."""
        near = 0
        if goal is not None and pose is not None:
            gx = getattr(goal, "x", None)
            if gx is not None and math.hypot(goal.x - pose.x,
                                             goal.y - pose.y) \
                    <= NEAR_STATION_M:
                near = 1
        emergency = 1 if battery <= BATTERY_EMERGENCY_PCT else 0
        return (int(task_priority), near, emergency, float(level),
                tuple(-ord(c) for c in rid))

    def _box_right_of_way_blocker(self, zone_id: str, path: List,
                                  cur_i: int, now: float) -> Optional[str]:
        """A peer that outranks me is in, or imminently entering, the box:
        I wait at the line. A peer moving MY way through the box (same
        direction, car-following) never blocks; the envelope spaces us."""
        zone = self.grid.zones.get(zone_id)
        if zone is None:
            return None
        heading = self._path_heading_into(path, cur_i, zone)
        my_goal = getattr(self.intent, "goal", None)
        mine = self._row_key(
            int(getattr(self.current_task, "priority", 0) or 0)
            if self.current_task is not None else 0,
            self.state.pose, my_goal, self.state.battery_pct,
            self._cmp_L, self.me)
        occ = traffic.zone_occupants(self.grid, zone_id,
                                     self._peer_positions(now))
        for rid, st in self.registry.alive().items():
            pose = self.registry.extrapolated_pose(rid, now) or st.pose
            in_box = rid in occ
            if not in_box:
                # imminent: its intent crosses the box within ~2 s
                held = self.registry.intents.get(rid)
                cells = held.intent.cells if held is not None else []
                coming = any(c in zone.cells for c in cells[:25])
                if not (coming and abs(st.v) >= 0.05):
                    continue
            if (heading is not None
                    and abs(_ang_norm(pose.theta - heading))
                    <= CONVOY_HEADING_RAD and abs(st.v) >= 0.05):
                continue            # same direction: car-following, not ROW
            theirs = self._row_key(
                int(getattr(st, "task_priority", 0) or 0), pose,
                getattr(st.intent, "goal", None),
                getattr(st, "battery_pct", 100.0),
                float(getattr(st, "priority_score", 0.0)), rid)
            if in_box or theirs > mine:
                return rid
        return None

    def _box_permit(self, now: float) -> Optional[Permit]:
        """ROAD-MODEL junction discipline (no mutexes): approaching a box
        junction (zone kind 'intersection'), stop ACQ_STANDOFF_M short of
        the line while (a) my exit beyond the box is blocked by a stopped
        robot (don't-block-the-box) or (b) a crossing robot with right of
        way is in or imminently entering the box. Re-evaluated every tick;
        no state, no requests, nothing held."""
        cur_i = self._path_index_near_current()
        inside = set(self._inside_zones())
        k, zhit = None, None
        for i in range(cur_i, min(cur_i + 80, len(self.path))):
            for zid, z in self.grid.zones.items():
                if (getattr(z, "kind", "") == "intersection"
                        and zid not in inside and self.path[i] in z.cells):
                    k, zhit = i, zid
                    break
            if k is not None:
                break
        if k is None:
            return None
        dist = traffic.path_distance_m(self.grid, self.path, cur_i, k) \
            - traffic.ACQ_STANDOFF_M
        if dist > ACQ_REQUEST_DISTANCE_M:
            return None
        who = self._junction_exit_blocked(zhit, self.path, cur_i, now) \
            or self._box_right_of_way_blocker(zhit, self.path, cur_i, now)
        if not who:
            return None
        why = f"box junction {zhit}: waiting for {who}"
        if dist <= ACQ_STOP_TOL_M:
            self._acq_waiting = True
            return Permit(action="STOP", speed_scale=0.0, reason=why,
                          blocking_robot=who, zone_id=zhit)
        if dist <= ACQ_SLOW_DISTANCE_M:
            return Permit(action="SLOW", speed_scale=MIN_SPEED_SCALE,
                          reason=why, blocking_robot=who, zone_id=zhit)
        return None

    # --------------------------------------------- don't-block-the-box (A2)
    def _junction_exit_blocked(self, zone_id: str, path: List, cur_i: int,
                               now: float) -> Optional[str]:
        """Road model A2: before entering a junction, my exit must be clear.
        Returns the blocking robot id if a STOPPED peer's hitbox overlaps my
        first BOX_EXIT_CLEAR_M of path BEYOND the junction, else None. Only
        zones of kind 'intersection' are checked; a moving peer is cross
        traffic the mutex and envelope already order."""
        zone = self.grid.zones.get(zone_id)
        if zone is None or getattr(zone, "kind", "") != "intersection":
            return None
        last_in = None
        for i in range(cur_i, len(path)):
            if path[i] in zone.cells:
                last_in = i
        if last_in is None or last_in + 1 >= len(path):
            return None
        exit_cells: Set = set()
        d = 0.0
        for i in range(last_in + 1, len(path)):
            exit_cells.add(path[i])
            if i + 1 < len(path):
                d += self.grid.cell_distance_m(path[i], path[i + 1])
            if d >= BOX_EXIT_CLEAR_M:
                break
        for rid, st in self.registry.peers.items():
            if not st.alive or abs(getattr(st, "v", 0.0)) >= 0.05:
                continue
            if now - self.registry.last_seen.get(rid, st.rx_time) >= T_GONE_S:
                continue
            pose = self.registry.extrapolated_pose(rid, now) or st.pose
            body = traffic.occupant_cells(
                self.grid, (pose.x, pose.y, pose.theta,
                            max(0.0, float(st.loc_sigma_lat or 0.0))))
            if body & exit_cells:
                return rid
        return None

    @property
    def hitbox_disc_mode(self) -> bool:
        """True while this robot must be treated as its 0.286 m circumscribed
        DISC rather than its oriented rect: spinning (|w| > 0.3 rad/s) or a
        > 45 deg turn within 0.5 m of upcoming path (core/hitbox.is_spinning).
        Read by sih-57's publish_diagnostics via getattr - do not rename."""
        try:
            from . import hitbox
        except Exception:
            return False
        pts = None
        if self.path:
            i0 = self._path_index_near_current()
            pts = [self.grid.cell_to_world(c)
                   for c in self.path[i0:i0 + 12]]
        try:
            return bool(hitbox.is_spinning(self.state.w, pts))
        except Exception:
            return False

    # ---------------------------------------- strict directional lanes (R)
    def _lane_confinement_active(self) -> bool:
        """Lane confinement is law whenever the planner's band map exists
        AND lanes govern the geometry: always in the road model
        (reservations off - the shipped configuration), else only in
        effective LANES mode. The reservations-on SPINE fallback is a
        capacity-1 mutex where corridors are single-file by construction;
        confining its recovery searches to half the corridor would only
        starve them (and its fixtures/tests pin the old shapes)."""
        if getattr(self.grid, "lane_band_at", None) is None:
            return False
        return (not self._reservations_on
                or self._eff_mode == traffic.LANES_MODE)

    def _my_travel_dir(self) -> str:
        """NB/SB/EB/WB: the direction of the lane band under my pose (MY OWN
        lane, whatever my transient heading while manoeuvring in it), else
        my heading's dominant compass direction (off-road, junction boxes)."""
        cell = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        hit = traffic.band_at(self.grid, cell)
        if hit is not None:
            return hit[1]
        return traffic.heading_dir(self.state.pose.theta)

    def _lane_no_go(self) -> Set[tuple]:
        """Cells lane confinement forbids for my current travel direction
        (cached per direction; empty while confinement is inactive). The
        DESIGNATED tier-1 cells (pockets / wait bays / retreat mouths) are
        re-admitted: the layout engineered them per direction, and they are
        the own-lane yield shape's hold spots."""
        if not self._lane_confinement_active():
            return set()
        d = self._my_travel_dir()
        cached = self._lane_nogo_cache.get(d)
        if cached is None:
            cached = (traffic.confined_no_go(self.grid, d)
                      - self._pocket_grade_cells())
            self._lane_nogo_cache[d] = cached
        return cached

    def _path_runs_against_a_lane(self, path) -> bool:
        """True if any step of `path` travels ALONG a directed lane against
        its direction, outside crossing zones and outside an engaged
        overtake's borrow window - the same predicate as the trajectory
        audit's wrong-lane rule, applied to a plan. Pure-lateral steps and
        diagonals whose along-component matches the lane are fine."""
        if (not self._lane_confinement_active()
                or getattr(self.grid, "lane_band_at", None) is None):
            return False
        borrow = self._overtake["borrow"] if self._overtake is not None else ()
        for (r0, c0), (r1, c1) in zip(path, path[1:]):
            if (r1, c1) in borrow or self.grid.in_crossing_zone(r1, c1):
                continue
            hit = traffic.band_at(self.grid, (r1, c1))
            if hit is None:
                continue
            dvx, dvy = traffic.DIR_VECS[hit[1]]
            if (c1 - c0) * dvx + (r1 - r0) * dvy < 0:
                return True
        return False

    def _recovery_no_go(self) -> Set[tuple]:
        """Ground recovery behaviours (refuge/retreat/make-way) may not use:
        ranked zones I neither hold nor stand in (rank order), plus - strict
        lanes - every band cell that is not my own direction's lane, not
        off-road and not a crossing zone. A yielder therefore never parks in
        or cuts through the opposing lane: it reverses along its own lane to
        a pocket/junction/turnaround instead."""
        return self._unusable_zone_cells() | self._lane_no_go()

    def _blocker_lane_cells(self, blockers, now: float) -> Set[tuple]:
        """While yielding, the whole DIRECTIONAL lane of every robot I yield
        to (its half of its band's rect, junction boxes included) is out of
        bounds for the retreat route and target: the blocker WILL drive that
        lane, however short its broadcast intent. Without this the measured
        sigma-0.3 head-on yielder crossed the blocker's lane column inside a
        junction box beyond the 3 s intent horizon, the blocker caught up,
        and the pair re-wedged mid-box every cycle."""
        out: Set[tuple] = set()
        if getattr(self.grid, "lane_band_at", None) is None:
            return out
        for rid in blockers:
            st = self.registry.peers.get(rid)
            if st is None:
                continue
            pose = self.registry.extrapolated_pose(rid, now) or st.pose
            band = traffic.band_at(
                self.grid, self.grid.world_to_cell(pose.x, pose.y))
            if band is None:        # in a box / off-road: read its intent
                band = next(
                    (b for b in (traffic.band_at(self.grid, c)
                                 for c in st.intent.cells) if b is not None),
                    None)
            if band is None:
                continue
            # The strip is the half the blocker TRAVELS, not the half its
            # body happens to stand in (a southbound robot stopped astride
            # the NB half must not seal the victim's own NB lane). Heading
            # picks the half; the position-half is the fallback when the
            # heading runs across the band's axis (mid-crossing).
            strip = traffic.lane_strip_cells(
                self.grid, band[0], traffic.heading_dir(pose.theta))
            out |= strip or traffic.lane_strip_cells(self.grid, band[0],
                                                     band[1])
        return out

    def _opposing_bystander(self, rid: str, now: float) -> bool:
        """Move-when-clear (strict lanes): a peer whose body lies WHOLLY in
        lane bands of another direction, outside crossing zones and off my
        remaining path's cells, is not a blocker - it gains no wait-for edge
        and no deadlock entry from me. This filters BLAME only, never
        geometry: the safety envelope keeps seeing every peer."""
        if not self._lane_confinement_active():
            return False
        st = self.registry.peers.get(rid)
        if st is None:
            return False
        pose = self.registry.extrapolated_pose(rid, now) or st.pose
        body = traffic.occupant_cells(
            self.grid, (pose.x, pose.y, pose.theta,
                        max(0.0, float(st.loc_sigma_lat or 0.0))))
        my_dir = self._my_travel_dir()
        seen_band = False
        for cell in body:
            if not self.grid.is_static_free(cell):
                continue            # wall/rack cells carry no lane
            hit = traffic.band_at(self.grid, cell)
            if hit is None or hit[1] == my_dir:
                return False
            if traffic.in_crossing(self.grid, cell):
                return False
            seen_band = True
        if not seen_band:
            return False
        if self.path:
            cur_i = self._path_index_near_current()
            if body & {tuple(c) for c in self.path[cur_i:]}:
                return False
        return True

    def _path_uturn(self, cur_i: int) -> Optional[dict]:
        """The first direction REVERSAL on my remaining path: the stretch
        where it crosses from a band of my travel direction into the
        OPPOSITE direction's band (NB<->SB, EB<->WB). Perpendicular band
        changes are ordinary junction turns, not U-turns. None when the
        grid has no band map or no reversal lies within the lookahead.

        Returns {'k0': last own-lane index (the stop line), 'j': first
        opposite-band index, 'cells': the crossing cells between the lane
        centres (through the first along-target step), 'my_band', 'target'}.
        """
        if getattr(self.grid, "lane_band_at", None) is None or not self.path:
            return None
        path = self.path
        # An engaged overtake legally drives opposite-direction cells inside
        # its gated borrow window: they are NOT a reversal to re-gate.
        borrow = self._overtake["borrow"] if self._overtake is not None else ()
        end = min(len(path), cur_i + UTURN_LOOKAHEAD_CELLS)
        d0 = None                   # (lane_id, direction) I travel in
        last0 = None                # last path index still in d0's direction
        for i in range(cur_i, end):
            if path[i] in borrow:
                continue            # gated overtake ground (its own gate)
            hit = traffic.band_at(self.grid, path[i])
            if hit is None:
                continue            # off-road / junction box
            if d0 is None or hit[1] == d0[1]:
                d0, last0 = hit, i
                continue
            if hit[1] != traffic.OPPOSITE_DIR.get(d0[1]):
                d0, last0 = hit, i  # perpendicular: an ordinary turn
                continue
            # Reversal ahead: path[i] is the first opposite-direction cell.
            return self._uturn_info(path, last0 if last0 is not None
                                    else max(cur_i, i - 1), i, d0, hit)
        # No reversal AHEAD - but I may already be ASTRIDE one: the
        # remaining path then reads all target-lane and the own-lane cells
        # lie behind cur_i (robustness for a restart/registration mid-turn).
        if d0 is None or cur_i == 0:
            return None
        for i in range(cur_i - 1, max(-1, cur_i - UTURN_LOOKAHEAD_CELLS), -1):
            if path[i] in borrow:
                continue            # gated overtake ground (its own gate)
            hb = traffic.band_at(self.grid, path[i])
            if hb is None or hb[1] == d0[1]:
                continue            # crossing stretch reads as target-lane
            if hb[1] != traffic.OPPOSITE_DIR.get(d0[1]):
                return None         # perpendicular behind: no reversal
            j = next((k for k in range(i + 1, end)
                      if traffic.band_at(self.grid, path[k]) is not None
                      and traffic.band_at(self.grid, path[k])[1] == d0[1]),
                     None)
            if j is None:
                return None
            info = self._uturn_info(path, i, j, hb, d0)
            # Only while still inside the crossing window: once the robot
            # is established in the target lane the turn is over.
            return info if cur_i <= info["j2"] else None
        return None

    def _uturn_info(self, path: List, k0: int, j: int, my_band,
                    target) -> dict:
        """The turn window: k0 = last own-lane index (the stop line), j =
        first opposite-band index; the swept cells run from k0+1 through the
        first step ALONG the target direction (the robot is re-aligned in
        its new lane there)."""
        dvx, dvy = traffic.DIR_VECS[target[1]]
        j2 = j
        for k in range(j, min(len(path) - 1, j + UTURN_WINDOW_INTO_TARGET)):
            j2 = k
            (r0, c0), (r1, c1) = path[k], path[k + 1]
            if (c1 - c0) * dvx + (r1 - r0) * dvy > 0:
                break
        cells = [tuple(c) for c in path[k0 + 1:j2 + 1]] or [tuple(path[j])]
        return dict(k0=k0, j=j, j2=j2, cells=cells, my_band=my_band,
                    target=target)

    def _uturn_permit(self, now: float) -> Optional[Permit]:
        """Execution half of the U-turn rule: ONE gate (traffic.uturn_gate)
        for the first reversal on my path. While the gate fails the robot
        waits IN ITS OWN LANE at the stand-off (an acquisition-style wait:
        bounded by the traffic passing, so the no-cycle deadlock timeout is
        suppressed); after UTURN_BLOCKED_REPLAN_S it replans around these
        turn cells through the ordinary machinery ('drive on to where the
        turn fits'). uturn_active (audit/webapp contract name) is True while
        a gated turn is engaging or executing; a robot already astride the
        divider always finishes - stopping across the opposing lane is the
        worst outcome, and the safety envelope still owns the live geometry
        every tick."""
        pose = self.state.pose
        here = self.grid.world_to_cell(pose.x, pose.y)
        if not self.path:
            self._uturn = None
            self._uturn_block_t0 = None
            return None
        cur_i = self._path_index_near_current()
        info = self._path_uturn(cur_i)
        if info is None:
            # Once I am astride the divider the remaining path shows no
            # reversal any more (its cells already read as the target lane):
            # the ENGAGED turn's cells carry uturn_active to completion.
            if self._uturn is not None and here in self._uturn["cells"]:
                self.uturn_active = True
                return None
            self._uturn = None
            self._uturn_block_t0 = None
            return None
        me = (pose.x, pose.y, pose.theta,
              max(0.0, float(self.state.loc_sigma_lat or 0.0)))
        ok, reason = traffic.uturn_gate(
            now, me, info["my_band"], info["target"], info["cells"],
            self.registry.alive(), self.grid)
        committed = cur_i > info["k0"] or here in set(info["cells"])
        if committed:
            self._uturn = dict(cells=set(info["cells"]))
            self.uturn_active = True
            self._uturn_block_t0 = None
            return None
        dist = traffic.path_distance_m(self.grid, self.path, cur_i,
                                       info["k0"])
        if ok:
            self._uturn_block_t0 = None
            if dist <= ACQ_SLOW_DISTANCE_M:
                self._uturn = dict(cells=set(info["cells"]))
                self.uturn_active = True
            return None
        self._uturn = None
        if self._uturn_block_t0 is None:
            self._uturn_block_t0 = now
        elif (now - self._uturn_block_t0 > UTURN_BLOCKED_REPLAN_S
              and self.goal_cell is not None):
            saved = (list(self.path), self.intent)
            if self.replan(now=now, extra_blocked=set(info["cells"])):
                self._uturn_block_t0 = None
                self.log(f"U-turn blocked ({reason}) - replanned to a turn "
                         f"spot farther on")
                return None
            self.path, self.intent = saved
            self._uturn_block_t0 = now      # keep waiting, retry the replan
        blocker = next((r for r in sorted(self.registry.peers)
                        if r and r in reason), "")
        why = f"U-turn gate: {reason}"
        if dist <= ACQ_STOP_TOL_M:
            self._acq_waiting = True
            return Permit(action="STOP", speed_scale=0.0, reason=why,
                          blocking_robot=blocker)
        if dist <= ACQ_SLOW_DISTANCE_M:
            return Permit(action="SLOW", speed_scale=MIN_SPEED_SCALE,
                          reason=why, blocking_robot=blocker)
        return None

    # ------------------------------------------- stuck self-recovery (F1)
    def _track_peer_motion(self, now: float) -> None:
        """Per-peer stationary-since clock from the broadcast v (feeds the
        overtake trigger): any |v| >= 0.05 or a self-declared fault resets."""
        for rid, st in self.registry.peers.items():
            if abs(getattr(st, "v", 0.0) or 0.0) >= 0.05:
                self._peer_still_since.pop(rid, None)
            else:
                self._peer_still_since.setdefault(rid, now)

    def _static_ahead_m(self) -> float:
        """Static free distance along my EXACT heading (ray-marched at half
        a cell). Small = the navigation stack's forward stop box is held by
        a wall or rack face: the self-stuck signature (F1 nose-to-wall at
        theta 1.9, F2 rack corner at theta 0.5). The exact ray, not a
        compass cone: a 45-deg neighbour ray clips rack corners diagonally
        and false-fired on robots driving a clear straight lane."""
        pose = self.state.pose
        if not self.grid.in_bounds(
                self.grid.world_to_cell(pose.x, pose.y)):
            return 0.0
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        step = 0.5 * self.grid.resolution
        d = step
        while d <= STUCK_STATIC_AHEAD_M + 1e-9:
            cell = self.grid.world_to_cell(pose.x + c * d, pose.y + s * d)
            if not self.grid.is_static_free(cell):
                return d
            d += step
        return float("inf")

    def _stuck_self_recover(self, now: float) -> Permit:
        """GO + no motion + NO alive peer to blame: the navigation stack is
        holding the robot against something STATIC (wall / rack corner).
        First replan with the next stretch of path hard-blocked, so A* -
        with the turn-shaping and forward-hazard terms - must produce a
        different approach geometry; while that is cooling down or fails,
        hold as STOP with no blocker: status becomes WAITING, and the
        deadlock timeout elects ME (an actionable victim) so the ordinary
        retreat machinery runs."""
        stop = Permit(action="STOP", speed_scale=0.0,
                      reason="no physical progress (self-stuck); recovering")
        if now < self._stuck_recover_at or not self.path:
            return stop
        self._stuck_recover_at = now + STUCK_RECOVER_COOLDOWN_S
        cur_i = self._path_index_near_current()
        ahead: Set[tuple] = set()
        d = 0.0
        for i in range(cur_i + 1, len(self.path)):
            ahead.add(tuple(self.path[i]))
            d += self.grid.cell_distance_m(self.path[i - 1], self.path[i])
            if d >= STUCK_BLOCK_AHEAD_M:
                break
        saved = (list(self.path), self.intent)
        if ahead and self.replan(now=now, extra_blocked=ahead):
            seen = self._static_ahead_m() < STUCK_STATIC_AHEAD_M
            self.log(f"stuck with GO ({now - (self._progress_t or now):.1f}s,"
                     f" nobody to blame, "
                     f"{'face in the nose ray' if seen else 'phantom hold'})"
                     f": replanned away from the obstruction "
                     f"({len(ahead)} cells blocked)")
            self._progress_xy = None        # fresh window for the new route
            self._progress_t = None
            return safety.most_restrictive(
                Permit(action="GO", speed_scale=1.0,
                       reason="self-stuck: replanned approach"),
                self._envelope(now))
        self.path, self.intent = saved
        self.log("stuck with GO and no alternative approach: holding as "
                 "WAITING (deadlock-eligible)")
        return stop

    # ------------------------------------------- street overtake (parked car)
    def _maybe_overtake(self, permit: Permit, now: float) -> Optional[Permit]:
        """Trigger: see the T_OVERTAKE_S constants block. Returns the GO
        permit of a freshly engaged overtake, or None."""
        if (permit.action != "STOP" or self._overtake is not None
                or self._retreat is not None or self.goal_cell is None
                or not self.path or not self._lane_confinement_active()):
            return None
        rid = permit.blocking_robot or ""
        st = self.registry.alive().get(rid) if rid else None
        if st is None:
            return None
        if now - self._peer_still_since.get(rid, now) < T_OVERTAKE_S:
            return None
        # Queue vs wreck: a robot WAITING ON SOMEONE (waiting_for set) is a
        # queue member and will move; a robot YIELDING is mid-manoeuvre.
        # But a robot WAITING on NOBODY is the self-stuck wreck class (the
        # phantom-hold recovery downgrades a wall-held GO robot to exactly
        # this state) - after T_OVERTAKE_S stationary it is a parked car.
        if (getattr(st, "status", "") == YIELDING
                or (getattr(st, "status", "") == WAITING
                    and (getattr(st, "waiting_for", "") or ""))):
            return None
        if (now < self._ot_cooldown_until.get(rid, 0.0)
                or now - self._ot_last_try < OVERTAKE_RETRY_S):
            return None
        self._ot_last_try = now
        pose = self.registry.extrapolated_pose(rid, now) or st.pose
        # At MY goal it is a station queue (goal-queue permit), never a pass.
        gx, gy = self.grid.cell_to_world(self.goal_cell)
        if math.hypot(pose.x - gx, pose.y - gy) <= MW_GOAL_CLEAR_M:
            return None
        me = self.state.pose
        here = self.grid.world_to_cell(me.x, me.y)
        # My lane: the band under me, else - queued just inside a junction
        # box, where band_at is None - the band under the BLOCKER (it is on
        # my next stretch of path, so its lane is the one being blocked).
        band = (traffic.band_at(self.grid, here)
                or traffic.band_at(self.grid,
                                   self.grid.world_to_cell(pose.x, pose.y)))
        if band is None or band[1] != traffic.heading_dir(me.theta):
            return None     # no borrowable lane geometry here
        lane_id, my_dir = band
        dvx, dvy = traffic.DIR_VECS[my_dir]
        a_me = me.x * dvx + me.y * dvy
        a_blk = pose.x * dvx + pose.y * dvy
        if not 0.0 < a_blk - a_me <= OVERTAKE_RANGE_M:
            return None
        sigma = max(0.0, float(getattr(st, "loc_sigma_lat", 0.0) or 0.0))
        if hitbox_mod is not None:
            body = hitbox_mod.obstacle_cells(self.grid, pose.x, pose.y,
                                             pose.theta, sigma)
        else:
            body = traffic.occupant_cells(
                self.grid, (pose.x, pose.y, pose.theta, sigma))
        cur_i = self._path_index_near_current()
        span = int(OVERTAKE_RANGE_M / self.grid.resolution) + 1
        if not (body & {tuple(c) for c in self.path[cur_i:cur_i + span]}):
            return None     # not actually on my next stretch of path
        # Only the robot DIRECTLY behind the blocker passes; everyone else
        # queues behind the overtaker's slot (no convoy weaving).
        for orid, ost in self.registry.alive().items():
            if orid == rid:
                continue
            op = self.registry.extrapolated_pose(orid, now) or ost.pose
            if (a_me < op.x * dvx + op.y * dvy < a_blk
                    and traffic.band_at(
                        self.grid, self.grid.world_to_cell(op.x, op.y))
                    == band):
                return None
        # A LEGAL in-lane detour (opposing lane hard-blocked) is preferred:
        # the ordinary deadlock reroute machinery will take it instead.
        blocked = self.grid.blocked_cells(now) | self._peer_obstacle_cells() \
            | set(body) | self._lane_no_go()
        blocked.difference_update(self._self_cells())
        kw = {"mode": self._eff_mode} if _ASTAR_HAS_MODE else {}
        if astar.astar(self.grid, here, self.goal_cell, blocked=blocked,
                       **kw):
            return None
        # The borrow window: the opposing half of MY band from just behind
        # me to past the blocker plus the merge-back run.
        opp = traffic.OPPOSITE_DIR[my_dir]
        strip = traffic.lane_strip_cells(self.grid, lane_id, opp)
        lo = a_me - OVERTAKE_BEHIND_M
        hi = a_blk + OVERTAKE_CLEAR_BEYOND_M
        clr = self.grid.clearance_m()
        borrow: Set[tuple] = set()
        for c in strip:
            # Undrivable slivers at the strip's rack-row edges are not part
            # of the manoeuvre - the planner never routes through them and
            # they must not fail the gate's drivability check.
            if clr[c[0]][c[1]] < traffic.UTURN_DRIVE_CLEAR_M:
                continue
            x, y = self.grid.cell_to_world(c)
            if lo <= x * dvx + y * dvy <= hi:
                borrow.add((int(c[0]), int(c[1])))
        borrow -= set(body)
        if not borrow:
            return None
        my_info = (me.x, me.y, me.theta,
                   max(0.0, float(self.state.loc_sigma_lat or 0.0)))
        ok, reason = traffic.overtake_gate(now, my_info, band, sorted(borrow),
                                           self.registry.alive(), self.grid,
                                           blocker_id=rid)
        if not ok:
            self.log(f"overtake of stationary {rid} denied: {reason}")
            return None
        saved = (list(self.path), self.intent)
        # Planning clearance around the parked body: the 0.88 m disc the
        # victim-reroute machinery measured as the minimum for a pass that
        # the stationary-rule envelope will actually let through (plus the
        # planner-grade rect cells for the odd sliver the disc misses).
        plan_avoid = set(map(tuple, body)) | self._disc_cells(
            pose.x, pose.y, REROUTE_BLOCKER_CLEAR_M)
        self._overtake = dict(blocker=rid, borrow=frozenset(borrow),
                              avoid=frozenset(plan_avoid), band=band,
                              t0=now, committed=False)
        if not self.replan(now=now):
            self._overtake = None
            self.path, self.intent = saved
            self._ot_cooldown_until[rid] = now + OVERTAKE_COOLDOWN_S
            return None
        self.overtake_count += 1
        self.overtake_active = True
        self.log(f"OVERTAKE: borrowing the {opp} half of {lane_id} to pass "
                 f"stationary {rid} (window {len(borrow)} cells)")
        self._end_wait(now)
        return safety.most_restrictive(
            Permit(action="GO", speed_scale=1.0,
                   reason=f"overtaking stationary {rid}"),
            self._envelope(now))

    def _overtake_tick(self, now: float) -> None:
        """Lifecycle of an engaged overtake: completion once my body and my
        remaining path have left the borrow window; pre-commit abort (and an
        in-lane replan) when the blocker moves again, the gate turns bad or
        the engage bound expires; COMMITTED - body inside the borrow cells -
        always finishes (stopping in the borrowed lane is forbidden; the
        envelope still owns the live geometry every tick)."""
        ot = self._overtake
        if ot is None:
            return
        here = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        borrow = ot["borrow"]
        in_borrow = here in borrow
        path_hits = False
        if self.path:
            cur_i = self._path_index_near_current()
            path_hits = any(tuple(c) in borrow for c in self.path[cur_i:])
        if not in_borrow and not path_hits:
            self._overtake = None
            self._ot_cooldown_until[ot["blocker"]] = now + OVERTAKE_COOLDOWN_S
            self.log(f"overtake of {ot['blocker']} complete: back in lane")
            return
        if in_borrow or ot.get("committed"):
            ot["committed"] = True
            self.overtake_active = True
            return
        abort = None
        st = self.registry.peers.get(ot["blocker"])
        if (st is not None and st.alive
                and abs(getattr(st, "v", 0.0) or 0.0) >= 0.05):
            abort = "blocker moving again"
        elif now - ot["t0"] > OVERTAKE_MAX_S:
            abort = "engage timeout"
        else:
            me = self.state.pose
            my_info = (me.x, me.y, me.theta,
                       max(0.0, float(self.state.loc_sigma_lat or 0.0)))
            ok, reason = traffic.overtake_gate(
                now, my_info, ot["band"], sorted(borrow),
                self.registry.alive(), self.grid, blocker_id=ot["blocker"])
            if not ok:
                abort = reason
        if abort is not None:
            blocker = ot["blocker"]
            self._overtake = None
            self._ot_cooldown_until[blocker] = now + OVERTAKE_COOLDOWN_S
            self.log(f"overtake of {blocker} aborted pre-commit ({abort}); "
                     f"replanning in lane")
            if self.goal_cell is not None:
                self.replan(now=now)
            return
        self.overtake_active = True

    def _proposed_permit(self, peers, now: float) -> Permit:
        """Layer order (merged spec 3): zone permit -> goal queue -> U-turn
        gate -> give-way -> make-way -> envelope (most_restrictive) -> the
        deadlock backstop (applied by tick()). The envelope applies whatever
        the priority order or zone state says.
        """
        base = self._zone_permit(now)
        base = safety.most_restrictive(base, self._goal_queue_permit(now))
        base = safety.most_restrictive(base, self._uturn_permit(now))
        base = safety.most_restrictive(base, self._give_way_permit(now))
        started = self._maybe_make_way(now)
        if started is not None:
            return started
        base = safety.most_restrictive(base, self._partition_permit(now))
        return safety.most_restrictive(base, self._envelope(now))

    # ----------------------------------------------- goal queue (spec 3)
    def _goal_queue_permit(self, now: float) -> Optional[Permit]:
        """A peer within GOALQ_NEAR_M of my goal cell: SLOW inside
        GOALQ_SLOW_M of the goal, STOP at GOALQ_STOP_M, blocking_robot set to
        the occupant - instead of driving the envelope into a nose-to-tail
        standoff over the station cell."""
        if self.goal_cell is None or not self.path:
            return None
        gx, gy = self.grid.cell_to_world(self.goal_cell)
        me_d = math.hypot(self.state.pose.x - gx, self.state.pose.y - gy)
        if me_d > GOALQ_SLOW_M:
            return None
        for rid in self.registry.alive():
            xy = self._peer_xy(rid, now)
            if xy is None or math.hypot(xy[0] - gx, xy[1] - gy) > GOALQ_NEAR_M:
                continue
            if me_d <= GOALQ_STOP_M:
                # A goal-queue wait is bounded by the occupant leaving the
                # station: structural, so it suppresses the no-cycle
                # deadlock timeout like an acquisition wait (cycles and the
                # 20 s cap still apply). Without this, 8 s of queueing
                # elected deadlock victims and 30-40 s retreats (measured).
                self._acq_waiting = True
                return Permit(action="STOP", speed_scale=0.0,
                              reason=f"goal queue: {rid} occupies my goal",
                              blocking_robot=rid)
            return Permit(action="SLOW", speed_scale=MIN_SPEED_SCALE,
                          reason=f"goal queue: {rid} at my goal",
                          blocking_robot=rid)
        return None

    # ------------------------------------------------- give-way (spec 3)
    def _my_lookahead(self, horizon_m: float):
        """[(x, y, cum_m)] of my path from the current index, to horizon_m."""
        if not self.path:
            return []
        cur_i = self._path_index_near_current()
        out, cum = [], 0.0
        px, py = self.state.pose.x, self.state.pose.y
        for i in range(cur_i, len(self.path)):
            cx, cy = self.grid.cell_to_world(self.path[i])
            cum += math.hypot(cx - px, cy - py)
            out.append((cx, cy, cum))
            px, py = cx, cy
            if cum > horizon_m:
                break
        return out

    def _peer_corridor(self, st: RobotState, now: float, horizon_m: float):
        """Peer body + its next horizon_m of broadcast intent, as
        [(x, y, peer_travel_m)], plus its unit travel direction (None if
        unknowable). peer_travel_m is how far ALONG ITS OWN ROUTE the peer
        is from that point - the give-way imminence test."""
        xy = self._peer_xy(st.robot_id, now) or (st.pose.x, st.pose.y)
        pts = [(xy[0], xy[1], 0.0)]
        cum, px, py = 0.0, xy[0], xy[1]
        for cell in st.intent.cells:
            cx, cy = self.grid.cell_to_world(cell)
            cum += math.hypot(cx - px, cy - py)
            if cum > horizon_m:
                break
            pts.append((cx, cy, cum))
            px, py = cx, cy
        if abs(st.v) >= 0.05:
            direction = (math.cos(st.pose.theta), math.sin(st.pose.theta))
        elif len(pts) >= 2:
            dx, dy = pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]
            n = math.hypot(dx, dy)
            direction = (dx / n, dy / n) if n > 1e-6 else None
        else:
            direction = None
        return pts, direction

    # MEASURED DEVIATION from merged spec 3: the give-way hold-short layer
    # is OFF by default. In this warehouse every crossing is already ordered
    # by a ranked-zone mutex (junctions), the lanes (spine), the envelope
    # (imminent geometry), the goal queue and make-way; adding hold-short on
    # top parked bodies inside the disc envelope and bred STOP<->STOP
    # 2-cycles and retreat pile-ups. 6-seed, 480 s random streams:
    #   give-way ON  (any of 4 tunings): mean 10.9-12.1 deliveries,
    #       8-17 deadlock firings/run, max waits up to 256 s;
    #   give-way OFF: mean 15.3, 9 firings TOTAL over 6 runs, max wait
    #       10.4 s, 0 contacts.
    # The layer stays implemented (and unit-tested) behind this flag for
    # open-floor geometries that have no zone protection.
    give_way_enabled = False

    def _give_way_permit(self, now: float) -> Optional[Permit]:
        """Hold short of a HIGHER-priority peer's corridor crossing my path
        at > GW_HEADING_RAD. Only the lower robot stops; same-direction
        following is car-following and stays the envelope's job."""
        if not self.give_way_enabled or not self.path:
            return None
        mine = self._my_lookahead(ROW_MY_LOOK_M)
        if not mine:
            return None
        ranked_cells = self._ranked_cells() if traffic.has_ranked_zones(
            self.grid) else set()
        worst = None
        for rid, st in self.registry.alive().items():
            if st.status in (WAITING, YIELDING) or abs(st.v) < 0.05:
                # A stopped peer is not DRIVING through the crossing: its
                # queueing is the zone layer's job, its wait on me is the
                # make-way trigger, and its parked body is the envelope's
                # stationary-peer rule. Holding short for it anyway was a
                # measured 2-cycle factory (seed 7: 16 deadlock firings and
                # 11 deliveries with it, 1 and 16 without).
                continue
            if not self._peer_outranks_me(st, now):
                continue
            corridor, pdir = self._peer_corridor(st, now, ROW_PEER_LOOK_M)
            hit = None
            for (x, y, cum) in mine:
                if (x, y) != (mine[0][0], mine[0][1]) and \
                        self.grid.world_to_cell(x, y) in ranked_cells:
                    # The crossing sits inside a ranked zone: the mutex
                    # already serializes it, and a second hold-short in
                    # front of it double-guards the junction (measured
                    # deadlock churn at J_N).
                    continue
                near = [pt for pt in corridor
                        if math.hypot(x - pt[0], y - pt[1])
                        <= ROW_CORRIDOR_M]
                if near:
                    # peer_travel of the crossing: how far the mover still
                    # is from it along its own route (imminence).
                    hit = (x, y, cum, min(pt[2] for pt in near))
                    break
            if hit is None:
                continue
            if pdir is not None:
                # My direction at the crossing vs the peer's travel
                # direction. Within GW_HEADING_RAD = same direction =
                # car-following (envelope's job). Above pi - GW_HEADING_RAD =
                # anti-parallel = a lane/aisle PASS, also the envelope's job
                # (holding short for every oncoming pass stopped the lanes
                # cold: lane centres sit 0.5-0.8 m apart, inside the 0.75 m
                # corridor test). Give-way is for true CROSSINGS.
                k = next(i for i, pt in enumerate(mine)
                         if pt[0] == hit[0] and pt[1] == hit[1])
                ax, ay = (mine[k - 1][:2] if k > 0
                          else (self.state.pose.x, self.state.pose.y))
                mdx, mdy = hit[0] - ax, hit[1] - ay
                n = math.hypot(mdx, mdy)
                if n > 1e-6:
                    cosang = (mdx * pdir[0] + mdy * pdir[1]) / n
                    ang = math.acos(max(-1.0, min(1.0, cosang)))
                    if (ang <= GW_HEADING_RAD
                            or ang >= math.pi - GW_HEADING_RAD):
                        continue
            room = hit[2] - ROW_HOLD_STANDOFF_M
            if worst is None or room < worst[0]:
                worst = (room, rid, hit[3])
        if worst is None:
            return None
        room, rid, peer_travel = worst
        if room <= ACQ_STOP_TOL_M and peer_travel <= GW_PEER_NEAR_M:
            # A brief hold while the mover actually passes. A hard stop for
            # a crosser still metres out parked a body in its (disc)
            # envelope and bred STOP<->STOP 2-cycles (measured: 16 deadlock
            # firings and 11 deliveries on seed 5, 0-1 and 15-17 without).
            # Structural, brief wait (the mover is within GW_PEER_NEAR_M
            # of the crossing): suppress the no-cycle deadlock timeout the
            # same way an acquisition wait does.
            self._acq_waiting = True
            return Permit(action="STOP", speed_scale=0.0,
                          reason=f"give way: holding short for {rid}",
                          blocking_robot=rid)
        if room <= ACQ_SLOW_DISTANCE_M:
            return Permit(action="SLOW", speed_scale=MIN_SPEED_SCALE,
                          reason=f"give way: approaching hold point for {rid}",
                          blocking_robot=rid)
        return None

    # ------------------------------------------------- make-way (spec 3)
    def _zone_wait_on_me(self, st: RobotState) -> bool:
        """True when the peer's wait is a ZONE wait: it lists a ranked zone I
        hold, request or stand in - the zone protocol resolves that, and a
        make-way would only vacate ground the zone still bars it from
        (measured +3 deliveries from this skip).

        EXCEPTION (measured livelock, scenario e / AC12): a zone I merely
        STAND IN while parked - no hold, no request, no route - is ground the
        zone protocol can never clear; the occupancy rule blocks the zone for
        the waiter forever. Make-way is the only resolution, so that case is
        NOT a zone wait."""
        inside = set(self._inside_zones())
        transiting = bool(self.path) or self.goal_cell is not None
        for z in st.intent.zones:
            if not traffic.is_ranked(self.grid, z):
                continue
            if self.arbiter.state.get(z, FREE) != FREE:
                return True          # I hold/request it: protocol resolves it
            if z in inside and transiting:
                return True          # I am driving through: I clear it myself
        return False

    def _maybe_make_way(self, now: float) -> Optional[Permit]:
        """Start a make-way manoeuvre (via the retreat machinery) when a
        higher-priority peer has been WAITING on me for MW_TRIGGER_WAIT_S and
        I stand in its corridor; INVERSION: when a LOWER peer and I have been
        mutually stuck for MW_INVERT_S, the higher one (me) moves."""
        if self._retreat is not None or self.state.status == FAULT:
            return None
        beneficiary = None
        for rid, st in self.registry.alive().items():
            if st.status != WAITING or st.waiting_for != self.me:
                continue
            if now < self._mw_cooldown_until.get(rid, 0.0):
                continue
            higher = self._peer_outranks_me(st, now)
            if higher and st.waiting_time >= MW_TRIGGER_WAIT_S:
                if self._zone_wait_on_me(st):
                    continue
                corridor, _ = self._peer_corridor(st, now, ROW_PEER_LOOK_M)
                me = self.state.pose
                dmin = min(math.hypot(me.x - pt[0], me.y - pt[1])
                           for pt in corridor)
                if dmin > ROW_CORRIDOR_M + 0.3:
                    continue
                if (dmin >= MW_CLEAR_POCKET_M
                        and self.grid.world_to_cell(me.x, me.y)
                        in self._pocket_grade_cells()):
                    # Already parked in an engineered two-abreast spot and
                    # clear of the corridor by the pocket margin: the layout
                    # has no better place for me, so pulling out would only
                    # re-block the aisle (measured). The waiter reroutes
                    # around me (its planner treats me as an obstacle).
                    continue
                beneficiary = st
                break
            if (not higher and st.waiting_time >= MW_INVERT_S
                    and self.state.waiting_for == rid
                    and self.state.waiting_time >= MW_INVERT_S):
                # Inversion graft: the design-1 no-target trace showed the
                # lower robot can be boxed in; the higher one yields instead.
                self.log(f"make-way INVERSION: yielding to lower-priority "
                         f"{rid} after mutual {MW_INVERT_S:.0f}s stall")
                beneficiary = st
                break
        if beneficiary is None:
            return None
        rid = beneficiary.robot_id
        if (self._mw_fails.get(rid, 0) >= MW_FAILS_TO_DEADLOCK
                or now - self._mw_last_try.get(rid, -1e9) < MW_RETRY_S):
            return None                 # hand the pair to the deadlock layer
        self._mw_last_try[rid] = now
        if self._start_make_way(beneficiary, now):
            self._mw_fails.pop(rid, None)
            self.makeway_count += 1
            permit = self._retreat_permit(now)
            if permit is not None:
                return permit
            return None
        self._mw_fails[rid] = self._mw_fails.get(rid, 0) + 1
        self.makeway_fail_count += 1
        return None

    # ------------------------------------------- partition stop (spec 4)
    def _partition_permit(self, now: float) -> Optional[Permit]:
        """After T_PART_S with no state from a peer, STOP whenever its reach
        (frozen pose + v_max*age, capped REACH_CAP) touches my next
        PART_LOOKAHEAD_M of path: in a partition nobody can renegotiate, so
        both sides hold short (the plan's partition rule)."""
        if not self.path:
            return None
        mine = None
        for rid, st in self.registry.peers.items():
            if not st.alive:
                continue            # a faulted peer is a planner obstacle
            age = now - self.registry.last_seen.get(rid, st.rx_time)
            if age <= T_PART_S or age >= T_GONE_S:
                continue
            reach = min(safety.V_MAX_MPS * age, 1.0) + PART_BODY_M
            if mine is None:
                mine = self._my_lookahead(PART_LOOKAHEAD_M)
                mine.insert(0, (self.state.pose.x, self.state.pose.y, 0.0))
            for (x, y, _cum) in mine:
                if math.hypot(x - st.pose.x, y - st.pose.y) <= reach:
                    return Permit(
                        action="STOP", speed_scale=0.0,
                        reason=(f"partition: {rid} silent {age:.1f}s and its "
                                f"reach touches my path"),
                        blocking_robot=rid)
        return None

    def _zone_permit(self, now: float) -> Permit:
        # Ranked traffic zones first: acquisition point + rank order
        # (core/traffic.py). Unranked legacy zones keep path-order gating.
        ranked = self._traffic_permit(now)
        if ranked is not None:
            return ranked
        if not self._reservations_on:
            # ROAD MODEL: no zone negotiation of any kind.
            return self._speed_from_ttc(now)

        zone_id = self._next_zone()
        if zone_id is None:
            return self._speed_from_ttc(now)

        dist = self._distance_to_zone_m(zone_id)
        if dist > ZONE_REQUEST_DISTANCE_M:
            return self._speed_from_ttc(now)

        relevant = self._required_granters(zone_id, now)
        if self.arbiter.state.get(zone_id) not in ("REQUESTING", "HELD"):
            # One request path whether or not anybody competes (the old
            # 'nobody competes' shortcut was a second, unguarded request).
            t_in, t_out = self._zone_window(zone_id, now)
            traffic.request(self.grid, self.arbiter, zone_id,
                            self.state.priority_score, t_in, t_out, now)

        try:
            if self.arbiter.may_enter(zone_id, relevant, now=now):
                if dist < ZONE_ENTRY_DISTANCE_M:
                    self._maybe_release_passed_zones()
                # Holding a zone must not override continuous proximity: two
                # robots holding DIFFERENT zones can still close head-on in
                # the unzoned space between them.
                ttc = self._speed_from_ttc(now)
                if ttc.action in ("STOP", "SLOW"):
                    ttc.zone_id = zone_id
                    return ttc
                return Permit(action="GO", speed_scale=1.0,
                              reason=f"zone {zone_id} acquired", zone_id=zone_id)
        except ZoneTimeout:
            # Decaying, mild penalty: the permanent 50/cell of the past drove
            # A* into the impassable 0.2 m side strips after one timeout.
            self._zone_penalties[zone_id] = 8.0
            self._zone_penalty_until[zone_id] = now + 30.0
            self.replan(avoid_zones={zone_id}, now=now)
            return Permit(action="REROUTE", speed_scale=MIN_SPEED_SCALE,
                          reason=f"zone {zone_id} timed out, rerouting",
                          zone_id=zone_id)

        # Zone not yet granted. Choose the LEAST disruptive response.
        blocker, free_at = self._zone_blocker(zone_id, now)

        if free_at is not None and dist > 0.1:
            # Can I arrive just after they leave, without stopping?
            t_avail = max(0.05, free_at - now)
            needed_speed = dist / t_avail
            if needed_speed < self.v_nominal:
                scale = max(MIN_SPEED_SCALE,
                            min(1.0, needed_speed / self.v_nominal))
                return Permit(action="SLOW", speed_scale=scale,
                              reason=(f"slowing for {blocker} at {zone_id} "
                                      f"(clear in {t_avail:.1f}s)"),
                              blocking_robot=blocker, zone_id=zone_id)

        if dist > ZONE_ENTRY_DISTANCE_M:
            return Permit(action="SLOW", speed_scale=MIN_SPEED_SCALE,
                          reason=f"approaching contested {zone_id}",
                          blocking_robot=blocker, zone_id=zone_id)

        return Permit(action="STOP", speed_scale=0.0,
                      reason=f"waiting for {zone_id}",
                      blocking_robot=blocker, zone_id=zone_id)

    # ------------------------------------------------------ traffic layer
    def _inside_zones(self) -> List[str]:
        """Ranked zones my own pose lies in."""
        cell = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        return traffic.zones_at(self.grid, cell)

    def _occupies(self, zone_id: str, sigma_scale: float = 1.0) -> bool:
        """Does MY hitbox (believed pose, inflated by K_SIG*sigma - the same
        rule traffic.zone_occupants applies to peers) still overlap the zone?
        The R10 release gate: a ranked zone stays held until this is False.

        sigma_scale scales MY sigma before the K_SIG inflation: the RELEASE
        check uses RELEASE_SIGMA_SCALE (effective 1*sigma, not 2*sigma) so a
        segment is handed back ~K_SIG*sigma/2 earlier - entry is additionally
        occupancy-gated by the peers' own full-K_SIG check, so this only
        shortens the measured 5.0 s junction-restart latency (AC12 bound
        1.5 s), it cannot create a same-segment co-occupancy on its own."""
        p = self.state.pose
        info = (p.x, p.y, p.theta,
                max(0.0, float(self.state.loc_sigma_lat or 0.0)) * sigma_scale)
        return bool(traffic.zone_occupants(self.grid, zone_id,
                                           {self.me: info}))

    def _peer_positions(self, now: float) -> Dict[str, tuple]:
        """Occupancy inputs: NOT just alive peers - a SILENT/DEAD peer's
        frozen body still occupies its zone (sih-57: a stale-but-present
        peer must keep its exclusion until its hitbox is provably out).
        (x, y, theta, sigma) tuples so traffic can rasterise the hitbox."""
        out = {}
        for rid, st in self.registry.peers.items():
            if now - self.registry.last_seen.get(rid, st.rx_time) >= T_GONE_S \
                    and not st.alive:
                continue            # long gone AND self-faulted: reclaimed
            pose = self.registry.extrapolated_pose(rid, now) or st.pose
            out[rid] = (pose.x, pose.y, pose.theta,
                        max(0.0, float(st.loc_sigma_lat or 0.0)))
        return out

    def _granter_ids(self, now: float) -> Set[str]:
        """EVERY known peer with alive=True and not GONE: the required
        granters of every ranked zone (merged spec 4). peers.granters() when
        the registry has it (HITBOX builder), else computed here."""
        g = getattr(self.registry, "granters", None)
        if callable(g):
            try:
                return set(g(now))
            except TypeError:
                pass
        out: Set[str] = set()
        for rid, st in self.registry.peers.items():
            if not st.alive:
                continue
            if now - self.registry.last_seen.get(rid, st.rx_time) >= T_GONE_S:
                continue
            out.add(rid)
        return out

    def _required_granters(self, zone_id: str, now: float) -> Set[str]:
        """Unranked zones: peers whose intent lists the zone (one round trip
        when uncontested). Ranked zones: EVERY alive-and-not-GONE peer plus
        every hitbox-cell occupant - a SILENT peer therefore blocks entry
        (strict partition) until it speaks again or goes GONE."""
        needing = self.registry.peers_needing_zone(zone_id)
        if not traffic.is_ranked(self.grid, zone_id):
            return needing
        occ = traffic.zone_occupants(self.grid, zone_id,
                                     self._peer_positions(now))
        return traffic.required_granters(needing | self._granter_ids(now),
                                         occ)

    def _committed_top(self, inside) -> Optional[tuple]:
        """Highest rank key among zones I hold AND am physically inside
        (those cannot be backed off)."""
        keys = [traffic.rank_key(self.grid, z) for z in self.arbiter.held_zones()
                if z in inside and traffic.is_ranked(self.grid, z)]
        return max(keys) if keys else None

    def _back_off_ranked(self, inside, above: Optional[str] = None) -> None:
        """Release my ranked zones that I have not entered: those ranked
        above `above` (so it can be requested in order), or - for a lease
        back-off - all of them that rank above every zone I am committed
        inside. A zone ranked below a committed one is never given back:
        re-acquiring it later would have to break the rank order."""
        top = self._committed_top(inside) if above is None else None
        for z, st in list(self.arbiter.state.items()):
            if st == FREE or z in inside or not traffic.is_ranked(self.grid, z):
                continue
            key = traffic.rank_key(self.grid, z)
            if above is not None and key <= traffic.rank_key(self.grid, above):
                continue
            if top is not None and key < top:
                continue
            self.log(f"traffic: backing off {z} (rank order)")
            self.arbiter.release(z)
            self._held_since.pop(z, None)

    def _drop_stale_route_claims(self, why: str) -> None:
        """Release every zone claim (HELD or REQUESTING, ranked or not) that
        my body is not physically inside. Used when the robot provably cannot
        use the claims (no valid route after REPLAN_FAIL_RELEASE_K straight
        failures): releasing flushes my deferred queues, so peers that CAN
        move get their grants instead of being pinned by a stuck claimant."""
        inside = set(self._inside_zones())
        for zid, st in list(self.arbiter.state.items()):
            if st == FREE:
                continue
            if st == HELD and zid in inside:
                continue            # traffic rule 4: never release while in it
            self.log(f"{why}: releasing stale claim {zid}")
            self.arbiter.release(zid)
            self._held_since.pop(zid, None)

    def _retreat_kept_zones(self, inside) -> Set[str]:
        """Held ranked zones a retreat must not give back: those the saved
        task route still needs that rank BELOW a zone I hold and am inside.
        Releasing one (it is not on the refuge route) would force the resumed
        route to re-request it while committed above it - hold-and-wait out
        of rank order. They go as soon as I leave the committed zone."""
        r = self._retreat
        if r is None or not r.get("route_zones"):
            return set()
        top = self._committed_top(inside)
        if top is None:
            return set()
        return {z for z in r["route_zones"]
                if self.arbiter.state.get(z) == HELD
                and traffic.is_ranked(self.grid, z)
                and traffic.rank_key(self.grid, z) < top}

    def _ranked_lease_standoff(self, zone_id: str, now: float) -> bool:
        """A peer that needs zone_id is physically inside a ranked zone I hold
        or request: it can never get past me, so I must back off."""
        mine = {z for z, st in self.arbiter.state.items()
                if st != FREE and traffic.is_ranked(self.grid, z)}
        occ = traffic.occupants_by_zone(self.grid, self._peer_positions(now))
        needers = self.registry.peers_needing_zone(zone_id)
        return any(rid in occ.get(z, ()) for z in mine for rid in needers)

    def _traffic_status(self, path: List, cur_i: int, inside: Set[str],
                        occ: Dict[str, Set[str]],
                        convoy: Set[str] = frozenset(),
                        now: Optional[float] = None):
        """(required, acquired, acquisition index) for `path` from cur_i.
        No side effects. A zone is acquired if I hold it and no peer is
        physically inside it (or I am inside it too: I am already there).
        A required zone my pose is in WITHOUT holding it (drifted over the
        boundary, grandfathered after a release) is not acquired, and the
        acquisition point is right here: stop and acquire, never drive on.

        R10 direction-aware exception: a CONVOY zone (_convoy_zones - every
        occupant and claimant is same-direction) counts acquired as soon as
        my own request is queued (REQUESTING or HELD): a same-direction
        leader's claim must not stop a follower; the envelope spaces us.
        A2 don't-block-the-box: a junction I hold but whose exit stretch is
        blocked by a stopped robot does NOT count acquired - I wait at the
        standoff and leave the cross-flow free."""
        required_all = traffic.required_zones(self.grid, path[cur_i:],
                                              self._eff_mode)
        acquired = {z for z in required_all
                    if (self.arbiter.state.get(z) == HELD
                        and (z in inside or not occ.get(z)))
                    or (z in convoy and self.arbiter.state.get(z)
                        in (REQUESTING, HELD))}
        if now is not None:
            acquired -= {z for z in acquired if z not in inside
                         and self._junction_exit_blocked(z, path, cur_i, now)}
        # R10: acquire only the WINDOW the next stretch of path needs (the
        # corridor segments up to the next junction, plus anything rank
        # order forces in), never the whole route's zone set - so one robot
        # never locks the entire corridor end to end.
        window = traffic.zone_window(self.grid, path[cur_i:], self._eff_mode,
                                     skip=acquired)
        required = traffic.rank_sorted(self.grid,
                                       sorted(acquired) + list(window))
        acq = None
        if len(acquired) < len(required):
            if any(z in inside and z not in acquired for z in required):
                acq = cur_i
            else:
                # Acquired zones count as enterable: the stop line is the
                # standoff before the first zone still to ACQUIRE, not the
                # boundary of a held segment the robot may drive into (R10).
                acq = traffic.acquisition_point(self.grid, path, cur_i,
                                                set(inside) | acquired,
                                                self._eff_mode)
        return required, acquired, acq

    def _drop_bay_detour(self, now: float, inside: Set[str]) -> None:
        """Everything is acquired: the wait-bay detour has served its purpose.
        Replan straight to the goal (from the bay, or from wherever I am if I
        got the zones on the way), so the robot never drives the bay loop -
        a route that doubles back on itself, which a pure-pursuit follower
        handles badly. The new route may only use zones I hold or am in."""
        if not self._bay_detour or self.goal_cell is None:
            return
        bays = set(getattr(getattr(self.grid, "traffic", None),
                           "wait_bays", {}).values())
        if not any(c in bays for c in self.path):
            self._bay_detour = False
            return
        # R10: the straight route may pass through DEFERRED segments - they
        # are acquired later at their junction standoffs (the acquisition
        # machinery stops the robot before any ranked zone it does not hold),
        # so the replan is no longer restricted to zones already held.
        saved = (list(self.path), self.intent)
        if self.replan(now=now):
            self.log("traffic: zones acquired - dropping the wait-bay detour")
            self._bay_passed = True
        else:
            self.path, self.intent = saved
        self._bay_detour = False

    def _acq_distance(self, cur_i: int, acq: int, inside: Set[str]) -> float:
        """Path distance to the acquisition point. While this route's wait
        bay has not been 'passed' (everything acquired), the bay stays the
        acquisition point even if the nearest-index has jumped past it onto
        the leg that doubles back beside it; a robot more than BAY_REACHED_M
        past the bay falls back to the zone stand-off (it never enters)."""
        dist = traffic.path_distance_m(self.grid, self.path, cur_i, acq)
        if self._bay_passed:
            return dist
        bays = set(getattr(getattr(self.grid, "traffic", None),
                           "wait_bays", {}).values())
        if not bays:
            return dist
        k = traffic.first_new_zone_index(self.grid, self.path, cur_i, inside,
                                         self._eff_mode)
        if k is None:
            return dist
        b = next((i for i in range(k - 1, -1, -1) if self.path[i] in bays),
                 None)
        if b is None:
            return dist
        bx, by = self.grid.cell_to_world(self.path[b])
        if math.hypot(self.state.pose.x - bx,
                      self.state.pose.y - by) <= BAY_REACHED_M + 1e-6:
            self._bay_reached = True
        if self._bay_reached:
            # Reached once: wait here. Pose noise pushing the belief more
            # than BAY_REACHED_M away must not turn into SLOW/GO along the
            # loop's return leg towards the zone still being acquired.
            return 0.0
        if cur_i <= b:
            return traffic.path_distance_m(self.grid, self.path, cur_i, b)
        return dist

    def _entry_occupants(self, occ: Dict[str, Set[str]],
                         now: float) -> Dict[str, Set[str]]:
        """Inside-beats-stuck tiebreak on the ENTRY occupancy gate.

        After ACQ_INSIDE_TIEBREAK_S stopped at an acquisition point, a ranked
        zone I HOLD no longer counts as occupied by a STOPPED peer whose
        believed CENTRE is outside it: that peer is queued/stuck next to the
        boundary and only its K_SIG*sigma-inflated hitbox pokes in (0.4 m at
        the SPINE-fallback sigma 0.2 - the measured webots_final.log wedge,
        where the holder of SPINE_N stood 'acquiring SPINE_N' for 237 s).
        Mutual exclusion is untouched - I hold the mutex, so the peer cannot
        legally enter before me - and contact safety stays with the envelope,
        which keeps the full-sigma live-gap check while I drive in. A MOVING
        occupant, a centre-inside occupant, and every occupant of a zone I do
        not hold keep the full inflated veto."""
        waited_out = (self._acq_wait_since is not None
                      and now - self._acq_wait_since > ACQ_INSIDE_TIEBREAK_S)
        if not waited_out and not self._tiebreak_zones:
            return occ
        out: Dict[str, Set[str]] = {}
        engaged: Set[str] = set()
        for zid, who in occ.items():
            if self.arbiter.state.get(zid) != HELD or not (
                    waited_out or zid in self._tiebreak_zones):
                out[zid] = set(who)
                continue
            zone = self.grid.zones.get(zid)
            keep = set()
            for rid in who:
                st = self.registry.peers.get(rid)
                if st is None or abs(getattr(st, "v", 0.0)) >= 0.05:
                    keep.add(rid)
                    continue
                pose = self.registry.extrapolated_pose(rid, now) or st.pose
                centre = self.grid.world_to_cell(pose.x, pose.y)
                if zone is None or centre in zone.cells:
                    keep.add(rid)
                else:
                    engaged.add(zid)
                    if zid not in self._tiebreak_zones:
                        self.log(f"traffic: entering held {zid} past stopped "
                                 f"{rid} (centre outside; inside-beats-stuck)")
            if keep:
                out[zid] = keep
        self._tiebreak_zones = engaged
        return out

    def _traffic_permit(self, now: float) -> Optional[Permit]:
        """Ranked acquisition (traffic rules 2-3). None = every ranked zone
        my remaining path needs is acquired (or there are none).

        ROAD MODEL (user supersede): with traffic.reservations False the
        whole reservation layer is OFF - no zone is ever requested, held or
        waited for (zones_held stays EMPTY); the only structural stop left
        is the box-junction line (_box_permit): don't-block-the-box +
        right-of-way, with the envelope as THE safety mechanism."""
        if not self.path or not traffic.has_ranked_zones(self.grid):
            return None
        if not self._reservations_on:
            return self._box_permit(now)
        cur_i = self._path_index_near_current()
        inside = set(self._inside_zones())
        occ_raw = traffic.occupants_by_zone(self.grid,
                                            self._peer_positions(now))
        convoy = self._convoy_zones(self.path, cur_i, occ_raw, now)
        occ = self._entry_occupants(occ_raw, now)
        required, acquired, acq = self._traffic_status(self.path, cur_i,
                                                       inside, occ,
                                                       convoy, now)
        if acq is None:
            self._bay_passed = True             # everything acquired
            self._drop_bay_detour(now, inside)
            return None
        if not self._bay_passed:
            # R10: the bay guarded only the route's FIRST window. Once that
            # window is acquired the bay has served its purpose - later
            # windows are acquired at their junction standoffs while
            # driving, never by waiting at the bay again.
            w0 = traffic.zone_window(self.grid, self.path[cur_i:],
                                     self._eff_mode)
            if w0 and all(z in acquired for z in w0):
                self._bay_passed = True
                self._drop_bay_detour(now, inside)
                cur_i = self._path_index_near_current()
                required, acquired, acq = self._traffic_status(
                    self.path, cur_i, inside, occ, convoy, now)
                if acq is None:
                    return None
        dist = self._acq_distance(cur_i, acq, inside)

        pending = None
        for z in required:                      # ascending rank
            if z in acquired:
                continue
            pending = z
            if dist > ACQ_REQUEST_DISTANCE_M:
                break                           # not yet: just drive on
            if self.arbiter.state.get(z, FREE) == FREE:
                self._back_off_ranked(inside, above=z)
                t_in, t_out = self._zone_window(z, now)
                # Zones I hold and am physically inside cannot be backed off.
                # If one outranks z (my route changed while I was inside it),
                # the request is a logged rank exception; the hold-lease
                # back-off of an un-entered holder and the deadlock retreat
                # are the backstops for the hold-and-wait it can create.
                committed = {h for h in self.arbiter.held_zones()
                             if h in inside}
                if traffic.out_of_order(self.grid, z, self.arbiter.state,
                                        ignore=committed) == [] and \
                        traffic.out_of_order(self.grid, z, self.arbiter.state):
                    self.log(f"traffic: RANK EXCEPTION requesting {z} while "
                             f"inside {sorted(committed)}")
                if not traffic.request(self.grid, self.arbiter, z,
                                       self.state.priority_score,
                                       t_in, t_out, now,
                                       allow_above=committed):
                    break
            granters = self._required_granters(z, now)
            if (not granters and not self.registry.peers
                    and self._t_boot is not None
                    and now - self._t_boot < BOOT_QUORUM_GRACE_S):
                # Boot race: no peer heard yet, so the quorum is vacuously
                # empty - hold at the acquisition point for the grace
                # instead of self-granting (measured 12.2 s double-hold).
                break
            try:
                ok = self.arbiter.may_enter(z, granters, now=now)
            except ZoneTimeout:
                # Waiting is structural here (rank order bounds it): no
                # reroute, just re-request in order on the next tick.
                self.log(f"traffic: {z} request timed out; re-requesting")
                ok = False
            if not ok or occ.get(z):
                # R10 convoy entry: a same-direction leader's claim or body
                # is not a wall - with my FIFO request queued I may follow
                # it in; the envelope keeps the spacing (car-following).
                if not (z in convoy and self.arbiter.state.get(z)
                        in (REQUESTING, HELD)):
                    break                       # next zone only once z is ours
                if z not in self._convoy_logged:
                    self._convoy_logged.add(z)
                    self.log(f"traffic: convoy entry into {z} behind "
                             f"same-direction traffic "
                             f"(occupants {sorted(occ.get(z, ()))})")
            if z not in inside and self._junction_exit_blocked(
                    z, self.path, cur_i, now):
                break       # A2: never enter a junction with a blocked exit
            acquired.add(z)
            pending = None
        if pending is None and len(acquired) == len(required):
            self._bay_passed = True
            self._drop_bay_detour(now, inside)
            return None

        if pending is None:
            pending = next(z for z in required if z not in acquired)
        missing = (self._required_granters(pending, now)
                   - self.arbiter.grants.get(pending, set()))
        # Wait-for edge: an occupant, else the missing granter most likely to
        # be the HOLDER - one that is not itself queued (WAITING/YIELDING).
        # Pointing at a fellow queuer makes a fake 2-cycle with it.
        alive = self.registry.alive()
        queued = {rid for rid in missing
                  if rid in alive and alive[rid].status in (WAITING, YIELDING)}
        blocker = (sorted(occ.get(pending, ()))
                   or sorted(missing, key=lambda r: (r in queued, r))
                   or [self._zone_blocker(pending, now)[0]])[0]
        if (not occ.get(pending) and now - self.arbiter.req_time.get(
                pending, now) < ACQ_EDGE_DELAY_S):
            # A request in its first round trip: the missing granter is
            # most likely just about to answer. Publishing it as a wait-for
            # edge makes transient 2-cycles with the holder (the 'deadlock'
            # the robot queued behind it then sees).
            blocker = ""
        why = f"acquiring {pending} (order {'<'.join(required)})"
        if dist <= ACQ_STOP_TOL_M:
            self._acq_waiting = True
            return Permit(action="STOP", speed_scale=0.0,
                          reason=f"at acquisition point: {why}",
                          blocking_robot=blocker, zone_id=pending)
        if dist <= ACQ_SLOW_DISTANCE_M:
            return Permit(action="SLOW", speed_scale=MIN_SPEED_SCALE,
                          reason=f"approaching acquisition point: {why}",
                          blocking_robot=blocker, zone_id=pending)
        ttc = self._speed_from_ttc(now)
        ttc.zone_id = pending
        return ttc

    def _speed_from_ttc(self, now: float) -> Permit:
        """Speed shaping only. TTC <= 1.0 s is a STOP owned by the safety
        envelope for every robot, priority winner included; it is left out
        here so it cannot override the envelope's escape rule."""
        worst = None
        for c in self.last_conflicts:
            if c.kind != "TTC" or c.t_conflict - now <= safety.STOP_HORIZON_S:
                continue
            if worst is None or c.t_conflict < worst.t_conflict:
                worst = c
        if worst is None:
            return Permit(action="GO", speed_scale=1.0, reason="clear")

        ttc = worst.t_conflict - now
        if ttc < safety.SLOW_HORIZON_S:
            return Permit(action="SLOW",
                          speed_scale=max(MIN_SPEED_SCALE,
                                          ttc / safety.SLOW_HORIZON_S),
                          reason=f"TTC {ttc:.1f}s with {worst.peer_id}",
                          blocking_robot=worst.peer_id)
        return Permit(action="GO", speed_scale=1.0, reason="clear")

    # ------------------------------------------------------------- baseline
    def _baseline_permit(self, peers, now: float) -> Permit:
        """Stop-and-wait: the comparison system.

        No intent sharing, no priority, no zones, no speed modulation.
        Purely reactive, and strictly binary: GO or STOP. This is what the
        proposed system is measured against.
        """
        R_STOP = 1.0
        nearest, dmin = "", 1e9
        for pid, p in peers.items():
            d = ((p.pose.x - self.state.pose.x) ** 2
                 + (p.pose.y - self.state.pose.y) ** 2) ** 0.5
            if d < dmin:
                dmin, nearest = d, pid
        if dmin < R_STOP:
            if self._wait_started is None:
                self._wait_started = now
            self.state.waiting_time = now - self._wait_started
            self.state.status = WAITING
            self.state.waiting_for = nearest
            if self.state.waiting_time > 10.0:
                return Permit(action="REROUTE", speed_scale=0.0,
                              reason="baseline deadlock recovery",
                              blocking_robot=nearest)
            return Permit(action="STOP", speed_scale=0.0,
                          reason=f"baseline: {nearest} within {R_STOP}m",
                          blocking_robot=nearest)
        self._wait_started = None
        self.state.waiting_time = 0.0
        self.state.waiting_for = ""
        self.state.status = MOVING if self.path else IDLE
        return Permit(action="GO", speed_scale=1.0, reason="baseline: clear")

    # --------------------------------------------------------------- yielding
    def _yield_permit(self, dl: dict, now: float) -> Permit:
        """I am the deadlock victim: physically clear the way.

        The old version replanned with a soft zone penalty, which almost
        always returned the same route (live peers are not planner
        obstacles), then emitted REROUTE - which velocity_gate treats as zero
        speed. The victim froze and was re-elected every tick. Now: a reroute
        counts only if it genuinely avoids the blocker; otherwise retreat to a
        refuge off every peer's path. Both move under GO permits.
        """
        cycle = dl.get("cycle", [])
        blockers = [r for r in cycle if r != self.me]
        if not blockers and self.state.waiting_for:
            blockers = [self.state.waiting_for]

        # deadlock.py 'escalate': yielding has not worked for T_YIELD_S, so
        # do not try another reroute - retreat.
        escalate = bool(dl.get("escalate"))
        # A reroute that left me where I was (e.g. the envelope kept me
        # stopped) resolved nothing; a second one would loop. Retreat instead.
        pose = self.state.pose
        lr = self._last_reroute
        stale = (lr is not None and now - lr[0] < REROUTE_RETRY_WINDOW_S
                 and math.hypot(pose.x - lr[1], pose.y - lr[2])
                 < REROUTE_MIN_PROGRESS_M)
        avoid = self._blocker_cells(blockers, now)
        saved = (list(self.path), self.intent)
        # Rule 6: the victim's reroute may cross the opposing half laterally
        # (rule 3: a band is always escapable sideways) but never DRIVE
        # ALONG it - see YIELD_REROUTE_LANE_CONFINED. A wrong-way reroute is
        # rejected here and the victim retreats instead; the gated overtake
        # is the only legal along-the-opposing-lane pass.
        if (not escalate and not stale and avoid and self.goal_cell is not None
                and self.replan(now=now, extra_blocked=(
                    avoid | self._tight_cells()
                    | self._reroute_forbidden_zone_cells(saved[0])))
                and not (YIELD_REROUTE_LANE_CONFINED
                         and self._path_runs_against_a_lane(self.path))
                and self._reroute_traffic_ok(now)):
            self.log(f"deadlock victim: rerouted clear of {blockers}")
            self._last_reroute = (now, pose.x, pose.y)
            self._end_wait(now)
            self._deadlock_cooldown_until = now + DEADLOCK_COOLDOWN_S
            # This GO replaces the enveloped permit, so re-apply the envelope.
            permit = safety.most_restrictive(
                Permit(action="GO", speed_scale=RETREAT_SPEED_SCALE,
                       reason=f"deadlock victim: rerouted around {blockers}"),
                self._envelope(now))
            permit.deadlock_detected, permit.deadlock_cycle = True, cycle
            return permit
        if self.path or self._replan_fail_streak < REPLAN_FAIL_RELEASE_K:
            self.path, self.intent = saved
        else:
            # The blocker-avoiding reroute was the K-th straight failure:
            # restoring the saved (unexecutable) path here used to revive its
            # intent and with it every stale ranked claim, pinning the robots
            # that could still move (webots_final.log: the 'REPLAN FAILED ->
            # retreat -> resume' loop kept SPINE claims alive for minutes).
            # Keep the empty path: claims not under my body were already
            # released by replan(), and the robot holds/retreats off-route.
            self.log("deadlock victim: persistent replan failure - "
                     "keeping empty path (stale claims released)")
        if escalate or stale:
            self.log(f"deadlock victim: "
                     + (f"escalated after {self.deadlock.t_yield:.0f}s"
                        if escalate else "last reroute made no progress")
                     + f" - retreating from {blockers}")

        if self._start_retreat(blockers, now):
            permit = self._retreat_permit(now)
            if permit is not None:
                permit.deadlock_detected = True
                permit.deadlock_cycle = cycle
                return permit

        # No refuge reachable: keep the wait-for edge and let the node's
        # local YIELD manoeuvre try (fleet_agent_node._maybe_retreat).
        return Permit(action="YIELD", speed_scale=0.0,
                      reason="deadlock victim: no refuge reachable, yielding",
                      blocking_robot=blockers[0] if blockers else "",
                      deadlock_detected=True, deadlock_cycle=cycle)

    def _reroute_forbidden_zone_cells(self, old_path: List) -> Set[tuple]:
        """Ranked-zone cells a deadlock reroute may not use: zones I neither
        hold nor am inside that my old route did not need (a reroute must
        not cut through SPINE or a spur it never acquired), and zones that
        would have to be acquired below an entered zone's rank."""
        if not traffic.has_ranked_zones(self.grid):
            return set()
        inside = set(self._inside_zones())
        held = set(self.arbiter.held_zones())
        old_req = set(traffic.required_zones(self.grid, old_path,
                                             self._eff_mode))
        committed = [z for z in held & inside if traffic.is_ranked(self.grid, z)]
        top = (max(traffic.rank_key(self.grid, z) for z in committed)
               if committed else None)
        out: Set[tuple] = set()
        for zid in traffic.ranked_zone_ids(self.grid):
            if zid in held or zid in inside:
                continue
            if not traffic.in_mode(self.grid.zones[zid], self._eff_mode):
                # Not a mutex in the current effective mode (the SPINE
                # segments carry modes:[SPINE] and are lane-governed in
                # LANES): required_zones never lists them, so 'not in
                # old_req' wrongly forbade EVERY lane-mode reroute across
                # the spine (measured: the parked-robot pass and the aisle
                # head-ons could never replan around a blocker because the
                # whole band was painted forbidden).
                continue
            if (zid not in old_req or
                    (top is not None and traffic.rank_key(self.grid, zid) < top)):
                # Overlaps (J_AW inside SPINE) stay forbidden: each zone is
                # its own mutex.
                out |= self.grid.zones[zid].cells
        return out

    def _reroute_traffic_ok(self, now: float) -> bool:
        """The new path can be acquired in rank order and does not stop the
        victim at once at an acquisition point (then it is no reroute)."""
        if not self.path or not traffic.has_ranked_zones(self.grid):
            return True
        cur_i = self._path_index_near_current()
        inside = set(self._inside_zones())
        occ = traffic.occupants_by_zone(self.grid, self._peer_positions(now))
        required, _acquired, acq = self._traffic_status(self.path, cur_i,
                                                        inside, occ)
        if not traffic.rank_safe(self.grid, required,
                                 self.arbiter.held_zones(), inside):
            return False
        return (acq is None or traffic.path_distance_m(
            self.grid, self.path, cur_i, acq) >= REROUTE_MIN_FREE_RUN_M)

    def _unusable_zone_cells(self) -> Set[tuple]:
        """Ranked-zone cells a retreat may not enter: every ranked zone I
        neither hold nor am inside (a retreat never acquires zones, so it
        can never take one out of rank order)."""
        if not traffic.has_ranked_zones(self.grid):
            return set()
        usable = set(self.arbiter.held_zones()) | set(self._inside_zones())
        out: Set[tuple] = set()
        for zid in traffic.ranked_zone_ids(self.grid):
            if zid not in usable:
                out |= self.grid.zones[zid].cells
        return out

    # ------------------------------------------------------------ retreat
    def _disc_cells(self, x: float, y: float, radius: float) -> Set[tuple]:
        res = self.grid.resolution
        center = self.grid.world_to_cell(x, y)
        n = int(radius / res) + 1
        out = set()
        for dr in range(-n, n + 1):
            for dc in range(-n, n + 1):
                if math.hypot(dr, dc) * res <= radius:
                    out.add((center[0] + dr, center[1] + dc))
        return out

    def _peer_xy(self, rid: str, now: float):
        p = self.registry.peers.get(rid)
        if p is None:
            return None
        pose = self.registry.extrapolated_pose(rid, now) or p.pose
        return pose.x, pose.y

    def _tight_cells(self) -> Set[tuple]:
        """Free cells too close to a wall/rack to drive through (cached).

        The planner's clearance term makes these expensive but finite, so a
        recovery route that must avoid a blocker happily squeezes along a
        rack face - scraping it, slipping the wheels and losing localization.
        Recovery routes treat them as hard obstacles instead.
        """
        if getattr(self, "_tight_cache", None) is None:
            clearance = self.grid.clearance_m()
            self._tight_cache = {
                (r, c)
                for r in range(self.grid.height)
                for c in range(self.grid.width)
                if self.grid.static[r][c] == 0
                and clearance[r][c] < RETREAT_PASS_CLEARANCE_M}
        return self._tight_cache

    def _blocker_cells(self, blockers: List[str], now: float) -> Set[tuple]:
        """Blocker bodies plus their next few seconds of path, as cells."""
        out: Set[tuple] = set()
        for rid in blockers:
            xy = self._peer_xy(rid, now)
            if xy is None:
                continue
            out |= self._disc_cells(xy[0], xy[1], REROUTE_BLOCKER_CLEAR_M)
            p = self.registry.peers.get(rid)
            for cell, t_in, _t_out in p.intent.triples():
                if t_in > now + REROUTE_INTENT_HORIZON_S:
                    break
                cx, cy = self.grid.cell_to_world(cell)
                out |= self._disc_cells(cx, cy, REROUTE_INTENT_CLEAR_M)
        return out

    def _pocket_grade_cells(self) -> Set[tuple]:
        """Engineered two-abreast spots: yaml pockets + tier-1 refuge cells
        (retreat spur mouths and wait bays). Cached; static geometry."""
        if getattr(self, "_pocket_grade_cache", None) is None:
            tier1, _tier2 = traffic.designated_refuge_cells(
                self.grid, self.grid.clearance_m(), RETREAT_CELL_CLEARANCE_M)
            self._pocket_grade_cache = set(
                map(tuple, getattr(self.grid, "pockets", ()) or ())) \
                | set(tier1)
        return self._pocket_grade_cache

    def _ranked_cells(self) -> Set[tuple]:
        """Every cell of every ranked traffic zone (cached)."""
        if getattr(self, "_ranked_cells_cache", None) is None:
            out: Set[tuple] = set()
            for zid in traffic.ranked_zone_ids(self.grid):
                out |= self.grid.zones[zid].cells
            self._ranked_cells_cache = out
        return self._ranked_cells_cache

    def _find_refuge(self, now: float,
                     blockers: Sequence[str] = ()) -> Optional[tuple]:
        """Where a deadlock victim parks: a DESIGNATED refuge off every alive
        peer's path and body.

        Candidates come from the yaml traffic section: tier 1 = retreat
        cells (spur mouths beside the spine) and wait bays; tier 2 = refuge
        centre-line cells (aisle B, aisle A east). The nearest reachable one
        of the first non-empty tier wins. Reachability is breadth-first over
        drivable cells, never through a peer's body and never through a
        ranked zone I neither hold nor am inside (a retreat acquires no
        zone, so it cannot break the rank order). Only if no designated cell
        is reachable does the old local search run, and it never parks in a
        ranked zone (refuges in the spine's rack gaps blocked the corridor).

        The refuge must also actually CLEAR the blockers it yields to:
        cells within MW_CLEAR_CORRIDOR_M of a blocker's corridor (body +
        next ROW_PEER_LOOK_M of intent) are avoided first - the measured
        aisle-A livelock retreated to in-aisle refuges inside the contested
        corridor, so neither robot ever cleared the other and the standoff
        replayed every ~30 s. Only when no corridor-clear refuge is
        reachable does the old (any-refuge) choice apply.
        """
        clearance = self.grid.clearance_m()
        forbid: Set[tuple] = set()
        bodies: Set[tuple] = set()
        for rid, p in self.registry.alive().items():
            xy = self._peer_xy(rid, now)
            if xy is None:
                continue
            forbid |= self._disc_cells(xy[0], xy[1], RETREAT_PEER_CLEAR_M)
            bodies |= self._disc_cells(xy[0], xy[1], RETREAT_PEER_BODY_M)
            for cell in p.intent.cells:
                cx, cy = self.grid.cell_to_world(cell)
                forbid |= self._disc_cells(cx, cy, RETREAT_INTENT_CLEAR_M)
            # Never park on a peer's destination: robot_3 retreated onto
            # robot_2's pickup and robot_2 could never finish its route
            # (webots_userrestart7, 0 deliveries in 8 min).
            g = self._peer_goal_xy(p)
            if g:
                forbid |= self._disc_cells(g[0], g[1], MW_GOAL_CLEAR_M)
        tier1, tier2 = traffic.designated_refuge_cells(
            self.grid, clearance, RETREAT_CELL_CLEARANCE_M)
        # Blocker corridors at two margins: a POCKET-GRADE cell (yaml pocket,
        # retreat spur mouth or wait bay - the engineered two-abreast spots)
        # is acceptable at the pocket margin; anything else, tier-2 in-aisle
        # refuge lines included, needs the full corridor clearance.
        designated = set(map(tuple, getattr(self.grid, "pockets", ()) or ())) \
            | set(tier1)
        corr_full: Set[tuple] = set()
        corr_near: Set[tuple] = set()
        alive = self.registry.alive()
        for rid in blockers:
            st = alive.get(rid)
            if st is None:
                continue
            pts, _ = self._peer_corridor(st, now, ROW_PEER_LOOK_M)
            for pt in pts:
                corr_full |= self._disc_cells(pt[0], pt[1],
                                              MW_CLEAR_CORRIDOR_M)
                corr_near |= self._disc_cells(pt[0], pt[1],
                                              MW_CLEAR_POCKET_M)
        corr = (corr_full - designated) | corr_near
        no_go = self._recovery_no_go()
        # Strict lanes: the retreat ROUTE must never cut across the
        # blocker's corridor either - the measured sigma-0.3 head-on wedge
        # had the victim crossing the junction box 1.5 m AHEAD of the robot
        # it yielded to, stalling on the envelope mid-box and parking ON the
        # blocker's path. Blocking the blocker's body + imminent intent for
        # the search leaves exactly the own-lane/behind escapes.
        route_block = set(bodies)
        if self._lane_confinement_active() and blockers:
            route_block |= self._blocker_cells(list(blockers), now)
            route_block |= self._blocker_lane_cells(blockers, now)
        start = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)

        me = self.state.pose
        # Strict lanes: park spots must be lane-parkable - crossing zones
        # (junction boxes / turnarounds) are for moving THROUGH, never for
        # parking; band cells only in my own direction's lane or when
        # DESIGNATED (per-direction pockets / bays / retreat mouths).
        confine = self._lane_confinement_active()
        my_dir = self._my_travel_dir() if confine else None
        designated_park = self._pocket_grade_cells() if confine else ()

        def pick(avoid: Set[tuple]) -> Optional[tuple]:
            def parkable(cell) -> bool:
                if confine and not traffic.cell_lane_parkable(
                        self.grid, cell, my_dir, designated_park):
                    return False
                cx, cy = self.grid.cell_to_world(cell)
                return (cell not in forbid and cell not in bodies
                        and cell not in avoid
                        and math.hypot(cx - me.x,
                                       cy - me.y) >= RETREAT_MIN_MOVE_M
                        and clearance[cell[0]][cell[1]]
                        >= RETREAT_CELL_CLEARANCE_M)

            if tier1 or tier2:
                steps = self._retreat_bfs(start, RETREAT_DESIGNATED_SEARCH_M,
                                          route_block, no_go, clearance)
                for tier in (tier1, tier2):
                    cands = [c for c in tier if c in steps and parkable(c)]
                    if cands:
                        return min(cands, key=lambda c: (steps[c], c))

            # Fallback: nearest parkable cell, as before, but not in a zone.
            ranked = self._ranked_cells()
            steps = self._retreat_bfs(start, RETREAT_SEARCH_M, route_block,
                                      no_go, clearance)
            cands = [c for c in steps if parkable(c) and c not in ranked]
            return min(cands, key=lambda c: (steps[c], c)) if cands else None

        if corr:
            refuge = pick(corr)
            if refuge is not None:
                return refuge
        return pick(set())

    def _retreat_bfs(self, start, max_m: float, bodies: Set[tuple],
                     no_go: Set[tuple], clearance) -> Dict[tuple, int]:
        """Cell -> BFS ring index over cells a retreat can drive through."""
        max_steps = int(max_m / self.grid.resolution)
        steps = {start: 0}
        frontier = [start]
        for k in range(1, max_steps + 1):
            nxt = []
            for r, c in frontier:
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        cell = (r + dr, c + dc)
                        if cell in steps or not self.grid.is_static_free(cell):
                            continue
                        if (clearance[cell[0]][cell[1]] < RETREAT_PASS_CLEARANCE_M
                                or cell in bodies or cell in no_go):
                            continue
                        steps[cell] = k
                        nxt.append(cell)
            if not nxt:
                break
            frontier = nxt
        return steps

    def _start_retreat(self, blockers: List[str], now: float) -> bool:
        refuge = self._find_refuge(now, blockers)
        if refuge is None:
            return False
        saved_goal, saved = self.goal_cell, (list(self.path), self.intent)
        self.goal_cell = refuge
        bodies: Set[tuple] = set()
        for rid in blockers:
            xy = self._peer_xy(rid, now)
            if xy is not None:
                bodies |= self._disc_cells(xy[0], xy[1], RETREAT_PEER_BODY_M)
        # The retreat route may only use ranked zones I hold or am inside,
        # and (strict lanes) never the opposing lane NOR the blocker's own
        # corridor: the yielder reverses along its own lane / off-road
        # ground to its refuge - never across the path it is yielding to.
        extra = bodies | self._tight_cells() | self._recovery_no_go()
        if self._lane_confinement_active() and blockers:
            extra |= self._blocker_cells(list(blockers), now)
            extra |= self._blocker_lane_cells(blockers, now)
        if not self.replan(now=now, extra_blocked=extra):
            self.goal_cell = saved_goal
            self.path, self.intent = saved
            return False
        self._retreat = dict(cell=refuge, saved_goal=saved_goal,
                             blockers=list(blockers), phase="DRIVE",
                             t0=now, hold_t0=None,
                             lane_dir=self._my_travel_dir(),
                             route_zones=traffic.required_zones(
                                 self.grid, saved[0], self._eff_mode))
        rx, ry = self.grid.cell_to_world(refuge)
        self.log(f"deadlock victim: retreating to refuge {refuge} "
                 f"({rx:.2f},{ry:.2f}) to let {blockers} pass")
        self._end_wait(now)
        return True

    # ----------------------------------------------------------- make-way
    def _mw_bfs(self, start, bodies: Set[tuple], no_go: Set[tuple],
                clearance):
        """BFS over drivable cells to MW_MAX_M, with parents (the first-leg
        rule needs the actual route to each candidate)."""
        max_steps = int(MW_MAX_M / self.grid.resolution)
        steps = {start: 0}
        parent: Dict[tuple, tuple] = {}
        frontier = [start]
        for k in range(1, max_steps + 1):
            nxt = []
            for r, c in frontier:
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        cell = (r + dr, c + dc)
                        if cell in steps or not self.grid.is_static_free(cell):
                            continue
                        if (clearance[cell[0]][cell[1]] < MW_DRIVE_CLEAR_M
                                or cell in bodies or cell in no_go):
                            continue
                        steps[cell] = k
                        parent[cell] = (r, c)
                        nxt.append(cell)
            if not nxt:
                break
            frontier = nxt
        return steps, parent

    def _chain(self, parent, start, cell) -> List[tuple]:
        out = [cell]
        while cell != start and cell in parent:
            cell = parent[cell]
            out.append(cell)
        out.reverse()
        return out

    def _mw_required_clear_m(self, b: RobotState) -> float:
        """Distance from the beneficiary's corridor at which a PARKED me no
        longer trips its stationary-peer safety rule: its stationary stop
        threshold g_st = gap_stop(my sigma, its sigma) on the rect gap, plus
        both bodies' lateral extents (my circumscribed radius - final parking
        heading unknown - and its half-width along its corridor)."""
        gs = getattr(hitbox_mod, "gap_stop", None) if hitbox_mod else None
        msig = max(0.0, float(self.state.loc_sigma_lat or 0.0))
        bsig = max(0.0, float(b.loc_sigma_lat or 0.0))
        if gs is None:
            g_st = 0.12 + 2.0 * math.sqrt(msig * msig + bsig * bsig)
        else:
            g_st = gs(msig, bsig, 0.0, 0.0)
        r_me = getattr(hitbox_mod, "HB_R_CIRC", 0.286) if hitbox_mod else 0.286
        hw_b = getattr(hitbox_mod, "HB_HALF_W", 0.205) if hitbox_mod else 0.205
        return g_st + r_me + hw_b

    def _find_make_way_target(self, b: RobotState, now: float,
                              min_clear_m: float = 0.0) -> Optional[tuple]:
        """Where to 'make space / take some steps back' (R1): BFS within
        MW_MAX_M, parking clear of every higher-priority corridor BY DISTANCE
        (MW_CLEAR_CORRIDOR_M), of their goals (MW_GOAL_CLEAR_M), of peer
        bodies, and of ranked zones I neither hold nor occupy; the first
        MW_FIRST_LEG_M never approach any higher robot (the measured lock);
        designated pockets/bays/retreats preferred; +MW_AHEAD_COST when the
        target is ahead of the beneficiary (lateral/backward preferred)."""
        clearance = self.grid.clearance_m()
        me = self.state.pose
        alive = self.registry.alive()
        higher = {rid: st for rid, st in alive.items()
                  if rid == b.robot_id or self._peer_outranks_me(st, now)}
        corridors: Dict[str, list] = {}
        goals: Dict[str, tuple] = {}
        bodies: Set[tuple] = set()
        poses: List[tuple] = []
        for rid, st in alive.items():
            xy = self._peer_xy(rid, now) or (st.pose.x, st.pose.y)
            poses.append(xy)
            bodies |= self._disc_cells(xy[0], xy[1], MW_PEER_BODY_M)
            if rid in higher:
                corridors[rid], _ = self._peer_corridor(st, now,
                                                        ROW_PEER_LOOK_M)
            # Every live peer's goal, not just higher-priority ones: parking
            # on a lower robot's pickup strands it (robot_3 on robot_2's).
            g = self._peer_goal_xy(st)
            if g:
                goals[rid] = g
        no_go = self._recovery_no_go()
        start = self.grid.world_to_cell(me.x, me.y)
        steps, parent = self._mw_bfs(start, bodies, no_go, clearance)

        bdir = (math.cos(b.pose.theta), math.sin(b.pose.theta))
        tier1, tier2 = traffic.designated_refuge_cells(
            self.grid, clearance, MW_PARK_CLEAR_M)
        designated = set(tier1) | set(tier2)
        # ONLY the engineered off-route spots get the reduced two-abreast
        # margin: yaml pockets and tier-1 cells (retreat spur mouths, wait
        # bays). Tier-2 refuge CENTRE-LINE cells sit in the aisles - with
        # the reduced margin an idle robot legally re-parked 0.45 m off the
        # carrier's corridor, which still closes a 1.6 m aisle (measured:
        # the make-way peer walked (74,80)->(66,70) ahead of the carrier).
        pocketish = set(map(tuple, getattr(self.grid, "pockets", ()) or ())) \
            | set(tier1)
        # Strict lanes: same park rule as the refuges - never hold in a
        # crossing zone, never in a foreign lane (designated cells exempt).
        confine = self._lane_confinement_active()
        my_dir_lane = self._my_travel_dir() if confine else None
        designated_park = self._pocket_grade_cells() if confine else ()

        def valid(cell) -> bool:
            if confine and not traffic.cell_lane_parkable(
                    self.grid, cell, my_dir_lane, designated_park):
                return False
            cx, cy = self.grid.cell_to_world(cell)
            if clearance[cell[0]][cell[1]] < MW_PARK_CLEAR_M:
                return False
            if math.hypot(cx - me.x, cy - me.y) < 0.3:
                return False        # clears nothing
            if any(math.hypot(cx - px, cy - py) < MW_PEER_BODY_M
                   for px, py in poses):
                return False
            # Pocket-grade cells are two-abreast by construction and pass
            # with the reduced lateral margin; everything else keeps the
            # full corridor clearance.
            clear_m = (MW_CLEAR_POCKET_M if cell in pocketish
                       else MW_CLEAR_CORRIDOR_M)
            # sih-57 re-target: the caller demands the beneficiary's
            # stationary-rule clearance on top of the structural margins.
            clear_m = max(clear_m, min_clear_m)
            for rid in corridors:
                if any(math.hypot(cx - pt[0], cy - pt[1]) < clear_m
                       for pt in corridors[rid]):
                    return False
            for g in goals.values():
                if math.hypot(cx - g[0], cy - g[1]) < MW_GOAL_CLEAR_M:
                    return False
            # First-leg rule: never start by approaching a higher robot.
            cum, px, py = 0.0, me.x, me.y
            for step_cell in self._chain(parent, start, cell)[1:]:
                sx, sy = self.grid.cell_to_world(step_cell)
                cum += math.hypot(sx - px, sy - py)
                px, py = sx, sy
                if cum > MW_FIRST_LEG_M:
                    break
                for st in higher.values():
                    if (math.hypot(sx - st.pose.x, sy - st.pose.y)
                            < math.hypot(me.x - st.pose.x,
                                         me.y - st.pose.y) - 1e-6):
                        return False
            return True

        def cost(cell):
            cx, cy = self.grid.cell_to_world(cell)
            ahead = ((cx - b.pose.x) * bdir[0]
                     + (cy - b.pose.y) * bdir[1]) > 0.0
            tier = 0 if cell in designated else 1
            return (tier, steps[cell] + (MW_AHEAD_COST if ahead else 0), cell)

        cands = [c for c in steps if valid(c)]
        return min(cands, key=cost) if cands else None

    def _start_make_way(self, b: RobotState, now: float) -> bool:
        target = self._find_make_way_target(b, now)
        if target is None:
            self.log(f"make-way for {b.robot_id}: no valid target "
                     f"within {MW_MAX_M}m")
            return False
        saved_goal, saved = self.goal_cell, (list(self.path), self.intent)
        self.goal_cell = target
        bodies: Set[tuple] = set()
        for rid in self.registry.alive():
            xy = self._peer_xy(rid, now)
            if xy is not None:
                bodies |= self._disc_cells(xy[0], xy[1], MW_PEER_BODY_M)
        if not self.replan(now=now, extra_blocked=(
                bodies | self._tight_cells() | self._recovery_no_go())):
            self.goal_cell = saved_goal
            self.path, self.intent = saved
            return False
        self._retreat = dict(cell=target, saved_goal=saved_goal,
                             blockers=[b.robot_id], phase="DRIVE",
                             t0=now, hold_t0=None, make_way=True,
                             beneficiary=b.robot_id, b_still=None,
                             lane_dir=self._my_travel_dir(),
                             route_zones=traffic.required_zones(
                                 self.grid, saved[0], self._eff_mode))
        tx, ty = self.grid.cell_to_world(target)
        self.log(f"make-way: clearing {b.robot_id}'s corridor, pulling over "
                 f"to {target} ({tx:.2f},{ty:.2f})")
        self._end_wait(now)
        return True

    def _mw_blocked_retarget(self, r: dict, now: float) -> Optional[Permit]:
        """sih-57 live stall, fix (c)+(a): during a make-way HOLD, a
        beneficiary that is STATIONARY and broadcasting waiting_for == me for
        MW_BLOCKED_RETARGET_S is stopped BY my parked spot (its
        stationary-peer safety rule; the stale-beneficiary release
        deliberately excludes waiting_for == me, so nothing else can fire).
        Re-target farther at once - this time demanding the beneficiary's
        stationary-rule clearance (_mw_required_clear_m) on top of the pocket
        margin - and drive there. With no such target reachable, end the hold
        as a FAILED make-way: resuming turns me into a moving peer, which the
        stationary rule ignores, and the deadlock layer takes the pair. Both
        outcomes unblock within ~MW_BLOCKED_RETARGET_S instead of freezing
        the group to the MW_HOLD_MAX_S = 20 s cap."""
        st = self.registry.alive().get(r["beneficiary"])
        if st is None or abs(st.v) >= 0.05 or st.waiting_for != self.me:
            r.pop("b_blocked_t0", None)
            return None
        t0 = r.setdefault("b_blocked_t0", now)
        if now - t0 < MW_BLOCKED_RETARGET_S:
            return None
        r.pop("b_blocked_t0", None)
        target = self._find_make_way_target(
            st, now, min_clear_m=self._mw_required_clear_m(st))
        if target is not None and target != r["cell"]:
            bodies: Set[tuple] = set()
            for rid in self.registry.alive():
                xy = self._peer_xy(rid, now)
                if xy is not None:
                    bodies |= self._disc_cells(xy[0], xy[1], MW_PEER_BODY_M)
            old_target = self.goal_cell
            self.goal_cell = target
            if self.replan(now=now, extra_blocked=(
                    bodies | self._tight_cells()
                    | self._recovery_no_go())):
                r.update(cell=target, phase="DRIVE", t0=now, hold_t0=None)
                tx, ty = self.grid.cell_to_world(target)
                self.log(f"make-way: {r['beneficiary']} still blocked by my "
                         f"spot - re-targeting to {target} ({tx:.2f},{ty:.2f})")
                base = Permit(action="GO", speed_scale=RETREAT_SPEED_SCALE,
                              reason=(f"making way for {r['blockers']}: "
                                      f"re-targeting farther"))
                self._mark_yielding()
                return safety.most_restrictive(base, self._envelope(now))
            self.goal_cell = old_target
        self.log(f"make-way: no spot clear of {r['beneficiary']}'s "
                 f"stationary rule reachable - ending the hold")
        self.makeway_fail_count += 1
        r["mw_blocked_done"] = True
        return None

    def _makeway_cleared(self, r: dict, now: float) -> bool:
        """Release: beneficiary >= 1.2 m away AND its intent >= 0.95 m clear
        of me; or it has sat still MW_BENEF_STATIONARY_S while I held
        MW_HELD_MIN_FOR_STALE_S (stale beneficiary); or _mw_blocked_retarget
        ended the hold (no spot clears the beneficiary's stationary rule);
        the MW_HOLD_MAX_S cap is applied by the caller."""
        if r.get("mw_blocked_done"):
            return True
        st = self.registry.alive().get(r["beneficiary"])
        if st is None:
            return True
        me = self.state.pose
        xy = self._peer_xy(st.robot_id, now) or (st.pose.x, st.pose.y)
        far = math.hypot(xy[0] - me.x, xy[1] - me.y) >= RETREAT_RELEASE_DIST_M
        if far:
            clear = True
            for cell in st.intent.cells:
                cx, cy = self.grid.cell_to_world(cell)
                if math.hypot(cx - me.x, cy - me.y) < RETREAT_INTENT_CLEAR_M:
                    clear = False
                    break
            if clear:
                return True
        held = now - r["hold_t0"]
        if abs(st.v) < 0.05 and st.waiting_for != self.me:
            # Stale only when its stop has nothing to do with me: a
            # beneficiary still broadcasting waiting_for==me is stopped BY
            # me, and resuming under it re-blocks the aisle (measured: the
            # parked robot resumed after 5.3 s mid-pass and re-parked
            # mid-aisle, and the carrier never got through).
            if r.get("b_still") is None:
                r["b_still"] = now
            elif (now - r["b_still"] >= MW_BENEF_STATIONARY_S
                    and held >= MW_HELD_MIN_FOR_STALE_S):
                return True          # it is not coming through: resume
        else:
            r["b_still"] = None
        return False

    def _peer_goal_xy(self, st) -> Optional[tuple]:
        """A peer's current goal in world metres, or None when it has none
        (intent.goal defaults to (0, 0); fall back to the last intent cell)."""
        g = st.intent.goal
        if g is not None and (abs(g.x) > 1e-9 or abs(g.y) > 1e-9):
            return (g.x, g.y)
        if st.intent.cells:
            return self.grid.cell_to_world(st.intent.cells[-1])
        return None

    def _retreat_cleared(self, now: float) -> bool:
        """Every blocker is away from me and its path no longer passes me."""
        me = self.state.pose
        for rid in self._retreat["blockers"]:
            p = self.registry.alive().get(rid)
            if p is None:
                continue                      # gone/dead: nothing to wait for
            xy = self._peer_xy(rid, now)
            if xy and math.hypot(xy[0] - me.x, xy[1] - me.y) < RETREAT_RELEASE_DIST_M:
                return False
            for cell in p.intent.cells:
                cx, cy = self.grid.cell_to_world(cell)
                if math.hypot(cx - me.x, cy - me.y) < RETREAT_INTENT_CLEAR_M:
                    return False
            # The broadcast intent is a short horizon (~1 m): a blocker whose
            # GOAL is beside me has not passed yet, however far its intent
            # reaches (webots_userrestart7: released after 2 s, re-knotted).
            g = self._peer_goal_xy(p)
            if g and math.hypot(g[0] - me.x, g[1] - me.y) < RETREAT_INTENT_CLEAR_M:
                return False
        return True

    def _retreat_permit(self, now: float) -> Optional[Permit]:
        """Drive to the refuge, hold until the blockers pass, then resume.

        Returns None once the retreat is over (normal coordination resumes).
        Status is YIELDING with no waiting_for, so no peer's wait-for graph
        gains an edge to me while I am clearing the way.
        """
        r = self._retreat
        self._mark_reverse(r)
        if r["phase"] == "DRIVE":
            rx, ry = self.grid.cell_to_world(r["cell"])
            reached = math.hypot(self.state.pose.x - rx,
                                 self.state.pose.y - ry) <= 0.25
            if reached or not self.path or now - r["t0"] > RETREAT_MAX_DRIVE_S:
                r["phase"], r["hold_t0"] = "HOLD", now
                self.log(f"retreat: holding at refuge {r['cell']} "
                         f"({'reached' if reached else 'drive ended'})")
            else:
                base = Permit(action="GO", speed_scale=RETREAT_SPEED_SCALE,
                              reason=f"yielding to {r['blockers']}: "
                                     f"driving to refuge")
                envelope = self._envelope(now)
                self._mark_yielding()
                return safety.most_restrictive(base, envelope)

        held = now - r["hold_t0"]
        if r.get("make_way"):
            retarget = self._mw_blocked_retarget(r, now)
            if retarget is not None:
                return retarget
            min_hold, max_hold = MW_HOLD_MIN_S, MW_HOLD_MAX_S
            done = self._makeway_cleared(r, now)
        else:
            min_hold, max_hold = RETREAT_HOLD_MIN_S, RETREAT_HOLD_MAX_S
            done = self._retreat_cleared(now)
        if held >= min_hold and (done or held > max_hold):
            goal = r["saved_goal"]
            if r.get("make_way"):
                self._mw_cooldown_until[r["beneficiary"]] = \
                    now + MW_COOLDOWN_S
            self._retreat = None
            self.goal_cell = goal
            if goal is not None:
                self.replan(now=now)
            else:
                self.path, self.intent = [], Intent()
            self._deadlock_cooldown_until = now + DEADLOCK_COOLDOWN_S
            self._end_wait(now)
            self.log(f"{'make-way' if r.get('make_way') else 'retreat'}: done "
                     f"after {held:.1f}s hold, resuming"
                     + ("" if goal is not None else " (no task)"))
            return None
        self._mark_yielding()
        return Permit(action="STOP", speed_scale=0.0,
                      reason=(f"making way for {r['blockers']}"
                              if r.get("make_way")
                              else f"yielding at refuge for {r['blockers']}"))

    def _mark_reverse(self, r: Optional[dict]) -> None:
        """reverse_active (audit/webapp contract name): True while a
        retreat/make-way DRIVE is a reverse along the robot's OWN lane -
        heading opposite the lane direction captured when the yield started
        (lane_dir), while positioned in that own-direction half or off-road.
        Motion through the OPPOSING half with the same heading is wrong-way
        driving and stays False, so the flag can never mask a violation;
        the trajectory audit uses it to exempt the rule-legal own-lane
        reverse (the yield shape) from its wrong-lane metric."""
        if r is None or r.get("phase") != "DRIVE":
            return
        own = r.get("lane_dir")
        if own is None:
            return
        if traffic.heading_dir(self.state.pose.theta) \
                != traffic.OPPOSITE_DIR.get(own):
            return                          # forward/lateral travel
        cell = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        band = traffic.band_at(self.grid, cell)
        if band is None or band[1] == own:
            self.reverse_active = True

    def _mark_yielding(self) -> None:
        self._wait_started = None
        self._wait_xy = None
        self.state.waiting_time = 0.0
        self.state.waiting_for = ""
        self.state.status = YIELDING

    def _end_wait(self, now: float) -> None:
        if self._wait_started is not None or self.deadlock.yield_start is not None:
            self.deadlock.mark_resolved(now)
        self._wait_started = None
        self._wait_xy = None
        self.state.waiting_time = 0.0
        self.state.waiting_for = ""
        self.state.status = MOVING if self.path else IDLE

    # ----------------------------------------------------------------- zones
    def _path_index_near_current(self) -> int:
        """The robot's progress index on its path: the nearest cell in
        [last tracked index, + PATH_INDEX_AHEAD], unless a cell elsewhere is
        PATH_INDEX_JUMP_M closer (teleport, big correction, backing up, or a
        new path: then the global nearest, earliest first)."""
        if not self.path:
            return 0
        cur = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        path, n = self.path, len(self.path)

        def d2(i):
            return (path[i][0] - cur[0]) ** 2 + (path[i][1] - cur[1]) ** 2

        best = min(range(n), key=lambda i: (d2(i), i))
        # Deterministic progress key (merged spec 7): id(path) differs
        # between processes and made fleet_sim runs non-reproducible (12 vs
        # 15 deliveries). Content key; any in-place edit of self.path that
        # keeps (len, ends, hash) resets nothing by construction.
        key = (n, path[0], path[-1], hash(tuple(path)))
        if key == self._progress_key and 0 <= self._progress_i < n:
            p = self._progress_i
            local = min(range(p, min(n, p + PATH_INDEX_AHEAD + 1)),
                        key=lambda i: (d2(i), i))
            res = self.grid.resolution
            if math.sqrt(d2(local)) * res <= (math.sqrt(d2(best)) * res
                                              + PATH_INDEX_JUMP_M):
                best = local
        self._progress_key, self._progress_i = key, best
        return best

    def _zone_path_indices(self, zone_id: str) -> List[int]:
        zone = self.grid.zones.get(zone_id)
        if zone is None:
            return []
        return [i for i, cell in enumerate(self.path) if cell in zone.cells]

    def _next_zone(self) -> Optional[str]:
        """Return the next zone still ahead of or containing the robot.

        ``Intent.zones`` is the complete route's zone list. It is not enough
        to return its first element forever: once the robot has physically
        passed that zone, it must be released and the next zone considered.
        """
        if not self.path:
            return None
        cur_i = self._path_index_near_current()
        candidates = []
        for zid in self.intent.zones:
            if traffic.is_ranked(self.grid, zid):
                continue            # gated by _traffic_permit, in rank order
            indices = self._zone_path_indices(zid)
            if not indices:
                continue
            future = [i for i in indices if i >= cur_i]
            if future:
                candidates.append((future[0], zid))
        return min(candidates)[1] if candidates else None

    def _distance_to_zone_m(self, zone_id: str) -> float:
        """Distance from the current robot position along the active path to a zone."""
        zone = self.grid.zones.get(zone_id)
        if zone is None or not self.path:
            return 1e9

        cur_i = self._path_index_near_current()
        cur_cell = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        travelled = self.grid.cell_distance_m(cur_cell, self.path[cur_i])
        if self.path[cur_i] in zone.cells:
            return 0.0

        for i in range(cur_i + 1, len(self.path)):
            travelled += self.grid.cell_distance_m(self.path[i - 1], self.path[i])
            if self.path[i] in zone.cells:
                return travelled
        return 1e9

    def _zone_window(self, zone_id: str, now: float):
        d = self._distance_to_zone_m(zone_id)
        if d > 1e8:
            return now, now + 5.0
        t_in = now + d / max(0.05, self.v_nominal)
        zone = self.grid.zones.get(zone_id)
        n = len(zone.cells) if zone else 2
        return t_in, t_in + (n * self.grid.resolution) / max(0.05, self.v_nominal)

    def _zone_blocker(self, zone_id: str, now: float):
        """Which peer holds/wants this zone, and when will it be free?"""
        best_id, best_t = "", None
        for rid, p in self.registry.alive().items():
            if zone_id not in p.intent.zones:
                continue
            t_exit = None
            zone = self.grid.zones.get(zone_id)
            if zone:
                for cell, _ti, to in p.intent.triples():
                    if cell in zone.cells:
                        t_exit = to if t_exit is None else max(t_exit, to)
            if t_exit is None:
                t_exit = now + 3.0
            if best_t is None or t_exit > best_t:
                best_t, best_id = t_exit, rid
        return best_id, best_t

    def _maybe_release_passed_zones(self) -> None:
        """Release zones whose final path cell is behind the robot.

        A held zone must not remain locked after traversal. Checking only
        ``zid not in intent.zones`` is wrong because the intent intentionally
        contains the zone for the whole route.
        """
        if not self.path:
            # A parked robot (route done or dropped) must not keep corridor
            # segments locked: it is going nowhere, so every ranked hold its
            # hitbox has cleared is handed back here. Measured AC12
            # end-block: a dock=False robot parked at the spine mouth still
            # HELD SPINE_S..J_AW (its stale intent kept them 'needed'), and
            # every approacher's make-way was skipped as a 'zone wait'.
            for zid in list(self.arbiter.held_zones()):
                if traffic.is_ranked(self.grid, zid) \
                        and not self._occupies(zid, RELEASE_SIGMA_SCALE):
                    self.arbiter.release(zid)
            return
        cur_i = self._path_index_near_current()
        for zid in self.arbiter.held_zones():
            indices = self._zone_path_indices(zid)
            if indices and cur_i > max(indices):
                if traffic.is_ranked(self.grid, zid):
                    # R10 gap-triggered release: a ranked zone is handed
                    # back the tick my HITBOX (believed pose, grown by
                    # K_SIG*sigma exactly like the occupancy rule peers
                    # apply to me) is fully out of it - never on path index
                    # or pose centre alone, and never on a timer.
                    if self._occupies(zid, RELEASE_SIGMA_SCALE):
                        continue
                self.arbiter.release(zid)

    # ------------------------------------------------------ broadcast intent
    def _update_speed_ema(self, now: float) -> None:
        if self._speed_ema_t is not None and now > self._speed_ema_t:
            a = 1.0 - math.exp(-(now - self._speed_ema_t) / SPEED_EMA_TAU_S)
            self._speed_ema += a * (abs(self.state.v) - self._speed_ema)
        self._speed_ema_t = now

    def current_intent(self, now: float) -> Intent:
        """The intent to broadcast: the not-yet-driven suffix of my path.

        Starts at the path index nearest my pose, holds at most
        INTENT_MAX_CELLS cells and INTENT_MAX_HORIZON_S of travel, and is
        re-timed from `now` with the measured-speed EMA, so a keepalive never
        re-sends windows computed at replan time. self.intent (the whole
        route, timed at replan) is left untouched.

        zones covers the whole REMAINING route, not just the suffix: zone
        negotiation starts ZONE_REQUEST_DISTANCE_M ahead, which a slow
        robot's 8 s suffix may not reach, and peers pick the granters they
        need from this list. Zones I hold or am requesting are never hidden.
        """
        if not self.path:
            return Intent(goal=self.intent.goal)
        cur_i = self._path_index_near_current()
        rest = self.path[cur_i:]
        speed = max(INTENT_MIN_SPEED_MPS, self._speed_ema)
        margin = 0.5
        out = astar.build_intent(self.grid, rest[:INTENT_MAX_CELLS], speed,
                                 now, safety_margin_s=margin,
                                 goal=self.intent.goal)
        keep = len(out.cells)
        for i in range(1, keep):
            if out.t_enter[i] + margin - now > INTENT_MAX_HORIZON_S:
                keep = i
                break
        del out.cells[keep:], out.t_enter[keep:], out.t_exit[keep:]
        # Windows before 'now' carry no information; the current cell is
        # occupied from now.
        out.t_enter = [max(now, t) for t in out.t_enter]

        remaining = set(self.grid.zones_on_path(rest))
        zones = [z for z in self.intent.zones
                 if z in remaining
                 or self.arbiter.state.get(z, FREE) != FREE]
        # Ranked zones I hold or request (e.g. held while still inside after
        # the route moved on) are never hidden; the list goes out in rank
        # order, so peers see the acquisition order.
        zones += [z for z, st in self.arbiter.state.items()
                  if st != FREE and z not in zones
                  and traffic.is_ranked(self.grid, z)]
        out.zones = traffic.rank_sorted(self.grid, zones)
        out.active_zone_index = 0 if out.zones else -1
        return out

    # --------------------------------------------------------------- metrics
    def metrics(self) -> dict:
        m = dict(replans=self.replan_count,
                 conflicts_active=len(self.last_conflicts),
                 make_way=self.makeway_count,
                 make_way_failed=self.makeway_fail_count,
                 overtakes=self.overtake_count,
                 traffic_mode=self._eff_mode)
        m.update({f"peer_{k}": v for k, v in self.registry.metrics().items()
                  if not isinstance(v, dict)})
        m.update({f"zone_{k}": v for k, v in self.arbiter.stats.items()})
        m.update(self.deadlock.metrics())
        return m
