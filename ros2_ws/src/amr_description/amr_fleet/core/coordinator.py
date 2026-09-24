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
from typing import Dict, List, Optional, Set

from . import astar
from .conflict import ConflictDetector
from .deadlock import DeadlockManager
from .gridmap import GridMap
from .models import (Conflict, Intent, Permit, Pose2D, RobotState, Task,
                     FAULT, IDLE, MOVING, WAITING, YIELDING)
from .peers import PeerRegistry
from .priority import priority_score
from .zone import ZoneArbiter, ZoneTimeout

# Distance ahead of a zone at which we begin negotiating. Must be far enough to
# stop comfortably: v^2/(2a) at 0.5 m/s and 0.5 m/s^2 is 0.25 m, plus reaction
# and a margin.
ZONE_REQUEST_DISTANCE_M = 2.5
ZONE_ENTRY_DISTANCE_M = 0.6
MIN_SPEED_SCALE = 0.25


class FleetCoordinator:
    def __init__(self, robot_id: str, gridmap: GridMap,
                 send_zone_request, send_zone_grant,
                 v_nominal: float = 0.4,
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

        self.state = RobotState(robot_id=robot_id)
        self.intent = Intent()
        self.path: List = []
        self.goal_cell = None
        self.current_task: Optional[Task] = None

        self.last_conflicts: List[Conflict] = []
        self.replan_count = 0
        self._wait_started: Optional[float] = None
        self._zone_penalties: Dict[str, float] = {}
        self._last_now: float = 0.0

    # ------------------------------------------------------------ inbound
    def on_peer_state(self, st: RobotState, now: float) -> None:
        self.registry.update(st, now=now)

    def on_zone_request(self, msg: dict) -> None:
        zid = msg.get("zone_id", "")
        needed = zid in self.intent.zones
        # Pass the relevant-peer set so the arbiter can apply the ownership
        # check that keeps mutual exclusion safe. See zone.py::on_request.
        self.arbiter.on_request(msg, i_need_zone=needed,
                                relevant_peers=self.registry.peers_needing_zone(zid))

    def on_zone_grant(self, msg: dict) -> None:
        self.arbiter.on_grant(msg)

    def on_map_update(self, reporter: str, blocked, cleared,
                      confidence: float, expiry: float, now: float) -> None:
        self.grid.merge_remote_update(reporter, blocked, cleared,
                                      confidence, expiry, self.state.pose)
        if self._path_invalidated(now):
            self.log("peer map update invalidated my path - replanning")
            self.replan(now=now)

    # -------------------------------------------------------------- planning
    def set_goal(self, goal_cell, now: Optional[float] = None) -> bool:
        self.goal_cell = goal_cell
        return self.replan(now=now)

    def clear_goal(self) -> None:
        self.goal_cell = None
        self.path = []
        self.intent = Intent()

    def replan(self, avoid_zones: Optional[Set[str]] = None,
               zone_penalty: float = 50.0, now: Optional[float] = None) -> bool:
        if self.goal_cell is None:
            self.path, self.intent = [], Intent()
            return False

        start = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        now = self._last_now if now is None else now

        blocked = self.grid.blocked_cells(now)
        for peer in self.registry.obstacles_from_dead():
            blocked.add(self.grid.world_to_cell(peer.pose.x, peer.pose.y))

        extra = astar.congestion_costs(
            [p.intent for p in self.registry.alive().values()], now)
        for zid, pen in self._zone_penalties.items():
            zone = self.grid.zones.get(zid)
            if zone:
                for c in zone.cells:
                    extra[c] = extra.get(c, 0.0) + pen
        for zid in (avoid_zones or set()):
            zone = self.grid.zones.get(zid)
            if zone:
                for c in zone.cells:
                    extra[c] = extra.get(c, 0.0) + zone_penalty

        path = astar.astar(self.grid, start, self.goal_cell,
                           blocked=blocked, extra_cost=extra)
        if not path:
            self.log(f"REPLAN FAILED {start} -> {self.goal_cell}")
            self.path, self.intent = [], Intent()
            return False

        self.path = path
        self.intent = astar.build_intent(self.grid, path, self.v_nominal, now)
        self.replan_count += 1
        return True

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
        p = astar.astar(self.grid, a, b, blocked=self.grid.blocked_cells(self._last_now))
        return astar.path_length_m(self.grid, p) if p else -1.0

    # ------------------------------------------------------------------ tick
    def tick(self, now: float) -> Permit:
        self._last_now = now
        self.grid.expire(now)
        newly_dead = self.registry.tick(now)
        for rid in newly_dead:
            self.arbiter.drop_peer(rid)
            self.log(f"released dependencies on failed peer {rid}")

        if self.state.status == FAULT:
            self.arbiter.release_all()
            return Permit(action="STOP", speed_scale=0.0, reason="FAULT")

        self._update_priority(now)
        peers = self.registry.alive()

        self.last_conflicts = self.detector.detect(self.state, self.intent,
                                                   peers, now)

        if self.mode == "baseline":
            return self._baseline_permit(peers, now)

        permit = self._proposed_permit(peers, now)

        dl = self.deadlock.tick(self.state, peers, now)
        if dl and dl.get("role") == "VICTIM":
            permit = self._yield_permit(dl, now)
        elif dl and dl.get("role") == "HOLD":
            permit.deadlock_detected = True
            permit.deadlock_cycle = dl.get("cycle", [])

        self._track_waiting(permit, now)
        return permit

    # ---------------------------------------------------------- priority
    def _update_priority(self, now: float) -> None:
        self.state.priority_score = priority_score(
            self.state, self.distance_to_goal_m(),
            self.arbiter.in_any_zone(), self.weights)

    def _track_waiting(self, permit: Permit, now: float) -> None:
        if permit.action in ("STOP", "YIELD"):
            if self._wait_started is None:
                self._wait_started = now
            self.state.waiting_time = now - self._wait_started
            self.state.status = (YIELDING if permit.action == "YIELD" else WAITING)
            self.state.waiting_for = permit.blocking_robot or ""
        else:
            if self._wait_started is not None:
                self.deadlock.mark_resolved(now)
            self._wait_started = None
            self.state.waiting_time = 0.0
            self.state.waiting_for = ""
            self.state.status = MOVING if self.path else IDLE

    # ------------------------------------------------------- the real system
    def _proposed_permit(self, peers, now: float) -> Permit:
        zone_id = self._next_zone()
        if zone_id is None:
            return self._speed_from_ttc(now)

        dist = self._distance_to_zone_m(zone_id)
        if dist > ZONE_REQUEST_DISTANCE_M:
            return self._speed_from_ttc(now)

        relevant = self.registry.peers_needing_zone(zone_id)
        if not relevant:
            # Nobody competes. Claim it without a negotiation round.
            self.arbiter.request(zone_id, self.state.priority_score,
                                 now, now + 5.0, now=now)

        if self.arbiter.state.get(zone_id) not in ("REQUESTING", "HELD"):
            t_in, t_out = self._zone_window(zone_id, now)
            self.arbiter.request(zone_id, self.state.priority_score, t_in, t_out, now=now)

        try:
            if self.arbiter.may_enter(zone_id, relevant, now=now):
                if dist < ZONE_ENTRY_DISTANCE_M:
                    self._maybe_release_passed_zones()
                return Permit(action="GO", speed_scale=1.0,
                              reason=f"zone {zone_id} acquired", zone_id=zone_id)
        except ZoneTimeout:
            self._zone_penalties[zone_id] = 50.0
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

    def _speed_from_ttc(self, now: float) -> Permit:
        """No zone contested, but continuous proximity may still need action."""
        worst = None
        for c in self.last_conflicts:
            if c.kind != "TTC":
                continue
            if worst is None or c.t_conflict < worst.t_conflict:
                worst = c
        if worst is None:
            return Permit(action="GO", speed_scale=1.0, reason="clear")

        ttc = max(0.05, worst.t_conflict - now)
        if ttc < 1.5 and not worst.i_have_priority:
            return Permit(action="STOP", speed_scale=0.0,
                          reason=f"TTC {ttc:.1f}s with {worst.peer_id}",
                          blocking_robot=worst.peer_id)
        if ttc < 3.5:
            return Permit(action="SLOW", speed_scale=max(MIN_SPEED_SCALE, ttc / 3.5),
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
        """I am the deadlock victim. Cheapest and safest option first."""
        cycle = dl.get("cycle", [])
        held = set()
        for rid in cycle:
            p = self.registry.peers.get(rid)
            if p:
                held.update(p.intent.zones[:1])

        if self.replan(avoid_zones=held, now=now):
            return Permit(action="REROUTE", speed_scale=1.0,
                          reason="deadlock victim: rerouting",
                          deadlock_detected=True, deadlock_cycle=cycle)

        # No alternative route. Hold and let the others move. Physical
        # reversing is a motion-control action and belongs to amr_navigation;
        # we signal YIELD and let that layer execute the manoeuvre.
        return Permit(action="YIELD", speed_scale=0.0,
                      reason="deadlock victim: yielding, no alternative route",
                      blocking_robot=cycle[1] if len(cycle) > 1 else "",
                      deadlock_detected=True, deadlock_cycle=cycle)

    # ----------------------------------------------------------------- zones
    def _path_index_near_current(self) -> int:
        """Return the path index nearest to the robot's current grid cell."""
        if not self.path:
            return 0
        cur = self.grid.world_to_cell(self.state.pose.x, self.state.pose.y)
        return min(range(len(self.path)),
                   key=lambda i: (self.path[i][0] - cur[0]) ** 2
                                 + (self.path[i][1] - cur[1]) ** 2)

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
            return
        cur_i = self._path_index_near_current()
        for zid in self.arbiter.held_zones():
            indices = self._zone_path_indices(zid)
            if indices and cur_i > max(indices):
                self.arbiter.release(zid)

    # --------------------------------------------------------------- metrics
    def metrics(self) -> dict:
        m = dict(replans=self.replan_count,
                 conflicts_active=len(self.last_conflicts))
        m.update({f"peer_{k}": v for k, v in self.registry.metrics().items()
                  if not isinstance(v, dict)})
        m.update({f"zone_{k}": v for k, v in self.arbiter.stats.items()})
        m.update(self.deadlock.metrics())
        return m
