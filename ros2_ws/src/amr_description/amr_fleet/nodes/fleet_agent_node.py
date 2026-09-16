#!/usr/bin/env python3
"""
fleet_agent_node - ONE process per robot, hosting every coordination module.

WHY ONE PROCESS AND NOT TWELVE
Each rclpy process costs 35-50 MB of interpreter plus 8-15 MB for its DDS
DomainParticipant. Twelve nodes x three robots = 36 processes is 1.6-2.3 GB
before Gazebo starts. Since Foxy, the RMW layer creates ONE DomainParticipant
per PROCESS (context), not per node, so composing the modules into one process
cuts both memory and O(N^2) discovery traffic dramatically.

The twelve logical modules still exist - they are the classes in
amr_fleet/core/. Node granularity is a DEPLOYMENT decision, not an
architectural one.

TOPIC CONVENTION (see docs/TOPICS.md)
  RELATIVE names  -> resolve under the node namespace -> /robot_1/odom
  ABSOLUTE /fleet/* -> shared bus, identical for every robot

  A leading '/' escapes the namespace. That one character is the whole design:
  if fleet topics were namespaced, robots could not hear each other.

WHAT THIS NODE DOES NOT DO
It never publishes cmd_vel by default. Motion control belongs to
amr_navigation / Nav2. We publish a MotionPermit decision. An OPTIONAL
cmd_vel gate can be enabled if the navigation team wants us to enforce it.
"""
import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from amr_description.msg import (Conflict as RosConflict, Intent as RosIntent,
                                 MapUpdate as RosMapUpdate,
                                 MotionPermit as RosPermit,
                                 RobotState as RosRobotState, Task as RosTask,
                                 TaskAward as RosTaskAward,
                                 TaskComplete as RosTaskComplete,
                                 TaskBid as RosTaskBid,
                                 ZoneGrant as RosZoneGrant,
                                 ZoneRequest as RosZoneRequest)
from amr_description.srv import Replan

from amr_fleet.core import astar
from amr_fleet.core.coordinator import FleetCoordinator
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import FAULT, IDLE, Pose2D
from amr_fleet.core.tasks import TaskAuction
from amr_fleet.nodes import conversions as cv
from amr_fleet.nodes.qos_profiles import (COORD_QOS, DIAGNOSTIC_QOS, MAP_QOS,
                                          SENSOR_QOS, STATE_QOS)

# ---------------------------------------------------------------- fleet bus
T_STATE = "/fleet/robot_state"
T_INTENT = "/fleet/intent"
T_ZONE_REQ = "/fleet/zone_request"
T_ZONE_GRANT = "/fleet/zone_grant"
T_MAP_UPDATE = "/fleet/map_update"
T_TASK_ANNOUNCE = "/fleet/task_announce"
T_TASK_BID = "/fleet/task_bid"
T_TASK_AWARD = "/fleet/task_award"
T_TASK_COMPLETE = "/fleet/task_complete"


class FleetAgentNode(Node):

    def __init__(self):
        super().__init__("fleet_agent")

        # ------------------------------------------------------- parameters
        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("grid_yaml", "")
        self.declare_parameter("coordination_mode", "proposed")  # | baseline
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("intent_keepalive_hz", 1.0)
        self.declare_parameter("v_nominal", 0.4)
        self.declare_parameter("pose_source", "odom")        # odom only
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("enable_cmd_vel_gate", False)
        self.declare_parameter("enable_obstacle_reporting", True)
        self.declare_parameter("obstacle_persist_scans", 5)
        self.declare_parameter("obstacle_expiry_s", 20.0)
        self.declare_parameter("battery_initial_pct", 100.0)
        self.declare_parameter("battery_per_metre_pct", 0.35)
        self.declare_parameter("goal_cell", [-1, -1])
        self.declare_parameter("executor_threads", 3)

        self.robot_id = self.get_parameter("robot_id").value
        self.mode = self.get_parameter("coordination_mode").value
        self.v_nominal = float(self.get_parameter("v_nominal").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.gate_enabled = bool(self.get_parameter("enable_cmd_vel_gate").value)

        # ------------------------------------------------------------ grid
        grid_path = self.get_parameter("grid_yaml").value
        if not grid_path or not os.path.exists(grid_path):
            self.get_logger().fatal(
                f"grid_yaml parameter is empty or missing: '{grid_path}'. "
                "Pass the path to warehouse_grid.yaml. Refusing to start - "
                "coordinating without a map would be unsafe.")
            raise SystemExit(2)
        try:
            self.grid = GridMap.from_yaml(grid_path)
        except Exception as exc:
            self.get_logger().fatal(f"failed to load grid: {exc}")
            raise SystemExit(2)
        self.get_logger().info(
            f"grid {self.grid.width}x{self.grid.height} "
            f"res={self.grid.resolution}m zones={list(self.grid.zones)}")

        # --------------------------------------------------- callback groups
        # sensors: reentrant so a slow scan never delays odometry
        self.g_sensor = ReentrantCallbackGroup()
        # All callbacks are serialized by SingleThreadedExecutor. Keeping
        # fleet callbacks in a named group documents the separation without
        # pretending that a MultiThreadedExecutor makes core/ thread-safe.
        self.g_fleet = MutuallyExclusiveCallbackGroup()
        # tick: MUTUALLY EXCLUSIVE. It is the single writer of coordinator
        # state, which is why no locks are needed anywhere in core/.
        self.g_tick = MutuallyExclusiveCallbackGroup()

        # --------------------------------------------------------- publishers
        self.pub_state = self.create_publisher(RosRobotState, T_STATE, STATE_QOS)
        self.pub_intent = self.create_publisher(RosIntent, T_INTENT, COORD_QOS)
        self.pub_zreq = self.create_publisher(RosZoneRequest, T_ZONE_REQ, COORD_QOS)
        self.pub_zgrant = self.create_publisher(RosZoneGrant, T_ZONE_GRANT, COORD_QOS)
        self.pub_map = self.create_publisher(RosMapUpdate, T_MAP_UPDATE, MAP_QOS)
        self.pub_tann = self.create_publisher(RosTask, T_TASK_ANNOUNCE, COORD_QOS)
        self.pub_tbid = self.create_publisher(RosTaskBid, T_TASK_BID, COORD_QOS)
        self.pub_tawd = self.create_publisher(RosTaskAward, T_TASK_AWARD, COORD_QOS)
        self.pub_tcomplete = self.create_publisher(RosTaskComplete, T_TASK_COMPLETE, COORD_QOS)

        # per-robot (relative names -> namespaced)
        self.pub_permit = self.create_publisher(RosPermit, "motion_permit", DIAGNOSTIC_QOS)
        self.pub_conflict = self.create_publisher(RosConflict, "conflicts", DIAGNOSTIC_QOS)
        self.pub_path = self.create_publisher(Path, "coordination_path", DIAGNOSTIC_QOS)
        self.pub_diag = self.create_publisher(String, "fleet_diagnostics", DIAGNOSTIC_QOS)

        # --------------------------------------------------------- subscribers
        self.create_subscription(Odometry, "odom", self.cb_odom,
                                 SENSOR_QOS, callback_group=self.g_sensor)
        self.create_subscription(LaserScan, "scan", self.cb_scan,
                                 SENSOR_QOS, callback_group=self.g_sensor)

        self.create_subscription(RosRobotState, T_STATE, self.cb_peer_state,
                                 STATE_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosIntent, T_INTENT, self.cb_peer_intent,
                                 COORD_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosZoneRequest, T_ZONE_REQ, self.cb_zone_request,
                                 COORD_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosZoneGrant, T_ZONE_GRANT, self.cb_zone_grant,
                                 COORD_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosMapUpdate, T_MAP_UPDATE, self.cb_map_update,
                                 MAP_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosTask, T_TASK_ANNOUNCE, self.cb_task_announce,
                                 COORD_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosTaskBid, T_TASK_BID, self.cb_task_bid,
                                 COORD_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosTaskAward, T_TASK_AWARD, self.cb_task_award,
                                 COORD_QOS, callback_group=self.g_fleet)
        self.create_subscription(RosTaskComplete, T_TASK_COMPLETE, self.cb_task_complete,
                                 COORD_QOS, callback_group=self.g_fleet)

        # OPTIONAL cmd_vel gate. Disabled by default so this package never
        # fights amr_navigation for the actuator. When enabled, remap
        # cmd_vel_raw <- the navigation output, and our cmd_vel becomes the
        # gated result.
        if self.gate_enabled:
            self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
            self.create_subscription(Twist, "cmd_vel_raw", self.cb_cmd_raw,
                                     10, callback_group=self.g_sensor)
            self.get_logger().warn(
                "cmd_vel gate ENABLED: this node now publishes cmd_vel. "
                "Ensure amr_navigation publishes to cmd_vel_raw instead.")

        # ------------------------------------------------------------ service
        self.create_service(Replan, "replan", self.srv_replan,
                            callback_group=self.g_tick)

        # ------------------------------------------------------- coordinator
        self.coord = FleetCoordinator(
            robot_id=self.robot_id, gridmap=self.grid,
            send_zone_request=self._send_zone_request,
            send_zone_grant=self._send_zone_grant,
            v_nominal=self.v_nominal, mode=self.mode,
            logger=lambda m: self.get_logger().info(m))

        self.auction = TaskAuction(
            robot_id=self.robot_id,
            announce=self._send_task_announce,
            bid=self._send_task_bid,
            award=self._send_task_award,
            path_length_m=self.coord.path_length_m_between,
            logger=lambda m: self.get_logger().info(m))

        # ---------------------------------------------------- runtime state
        self.state_seq = 0
        self.intent_seq = 0
        self._odom: Optional[Odometry] = None
        self._scan: Optional[LaserScan] = None
        self._last_scan_t = 0.0
        self._last_odom_t = 0.0
        self._last_intent_pub = 0.0
        self._last_intent_hash = None
        self._hits = {}
        self._reported = set()
        self._distance_m = 0.0
        self._task_phase = None
        self._last_xy = None
        self.coord.state.battery_pct = float(
            self.get_parameter("battery_initial_pct").value)

        gc = list(self.get_parameter("goal_cell").value)
        if len(gc) == 2 and gc[0] >= 0:
            self.coord.set_goal((int(gc[0]), int(gc[1])))

        # ------------------------------------------------------------- timers
        tick_hz = float(self.get_parameter("tick_rate_hz").value)
        state_hz = float(self.get_parameter("state_rate_hz").value)
        self.create_timer(1.0 / tick_hz, self.tick, callback_group=self.g_tick)
        self.create_timer(1.0 / state_hz, self.publish_state,
                          callback_group=self.g_tick)
        self.create_timer(1.0, self.publish_diagnostics, callback_group=self.g_tick)

        self.get_logger().info(
            f"fleet_agent up: robot_id={self.robot_id} mode={self.mode} "
            f"tick={tick_hz}Hz ns={self.get_namespace()} "
            f"domain={os.environ.get('ROS_DOMAIN_ID','0')} "
            f"rmw={rclpy.get_rmw_implementation_identifier()}")

    # ================================================== sensor callbacks ====
    # Callbacks STORE ONLY. All computation happens in the single tick, which
    # gives deterministic timing and removes every data race.

    def cb_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._last_odom_t = self._now()
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        if self._last_xy is not None:
            self._distance_m += math.hypot(x - self._last_xy[0],
                                           y - self._last_xy[1])
        self._last_xy = (x, y)

        self.coord.state.pose = Pose2D(x, y, theta)
        self.coord.state.v = msg.twist.twist.linear.x
        self.coord.state.w = msg.twist.twist.angular.z

    def cb_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._last_scan_t = self._now()

    def cb_cmd_raw(self, msg: Twist) -> None:
        """Optional gate: scale the navigation output by the permit."""
        scale = self._last_scale if hasattr(self, "_last_scale") else 1.0
        out = Twist()
        out.linear.x = msg.linear.x * scale
        out.angular.z = msg.angular.z * (1.0 if scale > 0.0 else 0.0)
        self.pub_cmd.publish(out)

    # ==================================================== fleet callbacks ===
    def cb_peer_state(self, msg: RosRobotState) -> None:
        if msg.robot_id == self.robot_id:
            return          # DDS delivers our own publications back to us
        self.coord.on_peer_state(cv.state_from_ros(msg), now=self._now())

    def cb_peer_intent(self, msg: RosIntent) -> None:
        if msg.robot_id == self.robot_id:
            return
        peer = self.coord.registry.peers.get(msg.robot_id)
        if peer is not None:
            peer.intent = cv.intent_from_ros(msg)

    def cb_zone_request(self, msg: RosZoneRequest) -> None:
        if msg.robot_id == self.robot_id:
            return
        self.coord.on_zone_request(dict(
            robot_id=msg.robot_id, zone_id=msg.zone_id,
            lamport_ts=int(msg.lamport_ts),
            priority_score=float(msg.priority_score),
            t_enter_est=float(msg.t_enter_est),
            t_exit_est=float(msg.t_exit_est), req_id=int(msg.req_id)))

    def cb_zone_grant(self, msg: RosZoneGrant) -> None:
        if msg.granter_id == self.robot_id:
            return
        self.coord.on_zone_grant(dict(
            granter_id=msg.granter_id, requester_id=msg.requester_id,
            zone_id=msg.zone_id, req_id=int(msg.req_id),
            granted=bool(msg.granted), lamport_ts=int(msg.lamport_ts)))

    def cb_map_update(self, msg: RosMapUpdate) -> None:
        if msg.reporter_id == self.robot_id:
            return
        reporter, blocked, cleared, conf, expiry = cv.map_update_from_ros(msg)
        self.coord.on_map_update(reporter, blocked, cleared, conf, expiry, now=self._now())

    def cb_task_announce(self, msg: RosTask) -> None:
        self._sync_auction_inputs()
        self.auction.on_announce(cv.task_from_ros(msg), now=self._now())

    def cb_task_bid(self, msg: RosTaskBid) -> None:
        if msg.robot_id == self.robot_id:
            return
        self.auction.on_bid(dict(robot_id=msg.robot_id, task_id=msg.task_id,
                                 bid=float(msg.bid)))

    def cb_task_award(self, msg: RosTaskAward) -> None:
        self.auction.on_award(dict(task_id=msg.task_id,
                                   winner_id=msg.winner_id))
        if msg.winner_id == self.robot_id and self.auction.my_task is not None:
            self.coord.current_task = self.auction.my_task
            self.coord.state.task_id = self.auction.my_task.task_id
            self.coord.state.task_priority = self.auction.my_task.priority
            self._task_phase = "PICKUP"
            self.coord.set_goal(self.auction.my_task.pickup, now=self._now())
        elif (msg.winner_id != self.robot_id
              and self.coord.current_task is not None
              and self.coord.current_task.task_id == msg.task_id
              and self.auction.my_task is None):
            # Lost a claim collision. Clear the navigation target immediately.
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self._task_phase = None

    def cb_task_complete(self, msg: RosTaskComplete) -> None:
        self.auction.on_complete(msg.task_id)
        if (self.coord.current_task is not None
                and self.coord.current_task.task_id == msg.task_id):
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._task_phase = None

    # ============================================================== senders ==
    def _send_zone_request(self, d: dict) -> None:
        m = RosZoneRequest()
        m.header.stamp = self.get_clock().now().to_msg()
        m.robot_id = d["robot_id"]; m.zone_id = d["zone_id"]
        m.lamport_ts = int(d["lamport_ts"])
        m.priority_score = float(d["priority_score"])
        m.t_enter_est = float(d["t_enter_est"])
        m.t_exit_est = float(d["t_exit_est"])
        m.req_id = int(d["req_id"])
        self.pub_zreq.publish(m)

    def _send_zone_grant(self, d: dict) -> None:
        m = RosZoneGrant()
        m.header.stamp = self.get_clock().now().to_msg()
        m.granter_id = d["granter_id"]; m.requester_id = d["requester_id"]
        m.zone_id = d["zone_id"]; m.req_id = int(d["req_id"])
        m.granted = bool(d["granted"]); m.lamport_ts = int(d["lamport_ts"])
        self.pub_zgrant.publish(m)

    def _send_task_announce(self, d: dict) -> None:
        self.pub_tann.publish(cv.task_to_ros(d, self.get_clock(), self.robot_id))

    def _send_task_bid(self, d: dict) -> None:
        m = RosTaskBid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.robot_id = d["robot_id"]; m.task_id = d["task_id"]
        m.bid = float(d["bid"])
        m.est_completion_s = float(d.get("est_completion_s", 0.0))
        m.battery_after_pct = float(d.get("battery_after_pct", 0.0))
        m.seq = int(d.get("seq", 0))
        self.pub_tbid.publish(m)

    def _send_task_award(self, d: dict) -> None:
        m = RosTaskAward()
        m.header.stamp = self.get_clock().now().to_msg()
        m.task_id = d["task_id"]; m.winner_id = d["winner_id"]
        m.winning_bid = float(d.get("winning_bid", 0.0))
        m.num_bidders = int(d.get("num_bidders", 0))
        m.stamp = float(d.get("stamp", 0.0))
        self.pub_tawd.publish(m)

    def _send_task_complete(self, task_id: str, now: float) -> None:
        m = RosTaskComplete()
        m.header.stamp = self.get_clock().now().to_msg()
        m.task_id = task_id
        m.robot_id = self.robot_id
        m.stamp = now
        self.pub_tcomplete.publish(m)

    # ================================================================= tick ==
    def tick(self) -> None:
        now = self._now()

        # ---- health gates. Fail safe: when uncertain, STOP. A stopped robot
        # is a static obstacle peers can route around; a confused moving robot
        # is a hazard.
        if self._odom is None:
            return
        if (now - self._last_odom_t) > 1.0:
            self._fault("odom timeout")
            return
        if self._scan is not None and (now - self._last_scan_t) > 1.0:
            self._fault("scan timeout")
            return
        if self.coord.state.status == FAULT and (now - self._last_odom_t) < 0.5:
            self.coord.state.status = IDLE       # recovered

        self._update_battery()
        self._advance_task_lifecycle(now)

        if self.get_parameter("enable_obstacle_reporting").value and self._scan:
            self._detect_obstacles(now)

        try:
            permit = self.coord.tick(now)
        except Exception as exc:
            self.get_logger().error(f"coordination tick failed: {exc}")
            self._fault(f"tick exception: {exc}")
            return

        self._last_scale = permit.speed_scale
        self.pub_permit.publish(cv.permit_to_ros(self.robot_id, permit,
                                                 self.get_clock()))
        for c in self.coord.last_conflicts[:10]:
            self.pub_conflict.publish(
                cv.conflict_to_ros(self.robot_id, c, self.get_clock()))

        self._sync_auction_inputs()
        self.auction.tick(now, self.coord.registry.alive_ids(),
                          self.coord.registry.last_seen)

        self._maybe_publish_intent(now)
        self._publish_path()

    def _fault(self, reason: str) -> None:
        if self.coord.state.status != FAULT:
            self.get_logger().error(f"FAULT: {reason}")
        self.coord.state.status = FAULT
        self.coord.state.alive = False
        self.coord.arbiter.release_all()     # never hold a zone while faulted
        self.auction.release_current()
        p = RosPermit()
        p.header.stamp = self.get_clock().now().to_msg()
        p.robot_id = self.robot_id
        p.action = 2                          # STOP
        p.speed_scale = 0.0
        p.reason = f"FAULT: {reason}"
        self.pub_permit.publish(p)
        self._last_scale = 0.0

    # ============================================================ publishing =
    def publish_state(self) -> None:
        self.state_seq += 1
        self.coord.state.seq = self.state_seq
        self.coord.state.stamp = self._now()
        self.pub_state.publish(
            cv.state_to_ros(self.coord.state, self.get_clock()))

    def _maybe_publish_intent(self, now: float) -> None:
        """Publish on change, plus a 1 Hz keepalive.

        Publishing a 40-cell intent at 10 Hz would be wasteful; publishing only
        on change would leave a late joiner blind. TRANSIENT_LOCAL plus a
        keepalive covers both.
        """
        h = (tuple(self.coord.intent.cells), tuple(self.coord.intent.zones))
        keepalive_period = 1.0 / float(
            self.get_parameter("intent_keepalive_hz").value)
        if h != self._last_intent_hash or (now - self._last_intent_pub) > keepalive_period:
            self.intent_seq += 1
            self.pub_intent.publish(cv.intent_to_ros(
                self.robot_id, self.intent_seq, self.coord.intent,
                self.get_clock(), self.coord.state.task_id,
                self.coord.state.task_priority, self.coord.state.priority_score))
            self._last_intent_hash = h
            self._last_intent_pub = now

    def _publish_path(self) -> None:
        if not self.coord.path:
            return
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.map_frame
        from geometry_msgs.msg import PoseStamped
        for cell in self.coord.path:
            x, y = self.grid.cell_to_world(cell)
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_path.publish(p)

    def publish_diagnostics(self) -> None:
        import json
        m = self.coord.metrics()
        m.update(self.auction.stats)
        m["distance_m"] = round(self._distance_m, 2)
        m["battery_pct"] = round(self.coord.state.battery_pct, 1)
        m["status"] = self.coord.state.status
        s = String(); s.data = json.dumps(m)
        self.pub_diag.publish(s)

    # =========================================================== obstacles ===
    def _detect_obstacles(self, now: float) -> None:
        """Persistent-hit filter: N consecutive scans before declaring a cell
        blocked. Stops a passing worker from permanently poisoning the map
        while still catching a dropped pallet within half a second."""
        scan = self._scan
        need = int(self.get_parameter("obstacle_persist_scans").value)
        expiry_s = float(self.get_parameter("obstacle_expiry_s").value)
        pose = self.coord.state.pose
        observed = set()

        step = max(1, len(scan.ranges) // 90)     # subsample: Nano-friendly
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]
            if not (scan.range_min < r < min(scan.range_max, 6.0)):
                continue
            a = scan.angle_min + i * scan.angle_increment + pose.theta
            cell = self.grid.world_to_cell(pose.x + r * math.cos(a),
                                           pose.y + r * math.sin(a))
            if not self.grid.is_static_free(cell):
                continue                          # expected rack/wall return
            observed.add(cell)
            self._hits[cell] = self._hits.get(cell, 0) + 1

        newly = []
        for cell, n in list(self._hits.items()):
            if cell not in observed:
                self._hits[cell] -= 1
                if self._hits[cell] <= 0:
                    del self._hits[cell]
                    self._reported.discard(cell)
            elif n >= need and cell not in self._reported:
                self._reported.add(cell)
                newly.append(cell)

        if newly:
            conf = min(1.0, need * 0.2)
            expiry = now + expiry_s
            self.grid.report_blocked(newly, self.robot_id, conf, expiry)
            self.pub_map.publish(cv.map_update_to_ros(
                self.robot_id, newly, [], conf, expiry, self.get_clock()))
            self.get_logger().info(f"blockage detected {newly} -> broadcast")
            if self.coord._path_invalidated():
                self.coord.replan(now=now)

    def _advance_task_lifecycle(self, now: float) -> None:
        """Drive the auction winner through pickup -> dropoff -> complete."""
        task = self.auction.my_task
        if task is None:
            self._task_phase = None
            return

        target = task.pickup if self._task_phase in (None, "PICKUP") else task.dropoff
        pose = self.coord.state.pose
        tx, ty = self.grid.cell_to_world(target)
        dist = math.hypot(pose.x - tx, pose.y - ty)
        threshold = max(0.15, 0.5 * self.grid.resolution)

        if self._task_phase in (None, "PICKUP") and dist <= threshold:
            self._task_phase = "DROPOFF"
            self.coord.set_goal(task.dropoff, now=now)
            self.get_logger().info(f"task {task.task_id}: pickup reached, heading to dropoff")
            return

        if self._task_phase == "DROPOFF" and dist <= threshold:
            tid = task.task_id
            self._send_task_complete(tid, now)
            self.auction.complete_current(now)
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._task_phase = None
            self.get_logger().info(f"task {tid}: completed")

    # ============================================================== helpers ==
    def _update_battery(self) -> None:
        rate = float(self.get_parameter("battery_per_metre_pct").value)
        if self._last_xy is not None:
            self.coord.state.battery_pct = max(
                0.0, float(self.get_parameter("battery_initial_pct").value)
                - rate * self._distance_m)

    def _sync_auction_inputs(self) -> None:
        self.auction.current_cell = self.grid.world_to_cell(
            self.coord.state.pose.x, self.coord.state.pose.y)
        self.auction.battery_pct = self.coord.state.battery_pct
        self.auction.nominal_speed = self.v_nominal
        self.auction.state_status = self.coord.state.status

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================== service ==
    def srv_replan(self, req, resp):
        avoid = set()
        n = min(len(req.avoid_rows), len(req.avoid_cols))
        for i in range(n):
            avoid.add((int(req.avoid_rows[i]), int(req.avoid_cols[i])))
        now = self._now()
        if not bool(req.force) and not avoid:
            resp.success = bool(self.coord.path)
            resp.path_length = len(self.coord.path)
            resp.est_duration_s = (self.coord.distance_to_goal_m()
                                   / max(0.05, self.v_nominal))
            resp.message = "current path retained" if resp.success else "no current path"
            return resp

        # Apply requested cells as temporary soft costs when avoid_penalty > 0,
        # otherwise treat them as hard obstacles for this replan. The map
        # itself is not permanently modified by the service request.
        penalty = float(req.avoid_penalty)
        if avoid and penalty <= 0.0:
            for c in avoid:
                self.grid.report_blocked([c], self.robot_id, 1.0, now + 30.0)
            ok = self.coord.replan(now=now)
        elif avoid:
            ok = self.coord.replan(avoid_zones=set(), zone_penalty=penalty, now=now)
            # The service has cell-level semantics, so perform an explicit A*
            # replan with those cells as soft costs when a positive penalty was
            # requested.
            start = self.grid.world_to_cell(self.coord.state.pose.x, self.coord.state.pose.y)
            blocked = self.grid.blocked_cells(now)
            extra = astar.congestion_costs(
                [p.intent for p in self.coord.registry.alive().values()], now)
            for c in avoid:
                extra[c] = extra.get(c, 0.0) + penalty
            path = astar.astar(self.grid, start, self.coord.goal_cell,
                               blocked=blocked, extra_cost=extra) if self.coord.goal_cell is not None else []
            if path:
                self.coord.path = path
                self.coord.intent = astar.build_intent(self.grid, path, self.v_nominal, now)
                self.coord.replan_count += 1
                ok = True
            else:
                ok = False
        else:
            ok = self.coord.replan(now=now)

        resp.success = ok
        resp.path_length = len(self.coord.path)
        resp.est_duration_s = (self.coord.distance_to_goal_m()
                               / max(0.05, self.v_nominal))
        resp.message = "ok" if ok else "no path found"
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FleetAgentNode()
    except SystemExit:
        rclpy.shutdown()
        return
    # Core state is intentionally single-writer. A single-threaded executor
    # guarantees fleet callbacks cannot mutate the registry while tick() is
    # reading it. The coordination algorithms remain deterministic.
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.coord.arbiter.release_all()   # clean shutdown: free zones
            node._fault("shutting down")
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
