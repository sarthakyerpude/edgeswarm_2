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
import secrets
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:   # AMCL's no-motion update service type; ships with Nav2.
    from std_srvs.srv import Empty as EmptySrv
except ImportError:  # pragma: no cover - optional runtime dependency
    EmptySrv = None

from amr_description.msg import (Conflict as RosConflict, Intent as RosIntent,
                                 MapUpdate as RosMapUpdate,
                                 MotionPermit as RosPermit,
                                 RobotState as RosRobotState, Task as RosTask,
                                 TaskAward as RosTaskAward,
                                 TaskComplete as RosTaskComplete,
                                 TaskRelease as RosTaskRelease,
                                 TaskBid as RosTaskBid,
                                 ZoneGrant as RosZoneGrant,
                                 ZoneRequest as RosZoneRequest)

try:    # Generated only after the next colcon build (R6, msg/TaskCancel.msg).
    from amr_description.msg import TaskCancel as RosTaskCancel
except ImportError:  # pragma: no cover - stale install
    RosTaskCancel = None

from amr_description.srv import Replan

from amr_fleet.core import astar
from amr_fleet.core import docking
from amr_fleet.core import priority as _core_priority
from amr_fleet.core.coordinator import FleetCoordinator
from amr_fleet.core.gridmap import GridMap
from amr_fleet.core.models import (CHARGING, FAULT, IDLE, LOC_DEGRADED,
                                   LOC_LOST, LOC_OK, YIELDING, Pose2D)
from amr_fleet.core.tasks import TASK_RELEASE_S, TaskAuction
from amr_fleet.nodes import conversions as cv
from amr_fleet.nodes.qos_profiles import (COORD_QOS, DIAGNOSTIC_QOS, MAP_QOS,
                                          PATH_QOS, PERMIT_QOS, SENSOR_QOS,
                                          STATE_QOS)

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
T_TASK_RELEASE = "/fleet/task_release"


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
        self.declare_parameter("pose_source", "tf")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_pose_startup_timeout_s", 15.0)
        self.declare_parameter("map_pose_timeout_s", 1.0)
        self.declare_parameter("enable_cmd_vel_gate", False)
        self.declare_parameter("enable_obstacle_reporting", True)
        self.declare_parameter("obstacle_persist_scans", 5)
        self.declare_parameter("obstacle_clear_scans", 3)
        self.declare_parameter("obstacle_expiry_s", 20.0)
        self.declare_parameter("scan_startup_timeout_s", 5.0)
        # Sensor / heartbeat tolerances (sim seconds). 1.0 s was too tight for
        # a CPU-loaded WSL simulation: one executor stall faulted the robot,
        # dropped its task and the fleet never recovered.
        self.declare_parameter("scan_timeout_s", 3.0)
        self.declare_parameter("odom_timeout_s", 3.0)
        self.declare_parameter("peer_suspect_s", 2.0)
        self.declare_parameter("peer_dead_s", 5.0)
        # Obstacle-report filters. Lidar hits on walls/racks that land in an
        # adjacent free cell (localization error at 0.1 m resolution) and hits
        # on peer robots' bodies must NOT become shared map blockages.
        self.declare_parameter("obstacle_static_margin_cells", 2)
        self.declare_parameter("obstacle_peer_radius_m", 0.6)
        self.declare_parameter("obstacle_max_angular_vel", 0.5)
        self.declare_parameter("obstacle_max_scan_age_s", 0.3)
        self.declare_parameter("task_route_failure_timeout_s", 5.0)
        self.declare_parameter("battery_initial_pct", 100.0)
        self.declare_parameter("battery_per_metre_pct", 0.35)
        self.declare_parameter("goal_cell", [-1, -1])
        # Home = spawn pose = charging dock (metres, map frame). NaN disables
        # return-to-dock; the launch file passes the spawn pose per robot.
        self.declare_parameter("home_x", float("nan"))
        self.declare_parameter("home_y", float("nan"))
        self.declare_parameter("home_yaw", float("nan"))   # NaN: any heading
        self.declare_parameter("return_home_enabled", True)
        self.declare_parameter("return_home_idle_s", 4.0)
        # Docking = sitting ON the spawn pose: see core/docking.py.
        self.declare_parameter("dock_approach_m", 0.35)
        self.declare_parameter("dock_pos_tol_m", 0.08)
        self.declare_parameter("dock_yaw_tol_deg", 5.0)
        self.declare_parameter("dock_align_timeout_s", 20.0)
        self.declare_parameter("dock_max_retries", 2)
        self.declare_parameter("dock_backoff_m", 0.4)
        self.declare_parameter("charge_rate_pct_per_s", 1.0)
        self.declare_parameter("battery_low_pct", 30.0)
        self.declare_parameter("battery_resume_pct", 80.0)
        # Load/unload dwell: STOP held at a reached pickup/dropoff marker.
        self.declare_parameter("arrival_dwell_s", 2.0)
        # Localization health gate on AMCL's reported covariance.
        self.declare_parameter("loc_max_sigma_xy_m", 0.6)
        self.declare_parameter("loc_max_sigma_yaw_rad", 0.4)
        self.declare_parameter("loc_lost_after_s", 2.0)
        self.declare_parameter("loc_recover_after_s", 2.0)
        self.declare_parameter("loc_nomotion_update_period_s", 1.0)
        # LOST recovery by odometry dead reckoning (see _maybe_reseed): AMCL
        # is re-seeded at the last confident pose + the odometry since then.
        self.declare_parameter("loc_reseed_enabled", True)
        self.declare_parameter("loc_reseed_period_s", 10.0)
        self.declare_parameter("loc_reseed_max_attempts", 3)
        # Never trust dead reckoning past this much odometry since the last
        # confident fix: the robot then simply stays LOST and stopped.
        self.declare_parameter("loc_reseed_max_odom_m", 5.0)
        # Re-seed ONLY with evidence: below this much odometry since the last
        # confident fix the "dead-reckoned" pose IS that fix - which, after an
        # aliased lock-in, is the diverged pose itself. Writing it back with a
        # tight covariance killed AMCL's injected recovery hypotheses and left
        # robot_3 confidently 3.58 m wrong for ~7 min (loc_validate2, t=163).
        self.declare_parameter("loc_reseed_min_odom_m", 0.3)
        # ...and never before AMCL's own recovery injection (fired by the same
        # sigma spike that declared LOST) has had this long to converge.
        self.declare_parameter("loc_reseed_hold_s", 8.0)
        # A parked robot gets no AMCL motion updates: while stationary with an
        # amcl_pose older than this, force a scan correction periodically.
        self.declare_parameter("loc_stationary_nomotion_after_s", 5.0)
        self.declare_parameter("loc_stationary_nomotion_period_s", 5.0)
        self.robot_id = self.get_parameter("robot_id").value
        self.mode = self.get_parameter("coordination_mode").value
        self.v_nominal = float(self.get_parameter("v_nominal").value)
        self.pose_source = str(self.get_parameter("pose_source").value).lower()
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        if self.pose_source not in ("tf", "odom"):
            self.get_logger().fatal("pose_source must be 'tf' or 'odom'")
            raise SystemExit(2)
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
        self._near_static = self._build_static_margin(
            int(self.get_parameter("obstacle_static_margin_cells").value))
        # Build the cached clearance field now: it is lazy, and the first
        # A* call otherwise pays ~100 ms inside a ROS callback on a Jetson.
        self.grid.clearance_cost()
        self.dock = self._make_dock_policy()

        # YIELD retreat manoeuvre state (see _maybe_retreat).
        self._retreat_cell = None
        self._retreat_until = None
        self._retreat_cooldown_until = 0.0

        self._tf_buffer = None
        self._tf_listener = None
        if self.pose_source == "tf":
            self._tf_buffer = Buffer(node=self)
            # Jazzy's TransformListener subscribes to absolute /tf topics;
            # fleet_agent.launch.py remaps them into this robot's namespace.
            self._tf_listener = TransformListener(
                self._tf_buffer, self, spin_thread=False)

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
        self.pub_trelease = self.create_publisher(RosTaskRelease, T_TASK_RELEASE, COORD_QOS)

        # per-robot (relative names -> namespaced)
        self.pub_permit = self.create_publisher(RosPermit, "motion_permit", PERMIT_QOS)
        self.pub_conflict = self.create_publisher(RosConflict, "conflicts", DIAGNOSTIC_QOS)
        self.pub_path = self.create_publisher(Path, "coordination_path", PATH_QOS)
        self.pub_diag = self.create_publisher(String, "fleet_diagnostics", DIAGNOSTIC_QOS)

        # --------------------------------------------------------- subscribers
        self.create_subscription(Odometry, "odom", self.cb_odom,
                                 SENSOR_QOS, callback_group=self.g_sensor)
        self.create_subscription(LaserScan, "scan", self.cb_scan,
                                 SENSOR_QOS, callback_group=self.g_sensor)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose",
                                 self.cb_amcl_pose, 10,
                                 callback_group=self.g_sensor)
        # AMCL only updates when the robot moves; a LOST robot is stopped, so
        # it asks AMCL for stationary updates or it could never recover.
        self._nomotion_cli = (
            self.create_client(EmptySrv, "request_nomotion_update")
            if EmptySrv is not None else None)

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
        self.create_subscription(RosTaskRelease, T_TASK_RELEASE, self.cb_task_release,
                                 COORD_QOS, callback_group=self.g_fleet)
        # R6: webapp abort before pickup (sih-57 publishes on this topic).
        if RosTaskCancel is not None:
            self.create_subscription(RosTaskCancel, "/fleet/task_cancel",
                                     self.cb_task_cancel, COORD_QOS,
                                     callback_group=self.g_fleet)
        else:
            self.get_logger().warning(
                "TaskCancel msg not installed (stale build): "
                "/fleet/task_cancel aborts are disabled on this robot")

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
            release=self._send_task_release,
            logger=lambda m: self.get_logger().info(m))

        t_suspect = max(0.1, float(self.get_parameter("peer_suspect_s").value))
        t_dead = max(t_suspect + 0.1, float(self.get_parameter("peer_dead_s").value))
        self.coord.registry.t_suspect = t_suspect
        self.coord.registry.t_dead = t_dead
        self.auction.task_release_s = max(TASK_RELEASE_S, t_dead)

        # Until a map-frame pose is available, this agent must neither accept
        # tasks nor publish itself as a healthy participant at (0, 0).
        self.coord.state.status = FAULT
        self.coord.state.alive = False

        # ---------------------------------------------------- runtime state
        self.state_seq = 0
        self.coord.state.boot_id = secrets.randbits(64)
        self.intent_seq = 0
        self._odom: Optional[Odometry] = None
        self._scan: Optional[LaserScan] = None
        self._scan_generation = 0
        self._processed_scan_generation = 0
        self._first_odom_t: Optional[float] = None
        self._map_pose_valid = False
        self._last_scan_t = 0.0
        self._last_odom_t = 0.0
        self._last_intent_pub = 0.0
        self._last_intent_hash = None
        self._last_path_signature = None
        self._hits = {}
        self._clear_hits = {}
        # Cell -> most recent report time. Persistent obstacles must refresh
        # their map lease before expiry; a set would publish only once and
        # let a continuously visible pallet vanish from the shared map later.
        self._reported = {}
        self._distance_m = 0.0
        # Battery integrates drain per metre and dock charging per second, so
        # it can refill; these remember what the last update already counted.
        self._battery_distance_seen = 0.0
        self._battery_t: Optional[float] = None
        self._task_phase = None
        self._route_failure_since = None
        self._reported_map_tf_wait = False
        self._last_xy = None
        # Arrival dwell: STOP until _dwell_until, then set the deferred goal
        # _dwell_resume = (task_id, cell) if that task is still mine.
        self._dwell_until: Optional[float] = None
        self._dwell_resume = None
        # Localization health. None until AMCL's first pose: the gate never
        # fires before then (the map-TF wait gate covers startup).
        self._amcl_cov = None
        self._amcl_cov_xy: Optional[float] = None
        self._loc_lost = False
        self._loc_bad_since: Optional[float] = None
        self._loc_good_since: Optional[float] = None
        self._last_nomotion_req = float("-inf")
        # (map pose, odom pose) at the last CONFIDENT localization, and the
        # re-seed attempts made during the current LOST episode.
        self._loc_anchor = None
        self._reseed_attempts = 0
        self._last_reseed = float("-inf")
        self._amcl_pose_t = None
        self._last_parked_nomotion = float("-inf")
        self._pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10)
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
        now = self._now()
        self._last_odom_t = now
        if self._first_odom_t is None:
            self._first_odom_t = now
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y

        if self._last_xy is not None:
            self._distance_m += math.hypot(x - self._last_xy[0],
                                           y - self._last_xy[1])
        self._last_xy = (x, y)

        if self.pose_source == "odom":
            q = msg.pose.pose.orientation
            theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.coord.state.pose = Pose2D(x, y, theta)
            self._map_pose_valid = True
        self.coord.state.v = msg.twist.twist.linear.x
        self.coord.state.w = msg.twist.twist.angular.z

    def _update_map_pose(self, now: float) -> bool:
        if self.pose_source == "odom":
            return self._map_pose_valid
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException:
            return False

        stamp = (transform.header.stamp.sec
                 + transform.header.stamp.nanosec * 1e-9)
        timeout = max(0.0, float(
            self.get_parameter("map_pose_timeout_s").value))
        age = now - stamp
        if stamp <= 0.0 or age > timeout or age < -0.1:
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        theta = math.atan2(2.0 * (rotation.w * rotation.z
                                  + rotation.x * rotation.y),
                           1.0 - 2.0 * (rotation.y * rotation.y
                                        + rotation.z * rotation.z))
        self.coord.state.pose = Pose2D(translation.x, translation.y, theta)
        self._map_pose_valid = True
        return True

    def cb_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_generation += 1
        self._last_scan_t = self._now()

    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose_t = self._now()
        cov = msg.pose.covariance
        self._amcl_cov = (cov[0], cov[7], cov[35])
        self._amcl_cov_xy = cov[1]

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
        st = cv.state_from_ros(msg)
        now = self._now()
        self.coord.on_peer_state(st, now=now)
        # R4(4): the peer's broadcast task_id is claim ground truth at state
        # rate. A duplicate claim resolves identically on both robots
        # (LOADED holder wins, else lowest id); the loser drops silently.
        lost = self.auction.on_peer_task(
            st.robot_id, st.task_id, self._class_of_score(st.priority_score),
            now, self._my_task_class())
        if lost:
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._set_task_phase(None)
            self._publish_path()

    def cb_peer_intent(self, msg: RosIntent) -> None:
        if msg.robot_id == self.robot_id:
            return
        # The registry keeps intents apart from the 10 Hz state (which would
        # otherwise erase them) and buffers ones from not-yet-seen peers.
        now = self._now()
        self.coord.registry.update_intent(
            msg.robot_id, cv.intent_from_ros(msg, rx_time=now), int(msg.seq),
            now=now, stamp=cv._stamp_to_float(msg.header.stamp))

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
        already_running = (self.coord.current_task is not None and
                           self.coord.current_task.task_id == msg.task_id)
        if msg.winner_id == self.robot_id and already_running:
            return                           # echo of my own ownership re-assert
        if msg.winner_id == self.robot_id and self.auction.my_task is not None:
            self.coord.current_task = self.auction.my_task
            self.coord.state.task_id = self.auction.my_task.task_id
            self.coord.state.task_priority = self.auction.my_task.priority
            self._set_task_phase("PICKUP")
            # R5: the aging/cycle clock starts when the task was first heard.
            self.coord.task_t0 = self.auction.first_heard.get(
                msg.task_id, self._now())
            if self.dock is not None:
                self.dock.reset()            # leaving the dock for work
            if self._dwell_until is not None:
                # Won during an unload dwell: drive off only once it ends.
                self._dwell_resume = (self.auction.my_task.task_id,
                                      self.auction.my_task.pickup)
            else:
                self.coord.set_goal(self.auction.my_task.pickup,
                                    now=self._now())
        elif (msg.winner_id != self.robot_id
              and self.coord.current_task is not None
              and self.coord.current_task.task_id == msg.task_id
              and self.auction.my_task is None):
            # Lost a claim collision. Clear the navigation target immediately,
            # and release zone state like complete/release do — a parked loser
            # that keeps REQUESTING/HELD zones silently vetoes them for every
            # peer (grant deadlock at the corridor).
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self._set_task_phase(None)
            self.coord.arbiter.release_all()
            self._publish_path()

    def cb_task_complete(self, msg: RosTaskComplete) -> None:
        self.auction.on_complete(msg.task_id)
        if (self.coord.current_task is not None
                and self.coord.current_task.task_id == msg.task_id):
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._set_task_phase(None)

    def cb_task_release(self, msg: RosTaskRelease) -> None:
        # The releasing agent already reopened its local auction. The release
        # event lets peers clear the stale assignee and join that bid window.
        if msg.robot_id == self.robot_id:
            return
        from amr_fleet.core.models import Task
        task = Task(
            task_id=msg.task_id,
            pickup=(int(msg.pickup_row), int(msg.pickup_col)),
            dropoff=(int(msg.dropoff_row), int(msg.dropoff_col)),
            priority=int(msg.priority), created_at=float(msg.created_at),
            deadline=float(msg.deadline))
        if not self.auction.on_release(task, msg.robot_id, now=self._now()):
            return
        if (self.coord.current_task is not None and
                self.coord.current_task.task_id == msg.task_id and
                self.auction.my_task is None):
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._set_task_phase(None)
            self._publish_path()

    def cb_task_cancel(self, msg) -> None:
        """R6: webapp abort. Refused once loaded (phase DROPOFF) — the robot
        finishes the delivery; the webapp infers REFUSED from the unchanged
        RobotState.task_id. Before pickup the task dies fleet-wide: the
        'cancelled' set blocks every resurrection path, and NO TaskComplete
        is sent (experiment_recorder must not count an abort)."""
        now = self._now()
        tid = msg.task_id
        mine = (self.auction.my_task is not None
                and self.auction.my_task.task_id == tid)
        if mine and self._task_phase == "DROPOFF":
            self.auction.on_cancel(tid, now, my_phase=self._task_phase)
            self.get_logger().warning(
                f"cancel refused for {tid} (from {msg.requester_id}): "
                "already loaded, continuing to dropoff")
            return
        self.auction.on_cancel(tid, now,
                               my_phase=self._task_phase if mine else None)
        if mine:
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._set_task_phase(None)
            if (self._dwell_resume is not None
                    and self._dwell_resume[0] == tid):
                self._dwell_resume = None       # never resume an aborted task
            self._publish_path()
            self.get_logger().info(
                f"task {tid} cancelled by {msg.requester_id or 'unknown'} "
                f"({msg.reason or 'no reason'}); aborted before pickup")

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

    def _send_task_release(self, task: dict, reason: str) -> None:
        m = RosTaskRelease()
        m.header.stamp = self.get_clock().now().to_msg()
        m.task_id = task["task_id"]
        m.pickup_row = int(task["pickup_row"])
        m.pickup_col = int(task["pickup_col"])
        m.dropoff_row = int(task["dropoff_row"])
        m.dropoff_col = int(task["dropoff_col"])
        m.priority = int(task.get("priority", 0))
        m.created_at = float(task.get("created_at", 0.0))
        m.deadline = float(task.get("deadline", 0.0))
        m.robot_id = self.robot_id
        m.reason = reason
        self.pub_trelease.publish(m)

    # ================================================================= tick ==
    def tick(self) -> None:
        now = self._now()

        # ---- health gates. Fail safe: when uncertain, STOP. A stopped robot
        # is a static obstacle peers can route around; a confused moving robot
        # is a hazard.
        if self._odom is None:
            return
        odom_timeout = float(self.get_parameter("odom_timeout_s").value)
        scan_timeout = float(self.get_parameter("scan_timeout_s").value)
        if (now - self._last_odom_t) > odom_timeout:
            self._fault("odom timeout")
            return
        if self._scan is None:
            startup_timeout = max(
                0.0, float(self.get_parameter("scan_startup_timeout_s").value))
            if (self._first_odom_t is not None and
                    now - self._first_odom_t > startup_timeout):
                self._fault("scan unavailable after startup")
                return
        elif (now - self._last_scan_t) > scan_timeout:
            self._fault("scan timeout")
            return

        if not self._update_map_pose(now):
            self.coord.state.status = FAULT
            self.coord.state.alive = False
            if not self._reported_map_tf_wait:
                self.get_logger().warning(
                    f"waiting for fresh {self.map_frame} -> {self.base_frame} TF; "
                    "task bidding and motion remain disabled")
                self._reported_map_tf_wait = True
            self._publish_stop_permit("waiting for fresh map-frame pose")
            startup_timeout = max(0.0, float(
                self.get_parameter("map_pose_startup_timeout_s").value))
            if (self._first_odom_t is not None and
                    now - self._first_odom_t > startup_timeout):
                self._fault("map-frame pose unavailable or stale")
            return

        self._reported_map_tf_wait = False
        if self.coord.state.status == FAULT and (now - self._last_odom_t) < 0.5:
            self.coord.state.status = IDLE       # recovered
            self.coord.state.alive = True

        self._update_battery(now)
        localized = self._update_localization_health(now)
        # A LOST pose must neither declare arrivals nor project lidar hits
        # into the shared map (it would broadcast phantom blockages).
        if localized:
            self._advance_task_lifecycle(now)
            if (self.get_parameter("enable_obstacle_reporting").value
                    and self._scan):
                self._detect_obstacles(now)

        try:
            permit = self.coord.tick(now)
        except Exception as exc:
            self.get_logger().error(f"coordination tick failed: {exc}")
            self._fault(f"tick exception: {exc}")
            return
        # The coordinator re-derives IDLE/MOVING every tick; a parked robot
        # on its dock reports CHARGING instead.
        if (self.dock is not None and self.dock.phase == docking.DOCKED
                and self.coord.state.status == IDLE):
            self.coord.state.status = CHARGING

        if not localized:
            self._publish_stop_permit("LOCALIZATION LOST")
        elif self._dwell_until is not None:
            self._publish_stop_permit("arrival dwell")
        elif self.dock is not None and self.dock.hold_motion:
            self._publish_stop_permit("dock alignment: peer too close to rotate")
        else:
            permit = self._maybe_retreat(permit, now)
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
        # Relinquish the task and withdraw its route before another robot can
        # take over. Otherwise a recovered agent could resume a stale goal
        # after its permit returns to GO, despite having re-auctioned the task.
        self.auction.release_current(now=self._now(), reason=reason)
        self.coord.current_task = None
        self.coord.state.task_id = ""
        self.coord.state.task_priority = 0
        self.coord.clear_goal()
        self._task_phase = None
        self._dwell_until = None
        self._dwell_resume = None
        if self.dock is not None:
            self.dock.reset()
        self._publish_path()
        self._publish_stop_permit(f"FAULT: {reason}")

    def _publish_stop_permit(self, reason: str) -> None:
        p = RosPermit()
        p.header.stamp = self.get_clock().now().to_msg()
        p.robot_id = self.robot_id
        p.action = 2                          # STOP
        p.speed_scale = 0.0
        p.reason = reason
        self.pub_permit.publish(p)
        self._last_scale = 0.0

    # ============================================================ publishing =
    def publish_state(self) -> None:
        if not self._map_pose_valid:
            return
        self.state_seq += 1
        self.coord.state.seq = self.state_seq
        self.coord.state.stamp = self._now()
        self.coord.state.intent_seq = self.intent_seq
        self.coord.state.loc_health = self._loc_health()
        self.coord.state.loc_sigma_lat = self._loc_sigma_lat()
        self.pub_state.publish(
            cv.state_to_ros(self.coord.state, self.get_clock()))

    def _maybe_publish_intent(self, now: float) -> None:
        """Publish on change, plus a 1 Hz keepalive.

        Publishing a 40-cell intent at 10 Hz would be wasteful; publishing only
        on change would leave a late joiner blind. TRANSIENT_LOCAL plus a
        keepalive covers both.

        Every publish sends coord.current_intent(now): the undriven suffix,
        re-timed from now and header-stamped with the same now (receivers
        convert with rx_time + t - header.stamp). intent_seq is bumped only
        when the plan changes (route or needed zones); a keepalive re-sends
        the same seq, so RobotState.intent_seq ahead of a peer's held intent
        means that peer really is missing my current plan.
        """
        cur = self.coord.current_intent(now)
        h = (tuple(self.coord.intent.cells), tuple(self.coord.intent.zones),
             tuple(cur.zones))
        keepalive_period = 1.0 / float(
            self.get_parameter("intent_keepalive_hz").value)
        changed = h != self._last_intent_hash
        if changed or (now - self._last_intent_pub) > keepalive_period:
            if changed:
                self.intent_seq += 1
            self.pub_intent.publish(cv.intent_to_ros(
                self.robot_id, self.intent_seq, cur,
                self.get_clock(), self.coord.state.task_id,
                self.coord.state.task_priority, self.coord.state.priority_score,
                stamp=now))
            self._last_intent_hash = h
            self._last_intent_pub = now

    def _publish_path(self) -> None:
        final = self._dock_final_pose()
        signature = (tuple(self.coord.path), final)
        if signature == self._last_path_signature:
            return
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.map_frame
        from geometry_msgs.msg import PoseStamped
        path = self.coord.path
        for index, cell in enumerate(path):
            x, y = self.grid.cell_to_world(cell)
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x, ps.pose.position.y = x, y
            if len(path) > 1:
                neighbor = path[index + 1] if index + 1 < len(path) else path[index - 1]
                nx, ny = self.grid.cell_to_world(neighbor)
                dx, dy = ((nx - x, ny - y) if index + 1 < len(path)
                          else (x - nx, y - ny))
                yaw = math.atan2(dy, dx)
                ps.pose.orientation.z = math.sin(yaw * 0.5)
                ps.pose.orientation.w = math.cos(yaw * 0.5)
            else:
                yaw = self.coord.state.pose.theta
                ps.pose.orientation.z = math.sin(yaw * 0.5)
                ps.pose.orientation.w = math.cos(yaw * 0.5)
            p.poses.append(ps)
        if final is not None and p.poses:
            # Docking: end EXACTLY on the spawn pose (not the cell centre),
            # facing the spawn yaw - Nav2's controller turns to the final
            # pose's heading on arrival. A one-cell path gets my current pose
            # in front so the controller has a segment to follow.
            fx, fy, fyaw = final
            if len(p.poses) == 1:
                cur = PoseStamped()
                cur.header = p.header
                cur.pose.position.x = self.coord.state.pose.x
                cur.pose.position.y = self.coord.state.pose.y
                cur.pose.orientation.z = math.sin(fyaw * 0.5)
                cur.pose.orientation.w = math.cos(fyaw * 0.5)
                p.poses.insert(0, cur)
            last = p.poses[-1].pose
            last.position.x, last.position.y = fx, fy
            last.orientation.z = math.sin(fyaw * 0.5)
            last.orientation.w = math.cos(fyaw * 0.5)
        self.pub_path.publish(p)
        self._last_path_signature = signature

    def publish_diagnostics(self) -> None:
        import json
        m = self.coord.metrics()
        m.update(self.auction.stats)
        m["distance_m"] = round(self._distance_m, 2)
        m["battery_pct"] = round(self.coord.state.battery_pct, 1)
        m["status"] = self.coord.state.status
        m["localization_lost"] = self._loc_lost
        m["reloc_attempts"] = self._reseed_attempts
        m["arrival_dwell"] = self._dwell_until is not None
        m["task_id"] = self.coord.state.task_id or None
        m["task_phase"] = (self._task_phase
                           if self.auction.my_task is not None else None)
        m["dock_phase"] = self.dock.phase if self.dock is not None else None
        m["charge_hold"] = bool(self.dock is not None and self.dock.charge_hold)
        m["home_cell"] = (list(self.dock.home_cell)
                          if self.dock is not None else None)
        if self.dock is not None:
            pose = self.coord.state.pose
            pe, ye = self.dock.errors(pose.x, pose.y, pose.theta)
            m["dock_err_m"] = round(pe, 3)
            m["dock_err_deg"] = round(math.degrees(ye), 1)
            m["dock_misaligned"] = self.dock.misaligned
            m["dock_retries"] = self.dock.retries
            m["dock_hold"] = self.dock.hold_motion
        # Hitbox spec: true while this robot counts as its 0.286 m disc
        # (spinning, or about to turn > 45 deg). None until the coordinator
        # provides it; the web app then falls back to |w| > 0.3.
        disc = getattr(self.coord, "hitbox_disc_mode", None)
        m["hitbox_disc_mode"] = None if disc is None else bool(
            disc() if callable(disc) else disc)
        # Legal lane-crossing manoeuvres (strict lanes): a gated U-turn into
        # the opposite lane, or a retreat/make-way driving against my lane.
        # The web app badges them so they read as deliberate, not as faults.
        m["uturn_active"] = bool(getattr(self.coord, "uturn_active", False))
        m["reverse_active"] = bool(getattr(self.coord, "reverse_active", False))
        # Gated opposite-lane pass of a robot frozen mid-lane (street overtake).
        m["overtake_active"] = bool(getattr(self.coord, "overtake_active", False))
        # This robot's view of the P2P layer, for the web app: how fresh each
        # peer looks from HERE, and the zone locks it holds or is acquiring.
        now = self._now()
        reg = self.coord.registry
        m["peers"] = {
            rid: {"fresh": reg.freshness(rid, now),
                  "age_ms": round(max(0.0, now - reg.last_seen.get(rid, now))
                                  * 1000.0),
                  "intent_seq": reg.held_intent_seq(rid),
                  "plan_unknown": bool(reg.plan_unknown(rid)),
                  "loss": round(reg.loss_rate(rid), 4)}
            for rid in reg.peers}
        arb = self.coord.arbiter
        m["zones_held"] = sorted(arb.held_zones())
        m["zones_requesting"] = {
            zid: sorted(arb.grants.get(zid, ()))
            for zid, st in arb.state.items() if st == "REQUESTING"}
        s = String(); s.data = json.dumps(m)
        self.pub_diag.publish(s)

    # =========================================================== obstacles ===
    def _detect_obstacles(self, now: float) -> None:
        """Filter lidar hits and clear known dynamic cells with ray evidence."""
        if self._scan_generation == self._processed_scan_generation:
            return
        self._processed_scan_generation = self._scan_generation
        scan = self._scan
        # Project the scan with the pose AT SCAN TIME. Using the latest TF
        # while the robot rotates (or while the executor lags) rotates every
        # wall into free space - that produced the diagonal 'blockage' lines
        # that poisoned the shared map and froze the fleet.
        pose = self._pose_at_scan(scan, now)
        if pose is None:
            return
        need = int(self.get_parameter("obstacle_persist_scans").value)
        clear_need = max(1, int(self.get_parameter("obstacle_clear_scans").value))
        expiry_s = float(self.get_parameter("obstacle_expiry_s").value)
        peer_r = float(self.get_parameter("obstacle_peer_radius_m").value)
        peer_r2 = peer_r * peer_r
        peer_xy = [(p.pose.x, p.pose.y)
                   for p in self.coord.registry.peers.values()]
        observed = set()
        clear_observed = set()
        dynamic_before_scan = set(self.grid.dynamic)
        resolution = self.grid.resolution
        max_distance = min(scan.range_max, 6.0)

        step = max(1, len(scan.ranges) // 90)     # subsample: Nano-friendly
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]
            if math.isnan(r) or r < scan.range_min:
                continue
            a = scan.angle_min + i * scan.angle_increment + pose.theta
            hit_distance = min(r, max_distance)
            if r < max_distance and math.isfinite(r):
                hx = pose.x + r * math.cos(a)
                hy = pose.y + r * math.sin(a)
                cell = self.grid.world_to_cell(hx, hy)
                if (self.grid.is_static_free(cell)
                        and cell not in self._near_static
                        and not any((hx - px) ** 2 + (hy - py) ** 2 <= peer_r2
                                    for px, py in peer_xy)):
                    observed.add(cell)
                    self._hits[cell] = min(self._hits.get(cell, 0) + 1, need)
                # Do not use the endpoint itself as free-space evidence.
                hit_distance = max(0.0, r - 0.5 * resolution)

            # Walk the observed free part of the ray. Only dynamic cells need
            # consideration, keeping this bounded by the small dynamic map.
            if dynamic_before_scan:
                ray_steps = max(
                    1, int(hit_distance / max(0.5 * resolution, 1e-3)))
                for ray_step in range(1, ray_steps + 1):
                    distance = hit_distance * ray_step / ray_steps
                    cell = self.grid.world_to_cell(
                        pose.x + distance * math.cos(a),
                        pose.y + distance * math.sin(a))
                    if cell in dynamic_before_scan:
                        clear_observed.add(cell)

        # A cell hit this scan must not also count as clear evidence from its
        # own (or a neighbouring) beam's free-space walk — without this the
        # detector is a self-clearing oscillator (report/clear every ~4 scans).
        clear_observed.difference_update(observed)

        newly = []
        for cell, n in list(self._hits.items()):
            if cell not in observed:
                # obstacle_persist_scans means CONSECUTIVE scans: one miss
                # resets the run (a +1/-1 random walk crosses any threshold
                # given enough time, which is how phantom cells ignited).
                del self._hits[cell]
                self._reported.pop(cell, None)
            elif (n >= need and
                  now - self._reported.get(cell, float('-inf'))
                  >= max(1.0, expiry_s * 0.5)):
                self._reported[cell] = now
                newly.append(cell)

        cleared = []
        self._clear_hits = {
            cell: count for cell, count in self._clear_hits.items()
            if cell in self.grid.dynamic
        }
        for cell in dynamic_before_scan:
            if cell in clear_observed:
                self._clear_hits[cell] = self._clear_hits.get(cell, 0) + 1
                if self._clear_hits[cell] >= clear_need:
                    cleared.append(cell)
                    self._clear_hits.pop(cell, None)
            else:
                self._clear_hits.pop(cell, None)

        if cleared:
            blocked_before = self.grid.blocked_cells(now)
            self.grid.report_cleared(cleared, self.robot_id)
            newly_unblocked = bool(
                blocked_before.intersection(cleared)
                - self.grid.blocked_cells(now))
            self.pub_map.publish(cv.map_update_to_ros(
                self.robot_id, [], cleared, 0.0, now, self.get_clock()))
            self.get_logger().info(f"cleared dynamic blockage {cleared} -> broadcast")
            self._reported = {cell: stamp for cell, stamp in self._reported.items()
                              if cell not in cleared}
            # A re-report after a clear must earn a fresh consecutive run.
            for cell in cleared:
                self._hits.pop(cell, None)
                self._clear_hits.pop(cell, None)
            if newly_unblocked and self.coord.goal_cell is not None:
                self.coord.replan(now=now)

        if newly:
            conf = min(1.0, need * 0.2)
            expiry = now + expiry_s
            self.grid.report_blocked(newly, self.robot_id, conf, expiry)
            self.pub_map.publish(cv.map_update_to_ros(
                self.robot_id, newly, [], conf, expiry, self.get_clock()))
            self.get_logger().info(f"blockage detected {newly} -> broadcast")
            if self.coord._path_invalidated():
                self.coord.replan(now=now)

    def _maybe_retreat(self, permit, now: float):
        """Execute the YIELD manoeuvre the permit layer can only signal.

        velocity_gate passes GO/SLOW only (fail-safe actuator contract), so a
        deadlock victim's YIELD becomes a normal short drive: pick a nearby
        static-free cell clear of peers, make it the goal, then restore the
        task route. This physically opens three-body knots the zone handoffs
        cannot untangle.
        """
        if self._retreat_until is not None:
            pose = self.coord.state.pose
            tx, ty = self.grid.cell_to_world(self._retreat_cell)
            done = math.hypot(pose.x - tx, pose.y - ty) <= 0.2
            if done or now >= self._retreat_until:
                self._retreat_cell = None
                self._retreat_until = None
                self._retreat_cooldown_until = now + 10.0
                task = self.auction.my_task
                if task is not None:
                    resume = (task.pickup
                              if self._task_phase in (None, "PICKUP")
                              else task.dropoff)
                    self.coord.set_goal(resume, now=now)
                    self._publish_path()
                    self.get_logger().info(
                        "retreat finished; resuming task route")
                else:
                    self.coord.clear_goal()
            return permit

        if permit.action != "YIELD" or now < self._retreat_cooldown_until:
            return permit
        cell = self._pick_retreat_cell()
        if cell is None:
            return permit
        self._retreat_cell = cell
        self._retreat_until = now + 8.0
        self.coord.set_goal(cell, now=now)
        self._publish_path()
        self.get_logger().info(
            f"YIELD: retreating to {cell} to clear "
            f"{permit.blocking_robot or 'the deadlock'}")
        try:
            # Re-run the permit for the retreat route so the gate can pass it.
            return self.coord.tick(now)
        except Exception:
            return permit

    def _pick_retreat_cell(self):
        """A static-free cell 0.8-1.8 m away maximising clearance to peers."""
        pose = self.coord.state.pose
        peers = [(p.pose.x, p.pose.y)
                 for p in self.coord.registry.alive().values()]
        res = self.grid.resolution
        cur = self.grid.world_to_cell(pose.x, pose.y)
        span = int(1.8 / res)
        best, best_clear = None, 0.0
        for dr in range(-span, span + 1):
            for dc in range(-span, span + 1):
                d = math.hypot(dr, dc) * res
                if not 0.8 <= d <= 1.8:
                    continue
                cell = (cur[0] + dr, cur[1] + dc)
                if (not self.grid.is_static_free(cell)
                        or cell in self._near_static):
                    continue
                x, y = self.grid.cell_to_world(cell)
                clear = min((math.hypot(x - px, y - py)
                             for px, py in peers), default=9.9)
                if clear < 0.7:
                    continue
                if clear > best_clear:
                    best, best_clear = cell, clear
        return best

    def _advance_task_lifecycle(self, now: float) -> None:
        """Drive the auction winner through pickup -> dropoff -> complete."""
        if self._dwell_until is not None:
            if now < self._dwell_until:
                return
            self._end_dwell(now)

        task = self.auction.my_task
        if task is None:
            self._set_task_phase(None)
            self._route_failure_since = None
            self._advance_dock(now)
            return

        if self.coord.goal_cell is not None and not self.coord.path:
            if self._route_failure_since is None:
                self._route_failure_since = now
            timeout = max(0.0, float(
                self.get_parameter("task_route_failure_timeout_s").value))
            if now - self._route_failure_since >= timeout:
                self.get_logger().warning(
                    f"task {task.task_id} route unavailable; returning it to fleet auction")
                self.auction.release_current(
                    now=now, reason="route unavailable after map update")
                self.coord.current_task = None
                self.coord.state.task_id = ""
                self.coord.state.task_priority = 0
                self.coord.clear_goal()
                self.coord.arbiter.release_all()
                self._set_task_phase(None)
                self._route_failure_since = None
                self._publish_path()
            return
        self._route_failure_since = None

        target = task.pickup if self._task_phase in (None, "PICKUP") else task.dropoff
        pose = self.coord.state.pose
        tx, ty = self.grid.cell_to_world(target)
        dist = math.hypot(pose.x - tx, pose.y - ty)
        # Must exceed Nav2's goal_checker xy_goal_tolerance (0.15): FollowPath
        # can SUCCEED up to 0.15 m off-center and the goal bridge latches the
        # completed path, so an equal threshold can park a task at the pickup.
        threshold = max(0.25, 0.5 * self.grid.resolution)
        # Inside the threshold, wait for Nav2 to finish converging (robot at
        # rest) so the dwell STOP lands on the marker, not 0.25 m short of it.
        settled = (abs(self.coord.state.v) < 0.02
                   and abs(self.coord.state.w) < 0.05)
        arrived = dist <= threshold and (settled or dist <= 0.1)

        if self._task_phase in (None, "PICKUP") and arrived:
            self._set_task_phase("DROPOFF")   # LOADED: cancel refused from here
            self._start_dwell(now, (task.task_id, task.dropoff))
            self.get_logger().info(
                f"task {task.task_id}: pickup reached ({dist:.2f} m off), "
                f"loading for {self._dwell_s():.1f}s then heading to dropoff")
            return

        if self._task_phase == "DROPOFF" and arrived:
            tid = task.task_id
            self._send_task_complete(tid, now)
            self.auction.complete_current(now)
            self.coord.current_task = None
            self.coord.state.task_id = ""
            self.coord.state.task_priority = 0
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._set_task_phase(None)
            self._start_dwell(now, None)
            self.get_logger().info(
                f"task {tid}: completed ({dist:.2f} m off), "
                f"unloading for {self._dwell_s():.1f}s")

    def _dwell_s(self) -> float:
        return max(0.0, float(self.get_parameter("arrival_dwell_s").value))

    def _start_dwell(self, now: float, resume) -> None:
        """Hold STOP at a reached marker; `resume` = (task_id, next goal)."""
        self._dwell_until = now + self._dwell_s()
        self._dwell_resume = resume
        if self._retreat_until is not None:
            # A retreat finishing mid-dwell would set its own resume goal and
            # bypass the deferred one; the task reached its marker anyway.
            self._retreat_cell = None
            self._retreat_until = None
            self._retreat_cooldown_until = now + 10.0

    def _end_dwell(self, now: float) -> None:
        self._dwell_until = None
        resume, self._dwell_resume = self._dwell_resume, None
        task = self.auction.my_task
        if resume is not None and task is not None and task.task_id == resume[0]:
            self.coord.set_goal(resume[1], now=now)

    def _update_localization_health(self, now: float) -> bool:
        """False while LOST: AMCL covariance too wide for loc_lost_after_s."""
        if self._amcl_cov is None:
            return True       # no AMCL pose yet (or pose_source=odom)
        cxx, cyy, cyaw = self._amcl_cov
        sigma_xy = math.sqrt(max(0.0, cxx + cyy))
        sigma_yaw = math.sqrt(max(0.0, cyaw))
        bad = (sigma_xy > float(self.get_parameter("loc_max_sigma_xy_m").value)
               or sigma_yaw > float(
                   self.get_parameter("loc_max_sigma_yaw_rad").value))
        if bad:
            self._loc_good_since = None
            if self._loc_bad_since is None:
                self._loc_bad_since = now
            if (not self._loc_lost and now - self._loc_bad_since > float(
                    self.get_parameter("loc_lost_after_s").value)):
                self._loc_lost = True
                self.get_logger().error(
                    f"LOCALIZATION LOST: sigma_xy={sigma_xy:.2f}m "
                    f"sigma_yaw={sigma_yaw:.2f}rad for "
                    f"{now - self._loc_bad_since:.1f}s - motion stopped, "
                    "task bidding suspended")
        else:
            self._loc_bad_since = None
            if not self._loc_lost:
                self._note_loc_anchor()
            if self._loc_lost:
                if self._loc_good_since is None:
                    self._loc_good_since = now
                if now - self._loc_good_since >= float(
                        self.get_parameter("loc_recover_after_s").value):
                    self._loc_lost = False
                    self._loc_good_since = None
                    self._reseed_attempts = 0
                    self.get_logger().info(
                        f"LOCALIZATION RECOVERED: sigma_xy={sigma_xy:.2f}m "
                        f"sigma_yaw={sigma_yaw:.2f}rad - resuming")
        if self._loc_lost:
            self._maybe_reseed(now)
            self._request_nomotion_update(now)
        else:
            self._parked_relocalize(now)
        return not self._loc_lost

    def _parked_relocalize(self, now: float) -> None:
        """A stationary robot never triggers AMCL's motion-gated updates, so
        a wrong-but-confident pose would persist for as long as it waits.
        While parked with a stale amcl_pose, force a scan correction."""
        if self._amcl_pose_t is None or self._odom is None:
            return
        tw = self._odom.twist.twist
        if abs(tw.linear.x) > 0.02 or abs(tw.angular.z) > 0.05:
            return
        after = float(self.get_parameter("loc_stationary_nomotion_after_s").value)
        period = float(self.get_parameter("loc_stationary_nomotion_period_s").value)
        if (now - self._amcl_pose_t < after
                or now - self._last_parked_nomotion < period):
            return
        self._last_parked_nomotion = now
        if self._nomotion_cli is not None and self._nomotion_cli.service_is_ready():
            self._nomotion_cli.call_async(EmptySrv.Request())

    # ------------------------------------------------- LOST recovery -----
    # Why dead reckoning: the equal-aisle layout makes neighbouring aisles
    # look identical to the lidar, so a lost AMCL cloud goes BIMODAL (its
    # mean can even land inside a rack) and stationary no-motion updates
    # cannot pick the right aisle. Wheel odometry drifts centimetres per
    # metre - far below the 2.2 m aisle pitch - so last-confident-pose +
    # odometry selects the correct mode. Uses only on-robot data, never
    # simulator ground truth. A global re-localization would instead spread
    # particles over every identical aisle.
    @staticmethod
    def _yaw_of(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom_xyt(self):
        if self._odom is None:
            return None
        p = self._odom.pose.pose
        return (p.position.x, p.position.y, self._yaw_of(p.orientation))

    def _note_loc_anchor(self) -> None:
        odom = self._odom_xyt()
        if odom is None or not self._map_pose_valid:
            return
        m = self.coord.state.pose
        self._loc_anchor = ((m.x, m.y, m.theta), odom)

    def _dead_reckoned_pose(self):
        """Anchor map pose composed with the odometry delta since then, and
        the odometry distance travelled (drives the seed's covariance)."""
        odom = self._odom_xyt()
        if self._loc_anchor is None or odom is None:
            return None
        (mx, my, mt), (ox, oy, ot) = self._loc_anchor
        dx, dy = odom[0] - ox, odom[1] - oy
        # delta expressed in the anchor's body frame, then in the map frame
        c, s = math.cos(-ot), math.sin(-ot)
        bx, by = c * dx - s * dy, s * dx + c * dy
        cm, sm = math.cos(mt), math.sin(mt)
        x = mx + cm * bx - sm * by
        y = my + sm * bx + cm * by
        t = math.atan2(math.sin(mt + odom[2] - ot), math.cos(mt + odom[2] - ot))
        return (x, y, t), math.hypot(dx, dy)

    @staticmethod
    def reseed_decision(travelled_m: float, lost_for_s: float,
                        min_odom_m: float, hold_s: float,
                        max_odom_m: float) -> str:
        """'seed', 'wait' (let AMCL's injected cloud converge first) or
        'too_far' (odometry no longer trustworthy). Pure: unit-tested."""
        if travelled_m > max_odom_m:
            return "too_far"
        if lost_for_s < hold_s or travelled_m < min_odom_m:
            return "wait"
        return "seed"

    def _maybe_reseed(self, now: float) -> None:
        if (not bool(self.get_parameter("loc_reseed_enabled").value)
                or self._reseed_attempts >= int(
                    self.get_parameter("loc_reseed_max_attempts").value)
                or now - self._last_reseed < float(
                    self.get_parameter("loc_reseed_period_s").value)):
            return
        est = self._dead_reckoned_pose()
        if est is None:
            return
        (x, y, t), travelled = est
        lost_for = now - (self._loc_bad_since if self._loc_bad_since is not None
                          else now)
        verdict = self.reseed_decision(
            travelled, lost_for,
            float(self.get_parameter("loc_reseed_min_odom_m").value),
            float(self.get_parameter("loc_reseed_hold_s").value),
            float(self.get_parameter("loc_reseed_max_odom_m").value))
        if verdict == "wait":
            return          # no new evidence: the nomotion updates keep going
        if verdict == "too_far":
            if self._reseed_attempts == 0:
                self.get_logger().error(
                    f"LOCALIZATION LOST: {travelled:.1f} m of odometry since "
                    "the last confident fix - too far to dead-reckon; staying "
                    "stopped")
            self._reseed_attempts = int(
                self.get_parameter("loc_reseed_max_attempts").value)
            return
        self._last_reseed = now
        self._reseed_attempts += 1
        # Uncertainty grows with how far odometry carried us since the
        # anchor; still well under the aisle pitch so the seed stays unimodal.
        sxy = min(0.6, 0.15 + 0.05 * travelled)
        syaw = min(0.6, 0.15 + 0.03 * travelled)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(t * 0.5)
        msg.pose.pose.orientation.w = math.cos(t * 0.5)
        cov = [0.0] * 36
        cov[0] = cov[7] = sxy * sxy
        cov[35] = syaw * syaw
        msg.pose.covariance = cov
        self._pub_initialpose.publish(msg)
        # Correct the seed against the scan at once instead of waiting for
        # the next periodic request (or for motion that a LOST robot lacks).
        self._last_nomotion_req = float("-inf")
        self._request_nomotion_update(now)
        self.get_logger().warning(
            f"LOCALIZATION LOST: re-seeding AMCL by odometry dead reckoning "
            f"(attempt {self._reseed_attempts}) at ({x:.2f}, {y:.2f}, "
            f"{math.degrees(t):.0f} deg), {travelled:.1f} m since last "
            f"confident fix, sigma {sxy:.2f} m / {syaw:.2f} rad")

    def _loc_health(self) -> int:
        """Broadcast form of the gate above: LOST, DEGRADED while the
        covariance is over the gate but not yet declared lost, else OK."""
        if self._loc_lost:
            return LOC_LOST
        if self._loc_bad_since is not None:
            return LOC_DEGRADED
        return LOC_OK

    def _loc_sigma_lat(self) -> float:
        """1-sigma AMCL position error perpendicular to my heading, metres.
        0.0 when there is no covariance (no AMCL pose yet, or odom poses)."""
        if self._amcl_cov is None:
            return 0.0
        cxx, cyy, _ = self._amcl_cov
        if self._amcl_cov_xy is None:
            return math.sqrt(max(0.0, cxx, cyy))
        # Variance along the lateral unit vector n = (-sin th, cos th).
        th = self.coord.state.pose.theta
        s, c = math.sin(th), math.cos(th)
        var = cxx * s * s - 2.0 * self._amcl_cov_xy * s * c + cyy * c * c
        if var < 0.0:
            return math.sqrt(max(0.0, cxx, cyy))
        return math.sqrt(var)

    def _request_nomotion_update(self, now: float) -> None:
        period = float(
            self.get_parameter("loc_nomotion_update_period_s").value)
        if (self._nomotion_cli is None or period <= 0.0
                or now - self._last_nomotion_req < period):
            return
        self._last_nomotion_req = now
        if self._nomotion_cli.service_is_ready():
            self._nomotion_cli.call_async(EmptySrv.Request())

    # ============================================================== helpers ==
    def _update_battery(self, now: float) -> None:
        rate = float(self.get_parameter("battery_per_metre_pct").value)
        travelled = self._distance_m - self._battery_distance_seen
        self._battery_distance_seen = self._distance_m
        pct = self.coord.state.battery_pct - rate * max(0.0, travelled)
        if (self._battery_t is not None and self.dock is not None
                and self.dock.phase == docking.DOCKED):
            charge = float(self.get_parameter("charge_rate_pct_per_s").value)
            pct += charge * max(0.0, now - self._battery_t)
        self._battery_t = now
        self.coord.state.battery_pct = min(100.0, max(0.0, pct))

    def _make_dock_policy(self) -> Optional[docking.DockPolicy]:
        hx = float(self.get_parameter("home_x").value)
        hy = float(self.get_parameter("home_y").value)
        if (not bool(self.get_parameter("return_home_enabled").value)
                or not (math.isfinite(hx) and math.isfinite(hy))):
            self.get_logger().info("return-to-dock disabled (no home_x/home_y)")
            return None
        home = self.grid.world_to_cell(hx, hy)
        if not self.grid.is_static_free(home):
            self.get_logger().error(
                f"home ({hx:.2f}, {hy:.2f}) -> cell {home} is not free; "
                "return-to-dock disabled")
            return None
        hyaw = float(self.get_parameter("home_yaw").value)
        yaw_tol = math.radians(float(self.get_parameter("dock_yaw_tol_deg").value))
        if not math.isfinite(hyaw):
            hyaw, yaw_tol = 0.0, math.pi     # no spawn yaw: any heading docks
        self.get_logger().info(
            f"dock at ({hx:.2f}, {hy:.2f}, yaw {math.degrees(hyaw):.0f} deg) "
            f"cell {home}: idle robots return here, align and charge")
        return docking.DockPolicy(
            home, home_pose=(hx, hy, hyaw),
            idle_s=float(self.get_parameter("return_home_idle_s").value),
            low_pct=float(self.get_parameter("battery_low_pct").value),
            resume_pct=float(self.get_parameter("battery_resume_pct").value),
            approach_m=float(self.get_parameter("dock_approach_m").value),
            pos_tol_m=float(self.get_parameter("dock_pos_tol_m").value),
            yaw_tol_rad=yaw_tol,
            align_timeout_s=float(
                self.get_parameter("dock_align_timeout_s").value),
            max_retries=int(self.get_parameter("dock_max_retries").value),
            backoff_m=float(self.get_parameter("dock_backoff_m").value))

    def _dock_final_pose(self):
        """Exact spawn pose the published path must end on, or None."""
        d = self.dock
        if (d is None or d.phase not in (docking.RETURNING, docking.ALIGNING)
                or self.coord.goal_cell != d.home_cell):
            return None
        return d.home_pose

    def _rotation_clear(self, now: float) -> bool:
        """No peer (alive, suspect or dead-and-frozen) within the swept circle
        of an in-place turn, widened by both robots' pose uncertainty."""
        me = self.coord.state
        reg = self.coord.registry
        for rid, st in reg.peers.items():
            pose = reg.extrapolated_pose(rid, now) or st.pose
            sigma = max(float(getattr(me, "loc_sigma_lat", 0.0) or 0.0),
                        float(getattr(st, "loc_sigma_lat", 0.0) or 0.0))
            if (math.hypot(pose.x - me.pose.x, pose.y - me.pose.y)
                    < docking.ROTATE_CLEAR_M + 2.0 * sigma):
                return False
        return True

    def _advance_dock(self, now: float) -> None:
        """No task: wait the idle grace, drive home, align on the spawn pose,
        then dock and charge."""
        if self.dock is None:
            return
        d = self.dock
        pose = self.coord.state.pose
        pos_err, yaw_err = d.errors(pose.x, pose.y, pose.theta)
        settled = (abs(self.coord.state.v) < 0.02
                   and abs(self.coord.state.w) < 0.05)
        before = d.phase
        action = d.step(
            now, has_task=self.auction.my_task is not None,
            # Dwells and both retreat kinds (this node's YIELD manoeuvre and
            # the coordinator's refuge recovery) own the goal until they end.
            transient=(self._dwell_until is not None
                       or self._retreat_until is not None
                       or getattr(self.coord, "_retreat", None) is not None
                       or self.coord.state.status == YIELDING),
            pos_err_m=pos_err, yaw_err_rad=yaw_err, settled=settled,
            has_goal=self.coord.goal_cell is not None,
            battery_pct=self.coord.state.battery_pct,
            rotation_clear=self._rotation_clear(now))
        if action == docking.GO_HOME:
            self.coord.set_goal(d.home_cell, now=now)
            self._publish_path()
            if before != d.phase or before is None:
                self.get_logger().info(
                    ("aligning on dock" if d.phase == docking.ALIGNING
                     else "no tasks left: returning to dock")
                    + (" (low battery)" if d.charge_hold else ""))
        elif action == docking.BACK_OFF:
            bx, by = d.backoff_pose()
            cell = self.grid.world_to_cell(bx, by)
            if not self.grid.is_static_free(cell):
                cell = d.home_cell              # nowhere to back into
            self.coord.set_goal(cell, now=now)
            self._publish_path()
            self.get_logger().warning(
                f"dock alignment timed out ({pos_err:.2f} m, "
                f"{math.degrees(yaw_err):.0f} deg off): backing off "
                f"{d.backoff_m:.1f} m to re-approach "
                f"(retry {d.retries}/{d.max_retries})")
        elif action == docking.DOCK:
            self.coord.clear_goal()
            self.coord.arbiter.release_all()
            self._publish_path()
            if d.misaligned:
                self.get_logger().warning(
                    f"docked MISALIGNED after {d.max_retries} retries "
                    f"({pos_err:.2f} m, {math.degrees(yaw_err):.1f} deg off); "
                    "charging anyway")
            else:
                self.get_logger().info(
                    f"docked on spawn pose ({pos_err:.3f} m, "
                    f"{math.degrees(yaw_err):.1f} deg), charging from "
                    f"{self.coord.state.battery_pct:.1f}%")

    def _sync_auction_inputs(self) -> None:
        self.auction.current_cell = self.grid.world_to_cell(
            self.coord.state.pose.x, self.coord.state.pose.y)
        self.auction.battery_pct = self.coord.state.battery_pct
        self.auction.nominal_speed = self.v_nominal
        # compute_bid refuses to bid while FAULT; a LOST robot must not win
        # work it cannot navigate to, but stays a live (stopped) fleet member.
        self.auction.state_status = (FAULT if self._loc_lost
                                     else self.coord.state.status)
        self.auction.charge_hold = bool(self.dock is not None
                                        and self.dock.charge_hold)
        # T1: a mid-dwell robot bids with its remaining load/unload time.
        now = self._now()
        self.auction.remaining_dwell_s = (
            max(0.0, self._dwell_until - now)
            if self._dwell_until is not None else 0.0)
        # R1 belt-and-braces: the coordinator's priority class must mirror
        # the node-owned phase even on paths outside the task callbacks
        # (e.g. _fault). _set_task_phase is the primary writer.
        self.coord.task_phase = (self._task_phase
                                 if self.auction.my_task is not None else None)

    def _set_task_phase(self, phase) -> None:
        """Single writer of the task phase. Mirrors it onto the coordinator:
        the R1 priority class C comes from THIS, never from goal_cell."""
        self._task_phase = phase
        self.coord.task_phase = phase
        if phase is None:
            self.coord.task_t0 = None

    @staticmethod
    def _class_of_score(score: float) -> int:
        """Priority class (0..4) from a broadcast L; tolerant of legacy
        [0, 1] scores (they map to class 0)."""
        fn = getattr(_core_priority, "class_of_score", None)
        if fn is not None:
            try:
                return int(fn(float(score)))
            except Exception:
                pass
        s = float(score)
        return max(0, min(4, int(s // 1000.0))) if s >= 1.0 else 0

    def _my_task_class(self) -> int:
        """My own class for claim-collision resolution (R4(4)): LOADED
        (carrying, phase DROPOFF) beats TO_PICKUP beats taskless."""
        if self.auction.my_task is None:
            return 0
        return 3 if self._task_phase == "DROPOFF" else 2

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _build_static_margin(self, margin: int) -> set:
        """Cells within `margin` cells of a wall/rack. Hits there are treated
        as the static structure itself, never as a new dynamic obstacle."""
        near = set()
        if margin <= 0:
            return near
        g = self.grid
        for r in range(g.height):
            row = g.static[r]
            for c in range(g.width):
                if row[c]:
                    for dr in range(-margin, margin + 1):
                        for dc in range(-margin, margin + 1):
                            near.add((r + dr, c + dc))
        return near

    def _pose_at_scan(self, scan: LaserScan, now: float) -> Optional[Pose2D]:
        """Sensor pose in the map frame at the scan timestamp, or None."""
        max_w = float(self.get_parameter("obstacle_max_angular_vel").value)
        if abs(self.coord.state.w) > max_w:
            return None          # fast rotation: projection is unreliable
        if self.pose_source != "tf" or self._tf_buffer is None:
            return self.coord.state.pose
        frame = scan.header.frame_id or self.base_frame
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, frame, Time.from_msg(scan.header.stamp))
        except TransformException:
            stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            max_age = float(self.get_parameter("obstacle_max_scan_age_s").value)
            if stamp > 0.0 and abs(now - stamp) <= max_age:
                return self.coord.state.pose
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return Pose2D(t.x, t.y, theta)

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
            blocked.update(self.coord._peer_obstacle_cells())
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
