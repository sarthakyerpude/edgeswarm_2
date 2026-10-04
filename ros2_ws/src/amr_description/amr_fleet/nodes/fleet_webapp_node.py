"""
fleet_webapp - live 2D warehouse view + click-to-create tasks in the browser.

Like task_generator, this node is an EVENT SOURCE, not a dispatcher. A task
created in the browser is announced on /fleet/task_announce and the robots'
auctioneer-free auction decides who does it. If every robot is busy, the
auction's no-bid retry keeps the order alive until one frees up - that IS the
queue; this node only displays it.

Transport is the Python standard library only (no rosbridge, no pip):
    GET  /                 the single-page app (share/amr_description/web)
    GET  /static/<file>    its JS/CSS
    GET  /api/map          static layout: occupancy, zones, stations, docks
    GET  /api/stream       Server-Sent Events, live fleet snapshot at ~10 Hz
                           (+ the P2P view and new feed events, p2p_monitor)
    POST /api/tasks        {pickup:{x,y}, drop:{x,y}, priority} -> Task
    POST /api/tasks/<id>/abort   -> TaskCancel (only before pickup)
"""
import json
import math
import os
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from amr_description.msg import (RobotState, Task as RosTask, TaskAward,
                                 TaskComplete, TaskRelease)
try:   # /fleet/task_cancel arrives with the fleet's task-rules increment
    from amr_description.msg import TaskCancel
except ImportError:  # pragma: no cover - message not built yet
    TaskCancel = None
from amr_fleet.core import astar
from amr_fleet.core.gridmap import GridMap
try:   # rack slot labels ('1R6'), added with the equal-aisle layout
    from amr_fleet.core.rack_slots import RackSlots
except ImportError:  # pragma: no cover - older source tree
    RackSlots = None
from amr_fleet.nodes.p2p_monitor import P2PMonitor
from amr_fleet.nodes.qos_profiles import (COORD_QOS, DIAGNOSTIC_QOS, PATH_QOS,
                                          STATE_QOS)

STATUS = ['IDLE', 'MOVING', 'WAITING', 'YIELDING', 'CHARGING', 'FAULT']
CONTENT_TYPES = {'.html': 'text/html; charset=utf-8',
                 '.js': 'text/javascript; charset=utf-8',
                 '.css': 'text/css; charset=utf-8',
                 '.svg': 'image/svg+xml', '.png': 'image/png',
                 '.ico': 'image/x-icon'}
MAX_DONE_TASKS = 200
FINISHED = ('DONE', 'ABORTED')
ABORT_SETTLE_S = 1.0     # robot state must show the task gone this long
ABORT_TIMEOUT_S = 10.0   # no robot let go of it: the fleet ignored the cancel


class TaskRejected(ValueError):
    """A browser request that must not reach the fleet bus (HTTP 422)."""
    code = 422


class TaskConflict(TaskRejected):
    """Valid request, wrong moment: e.g. abort after pickup (HTTP 409)."""
    code = 409


class TaskNotFound(TaskRejected):
    code = 404


class TaskUnavailable(TaskRejected):
    """The fleet does not support this yet (HTTP 503)."""
    code = 503


class FleetWebAppNode(Node):

    def __init__(self):
        super().__init__('fleet_webapp')
        share = get_package_share_directory('amr_description')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8080)
        self.declare_parameter('robot_ids', ['robot_1', 'robot_2', 'robot_3'])
        # Flat [x1, y1, x2, y2, ...] dock (= spawn) poses in robot_ids order,
        # so docks are drawn before any agent is up.
        self.declare_parameter('home_xy', [-3.0, -4.0, 0.0, -4.0, 3.0, -4.0])
        self.declare_parameter(
            'grid_yaml', os.path.join(share, 'config', 'warehouse_grid.yaml'))
        self.declare_parameter('web_root', os.path.join(share, 'web'))
        # A clicked point must sit where the 0.32 m chassis fits comfortably.
        self.declare_parameter('min_clearance_m', 0.30)
        self.declare_parameter('snap_radius_m', 0.5)
        # 5 Hz is smooth on the map (robots are interpolated) and halves the
        # snapshot/JSON cost per browser vs 10 Hz on a saturated host.
        self.declare_parameter('stream_hz', 5.0)
        self.declare_parameter('offline_after_s', 1.5)

        self.robot_ids = [str(r) for r in self.get_parameter('robot_ids').value]
        # abspath, not realpath: under --symlink-install every file in web/
        # is a symlink into src/, and containment is checked on the name.
        self.web_root = os.path.abspath(str(self.get_parameter('web_root').value))
        self.min_clearance = float(self.get_parameter('min_clearance_m').value)
        self.snap_radius = float(self.get_parameter('snap_radius_m').value)
        self.stream_period = 1.0 / max(
            1.0, float(self.get_parameter('stream_hz').value))
        self.offline_after = float(self.get_parameter('offline_after_s').value)

        self.grid = GridMap.from_yaml(str(self.get_parameter('grid_yaml').value))
        self._clearance = self.grid.clearance_m()
        self.grid.clearance_cost()     # warm A*'s cache before any request
        homes = list(self.get_parameter('home_xy').value)
        self.homes = {rid: (float(homes[2 * i]), float(homes[2 * i + 1]))
                      for i, rid in enumerate(self.robot_ids)
                      if 2 * i + 1 < len(homes)}
        self.slots = None
        if RackSlots is not None:
            try:
                self.slots = RackSlots.from_yaml(
                    str(self.get_parameter('grid_yaml').value))
            except (KeyError, ValueError) as exc:
                self.get_logger().warning(f'no rack slot labels: {exc}')
        self._map_json = json.dumps(self._map_payload()).encode('utf-8')

        # Per-run prefix: robots remember completed task ids, so a restarted
        # webapp must never reuse W-0001 or the fleet would silently drop it.
        self._id_prefix = f'W-{secrets.token_hex(2)}'
        self._task_seq = 0
        self._lock = threading.Lock()
        self.states = {}       # robot_id -> (RobotState, wall time)
        self.paths = {}        # robot_id -> [[x, y], ...]
        self.diags = {}        # robot_id -> dict from fleet_diagnostics
        self.tasks = {}        # task_id -> row dict
        self._running = True

        self.pub_task = self.create_publisher(
            RosTask, '/fleet/task_announce', COORD_QOS)
        self.create_subscription(RobotState, '/fleet/robot_state',
                                 self._on_state, STATE_QOS)
        self.create_subscription(RosTask, '/fleet/task_announce',
                                 self._on_announce, COORD_QOS)
        self.create_subscription(TaskAward, '/fleet/task_award',
                                 self._on_award, COORD_QOS)
        self.create_subscription(TaskComplete, '/fleet/task_complete',
                                 self._on_complete, COORD_QOS)
        self.create_subscription(TaskRelease, '/fleet/task_release',
                                 self._on_release, COORD_QOS)
        self.pub_cancel = None
        if TaskCancel is not None:
            self.pub_cancel = self.create_publisher(
                TaskCancel, '/fleet/task_cancel', COORD_QOS)
            self.create_subscription(TaskCancel, '/fleet/task_cancel',
                                     self._on_cancel, COORD_QOS)
        for rid in self.robot_ids:
            self.create_subscription(
                Path, f'/{rid}/coordination_path',
                lambda m, rid=rid: self._on_path(rid, m), PATH_QOS)
            self.create_subscription(
                String, f'/{rid}/fleet_diagnostics',
                lambda m, rid=rid: self._on_diag(rid, m), DIAGNOSTIC_QOS)
        self.p2p = P2PMonitor(self, self.grid, self.robot_ids)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self._server = ThreadingHTTPServer((host, port), self._handler_class())
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'fleet webapp at http://{host}:{port} '
            f'(robots={self.robot_ids}, task ids {self._id_prefix}-NNN)')

    # ========================================================== static map ==
    def _map_payload(self):
        g = self.grid
        rows = []
        for r in range(g.height):
            line = []
            for c in range(g.width):
                if g.static[r][c]:
                    line.append('#')
                elif self._clearance[r][c] < self.min_clearance:
                    line.append(':')      # free, but too tight for a marker
                else:
                    line.append('.')
            rows.append(''.join(line))
        zones = []
        for zid, z in g.zones.items():
            if not z.cells:
                continue
            rs = [cell[0] for cell in z.cells]
            cs = [cell[1] for cell in z.cells]
            zones.append({'id': zid, 'kind': z.kind,
                          'rect': [min(rs), min(cs), max(rs), max(cs)]})
        # The yaml lists each marker twice (L1 / PICKUP_1); keep the short
        # name. CHG* are legacy - the docks are now the robots' spawn poses.
        stations = {}
        for sid, cell in sorted(g.stations.items(), key=lambda kv: len(kv[0])):
            kind = ('pickup' if sid.startswith(('L', 'PICKUP')) else
                    'drop' if sid.startswith(('P', 'DROPOFF')) else None)
            if kind is None or (cell, kind) in stations:
                continue
            x, y = g.cell_to_world(cell)
            stations[(cell, kind)] = {'id': sid, 'kind': kind,
                                      'cell': list(cell), 'x': x, 'y': y}
        return {
            'width': g.width, 'height': g.height,
            'resolution': g.resolution, 'origin': list(g.origin),
            'occupancy': rows, 'zones': zones,
            'stations': list(stations.values()),
            'docks': [{'robot': rid, 'x': x, 'y': y}
                      for rid, (x, y) in self.homes.items()],
            'robots': self.robot_ids,
            'min_clearance_m': self.min_clearance,
            'traffic': self._traffic_layout(),
            'aisle_width_m': (self.slots.aisle_width_m if self.slots else 1.6),
            'racks': ([{'id': r.rack_id, 'x_min': r.x_min, 'x_max': r.x_max,
                        'y_min': r.y_min, 'y_max': r.y_max}
                       for r in sorted(self.slots.racks.values())]
                      if self.slots else []),
            'slots': ([{'label': sl.label, 'side': sl.side, 'rack': sl.rack,
                        'face': sl.face, 'section': sl.section,
                        'x0': round(sl.x0, 4), 'x1': round(sl.x1, 4),
                        'face_y': round(sl.face_y, 4),
                        'approach': [round(sl.approach[0], 4),
                                     round(sl.approach[1], 4)]}
                       for sl in self.slots.slots()] if self.slots else []),
        }

    def _traffic_layout(self):
        """Two-way road model from warehouse_grid.yaml (P2P session schema):
        traffic.road_lanes [{id, direction NB|SB|EB|WB, polyline [[x,y],..]}] and
        traffic.junctions [{id, center [x,y], radius}] and the lane-crossing
        zones traffic.turnarounds [{id, center, half_size}]. Empty when absent."""
        import yaml
        try:
            with open(str(self.get_parameter('grid_yaml').value)) as f:
                t = (yaml.safe_load(f) or {}).get('traffic') or {}
        except (OSError, yaml.YAMLError):
            return {'lanes': [], 'junctions': [], 'turnarounds': []}
        lanes, junctions, turnarounds = [], [], []
        # 'road_lanes' (list); 'lanes' is the planner's internal band dict.
        for ln in t.get('road_lanes') or []:
            try:
                pts = [[float(x), float(y)] for x, y in ln['polyline']]
            except (KeyError, TypeError, ValueError):
                continue
            if len(pts) >= 2:
                lanes.append({'id': str(ln.get('id', '')), 'polyline': pts,
                              'direction': str(ln.get('direction', ''))})
        for j in t.get('junctions') or []:
            # Box junctions {center, half_size:[w,h]} (current schema); the
            # older {center, radius} becomes a square of that half-size.
            try:
                if j.get('half_size') is not None:
                    hw, hh = (float(j['half_size'][0]), float(j['half_size'][1]))
                else:
                    hw = hh = float(j['radius'])
                junctions.append({'id': str(j.get('id', '')),
                                  'center': [float(j['center'][0]),
                                             float(j['center'][1])],
                                  'half_size': [hw, hh]})
            except (KeyError, TypeError, ValueError, IndexError):
                continue
        # Strict-lane crossing zones: the only places (besides junction boxes)
        # where a robot may cross into the opposite lane - aisle dead-end
        # turnarounds, the south-throat apron, spur-mouth aprons.
        for z in t.get('turnarounds') or []:
            try:
                turnarounds.append({
                    'id': str(z.get('id', '')),
                    'center': [float(z['center'][0]), float(z['center'][1])],
                    'half_size': [float(z['half_size'][0]),
                                  float(z['half_size'][1])]})
            except (KeyError, TypeError, ValueError, IndexError):
                continue
        return {'lanes': lanes, 'junctions': junctions,
                'turnarounds': turnarounds}

    # ======================================================= ROS callbacks ==
    def _on_state(self, m):
        self.p2p.observe('/fleet/robot_state', m, m.robot_id)
        with self._lock:
            self.states[m.robot_id] = (m, time.time())

    def _on_path(self, rid, m):
        pts = [[p.pose.position.x, p.pose.position.y] for p in m.poses]
        if len(pts) > 2:              # 0.1 m cells: every 3rd point is plenty
            pts = pts[:-1:3] + [pts[-1]]
        with self._lock:
            self.paths[rid] = pts

    def _on_diag(self, rid, m):
        self.p2p.observe(f'/{rid}/fleet_diagnostics', m, rid)
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self._lock:
            self.diags[rid] = d
        self.p2p.on_diag(rid, d)

    def _row(self, tid):
        row = self.tasks.get(tid)
        if row is None:
            row = {'id': tid, 'source': 'web' if tid.startswith('W-') else 'gen',
                   'pickup': None, 'drop': None, 'priority': 0,
                   'status': 'PENDING', 'robot': None,
                   'created': time.time(), 'updated': time.time()}
            self.tasks[tid] = row
        return row

    def _set_cells(self, row, m):
        p = (int(m.pickup_row), int(m.pickup_col))
        d = (int(m.dropoff_row), int(m.dropoff_col))
        row['pickup'] = list(self.grid.cell_to_world(p))
        row['drop'] = list(self.grid.cell_to_world(d))
        row['pickup_slot'] = self._slot_label(*row['pickup'])
        row['drop_slot'] = self._slot_label(*row['drop'])
        row['priority'] = int(m.priority)
        # Absolute, on the announcer's ROS clock (sim time in simulation;
        # this node runs on the same clock). 0 = no deadline.
        row['deadline'] = float(getattr(m, 'deadline', 0.0) or 0.0)

    def _on_announce(self, m):
        self.p2p.observe('/fleet/task_announce', m, m.announcer_id)
        self.p2p.event('task', m.announcer_id or '?', '*',
                       f'announces {m.task_id} (priority {m.priority})')
        with self._lock:
            row = self._row(m.task_id)
            if row['status'] in FINISHED:
                return                     # stale TRANSIENT_LOCAL replay
            self._set_cells(row, m)
            if row['status'] == 'ASSIGNED' and m.announcer_id in self.robot_ids:
                # A survivor re-announced a dead robot's task.
                row['status'], row['robot'] = 'PENDING', None
            row['updated'] = time.time()

    def _on_award(self, m):
        self.p2p.observe('/fleet/task_award', m, m.winner_id)
        self.p2p.event('task', m.winner_id, '*',
                       f'wins {m.task_id} (bid {m.winning_bid:.4f}, '
                       f'{m.num_bidders} bidders)')
        with self._lock:
            row = self._row(m.task_id)
            if row['status'] == 'ABORTED':
                # A robot took an aborted task after all: show the truth.
                self.p2p.event('task', m.winner_id, '*',
                               f'abort of {m.task_id} not honoured by the fleet')
            if row['status'] != 'DONE':
                row['status'], row['robot'] = 'ASSIGNED', m.winner_id
                row['updated'] = time.time()

    def _on_complete(self, m):
        self.p2p.observe('/fleet/task_complete', m, m.robot_id)
        self.p2p.event('task', m.robot_id, '*', f'completed {m.task_id}')
        with self._lock:
            row = self._row(m.task_id)
            row['status'], row['robot'] = 'DONE', m.robot_id
            row['updated'] = time.time()
            done = sorted((r for r in self.tasks.values()
                           if r['status'] in FINISHED),
                          key=lambda r: r['updated'])
            for r in done[:-MAX_DONE_TASKS]:
                self.tasks.pop(r['id'], None)

    def _on_release(self, m):
        self.p2p.observe('/fleet/task_release', m, m.robot_id)
        self.p2p.event('task', m.robot_id, '*',
                       f'releases {m.task_id}: {m.reason}')
        with self._lock:
            row = self._row(m.task_id)
            if row['status'] not in FINISHED:
                self._set_cells(row, m)
                row['status'], row['robot'] = 'PENDING', None
                row['updated'] = time.time()

    def _on_cancel(self, m):
        self.p2p.observe('/fleet/task_cancel', m, m.requester_id)
        self.p2p.event('task', m.requester_id or '?', '*',
                       f'cancels {m.task_id}' + (f': {m.reason}' if m.reason else ''))
        with self._lock:
            row = self.tasks.get(m.task_id)
            if row is not None and row['status'] not in FINISHED + ('ABORTING',):
                row['status'] = 'ABORTING'      # e.g. another browser asked
                row['abort_t'] = time.time()

    def _settle_aborts(self, now, phase):
        """ABORTING -> ABORTED only on POSITIVE evidence: the robot that held
        the task has reported, after the abort, without it. Silence (a stale
        or offline robot) proves nothing, so the row just stays ABORTING.
        Back to its live state if the holder had already loaded, or kept
        reporting the task well past the abort."""
        for row in self.tasks.values():
            if row['status'] != 'ABORTING':
                continue
            abort_t = row.get('abort_t', now)
            waited = now - abort_t
            holder, holder_fresh = None, False
            for rid, (s, seen) in self.states.items():
                if s.task_id == row['id']:      # last-known report, any age
                    holder = rid
                    holder_fresh = now - seen <= self.offline_after
            if holder is None:
                prev = row.get('robot')
                seen = self.states[prev][1] if prev in self.states else None
                released = prev is None or (seen is not None and seen > abort_t
                                            and now - seen <= self.offline_after)
                if released and waited >= ABORT_SETTLE_S:
                    row['status'], row['updated'] = 'ABORTED', now
                    self.p2p.event('task', 'webapp', '*', f'{row["id"]} aborted')
            elif not holder_fresh:
                continue                        # no news from the holder yet
            elif phase.get(holder) == 'DROPOFF':
                row['status'], row['robot'] = 'ASSIGNED', holder
                self.p2p.event('task', holder, '-',
                               f'refused abort of {row["id"]}: already picked up')
            elif waited > ABORT_TIMEOUT_S:
                row['status'], row['robot'] = 'ASSIGNED', holder
                self.p2p.event('task', holder, '-',
                               f'kept {row["id"]}: no response to the abort')

    # ============================================================ snapshot ==
    def snapshot(self):
        now = time.time()
        ros_now = self.get_clock().now().nanoseconds * 1e-9   # deadlines' clock
        with self._lock:
            robots = []
            for rid in sorted(set(self.robot_ids) | set(self.states)):
                entry = self.states.get(rid)
                diag = self.diags.get(rid, {})
                home = self.homes.get(rid)
                if entry is None:
                    robots.append({'id': rid, 'known': False,
                                   'status': 'OFFLINE', 'home': home})
                    continue
                s, seen = entry
                age = now - seen
                idx = int(s.status)
                status = STATUS[idx] if 0 <= idx < len(STATUS) else 'UNKNOWN'
                if age > self.offline_after:
                    status = 'OFFLINE'
                robots.append({
                    'id': rid, 'known': True, 'status': status,
                    'x': s.pose.x, 'y': s.pose.y, 'theta': s.pose.theta,
                    'v': s.velocity.linear.x,
                    'w': s.velocity.angular.z,
                    'battery': float(s.battery_pct),
                    'task_id': s.task_id or None, 'alive': bool(s.alive),
                    'age_ms': round(age * 1000.0),
                    'task_phase': diag.get('task_phase'),
                    'dock_phase': diag.get('dock_phase'),
                    'charge_hold': bool(diag.get('charge_hold', False)),
                    'dock_err_m': diag.get('dock_err_m'),
                    'dock_err_deg': diag.get('dock_err_deg'),
                    'dock_misaligned': bool(diag.get('dock_misaligned', False)),
                    'dock_retries': diag.get('dock_retries', 0),
                    'dock_hold': bool(diag.get('dock_hold', False)),
                    'disc_mode': diag.get('hitbox_disc_mode'),
                    'reloc_attempts': int(diag.get('reloc_attempts') or 0),
                    'uturn_active': bool(diag.get('uturn_active', False)),
                    'reverse_active': bool(diag.get('reverse_active', False)),
                    'overtake_active': bool(diag.get('overtake_active', False)),
                    'home': home,
                    # For the hitbox layer: the robot's own pose uncertainty.
                    'sigma': float(getattr(s, 'loc_sigma_lat', 0.0) or 0.0),
                    'loc_health': int(getattr(s, 'loc_health', 0) or 0),
                })
            phase = {r['id']: r.get('task_phase') for r in robots}
            # RobotState.task_id is ground truth for ownership; it also heals
            # out-of-order TRANSIENT_LOCAL replays when the webapp starts late.
            owner_of = {s.task_id: rid for rid, (s, seen) in self.states.items()
                        if s.task_id and now - seen <= self.offline_after}
            self._settle_aborts(now, phase)
            tasks = []
            for row in self.tasks.values():
                t = dict(row)
                if t['status'] in ('PENDING', 'ASSIGNED') and t['id'] in owner_of:
                    t['status'], t['robot'] = 'ASSIGNED', owner_of[t['id']]
                if t['status'] == 'ASSIGNED':
                    rp = phase.get(t['robot'])
                    owner = self.states.get(t['robot'])
                    if rp and owner and owner[0].task_id == t['id']:
                        t['status'] = 'TO_PICKUP' if rp == 'PICKUP' else 'TO_DROP'
                t['age_s'] = round(now - t['created'], 1)
                dl = t.get('deadline') or 0.0
                t['deadline_left_s'] = (round(dl - ros_now, 1)
                                        if dl > 0.0 and t['status'] not in FINISHED
                                        else None)
                t.pop('abort_t', None)
                tasks.append(t)
            tasks.sort(key=lambda t: (t['status'] in FINISHED, -t['updated']
                                      if t['status'] in FINISHED else t['created']))
            paths = {rid: pts for rid, pts in self.paths.items() if pts}
        return {'t': now, 'robots': robots, 'paths': paths, 'tasks': tasks}

    # ========================================================= create task ==
    def _snap(self, x, y, label):
        """Nearest cell with enough clearance within snap_radius of (x, y)."""
        g = self.grid
        if not (math.isfinite(x) and math.isfinite(y)):
            raise TaskRejected(f'{label}: coordinates must be numbers')
        r0, c0 = g.world_to_cell(x, y)
        span = int(math.ceil(self.snap_radius / g.resolution))
        best, best_d = None, float('inf')
        for r in range(r0 - span, r0 + span + 1):
            for c in range(c0 - span, c0 + span + 1):
                if not g.is_static_free((r, c)):
                    continue
                if self._clearance[r][c] < self.min_clearance:
                    continue
                cx, cy = g.cell_to_world((r, c))
                d = math.hypot(cx - x, cy - y)
                if d <= self.snap_radius and d < best_d:
                    best, best_d = (r, c), d
        if best is None:
            raise TaskRejected(
                f'{label} ({x:.2f}, {y:.2f}) is inside or too close to a '
                'rack/wall - pick a point in an aisle')
        return best

    def _endpoint(self, point, label):
        """{x, y} -> nearest good cell; {slot: '1R6'} -> that slot's approach
        cell (the cell the robots use for it). Returns (cell, slot label)."""
        if not isinstance(point, dict):
            raise TaskRejected(f'{label}: expected {{x, y}} or {{slot}}')
        if point.get('slot'):
            if self.slots is None:
                raise TaskRejected('rack slot labels are not available')
            try:
                sl = self.slots.slot(str(point['slot']))
            except ValueError as exc:
                raise TaskRejected(f'{label}: {exc}')
            if self.grid.is_static_free(sl.approach_cell):
                return sl.approach_cell, sl.label
            return self._snap(sl.approach[0], sl.approach[1], label), sl.label
        try:
            x, y = float(point['x']), float(point['y'])
        except (KeyError, TypeError, ValueError):
            raise TaskRejected(f'{label}: expected {{x, y}} or {{slot}}')
        cell = self._snap(x, y, label)
        return cell, self._slot_label(*self.grid.cell_to_world(cell))

    def _slot_label(self, x, y):
        """Slot whose approach point is within one cell of (x, y), if any."""
        if self.slots is None:
            return None
        return self.slots.label_at(x, y, max_dist_m=0.15)

    def create_task(self, body):
        try:
            priority = int(body.get('priority', 1))
            pick_pt, drop_pt = body['pickup'], body['drop']
        except (KeyError, TypeError, ValueError, AttributeError):
            raise TaskRejected('expected {pickup:{x,y}|{slot}, drop:{x,y}|{slot}, '
                               'priority}')
        if not 0 <= priority <= 3:
            raise TaskRejected('priority must be 0..3')
        pickup, pick_label = self._endpoint(pick_pt, 'pickup')
        drop, drop_label = self._endpoint(drop_pt, 'drop')
        if self.grid.cell_distance_m(pickup, drop) < 0.5:
            raise TaskRejected('pickup and drop must be at least 0.5 m apart')
        if not astar.astar(self.grid, pickup, drop):
            raise TaskRejected('no route between pickup and drop')

        with self._lock:
            self._task_seq += 1
            tid = f'{self._id_prefix}-{self._task_seq:03d}'
            busy = [r['id'] for r in self.tasks.values()
                    if r['status'] not in FINISHED and r['pickup'] is not None
                    and self.grid.world_to_cell(*r['pickup']) == pickup]

        m = RosTask()
        m.header.stamp = self.get_clock().now().to_msg()
        m.task_id = tid
        m.pickup_row, m.pickup_col = pickup
        m.dropoff_row, m.dropoff_col = drop
        m.priority = priority
        m.created_at = self.get_clock().now().nanoseconds * 1e-9
        m.deadline = 0.0
        m.announcer_id = 'webapp'
        with self._lock:
            row = self._row(tid)
            self._set_cells(row, m)
        self.pub_task.publish(m)
        self.get_logger().info(
            f'announced {tid} pickup={pickup} drop={drop} prio={priority}')
        return {'task_id': tid,
                'pickup': list(self.grid.cell_to_world(pickup)),
                'drop': list(self.grid.cell_to_world(drop)),
                'pickup_slot': pick_label, 'drop_slot': drop_label,
                'warning': (f'pickup already used by {", ".join(busy)}; '
                            'robots may meet head-on there' if busy else None)}

    def abort_task(self, tid):
        """Ask the fleet to drop a task that has not been picked up yet."""
        now = time.time()
        with self._lock:
            row = self.tasks.get(tid)
            if row is None:
                raise TaskNotFound(f'unknown task {tid}')
            if row['status'] in FINISHED:
                raise TaskConflict(f'{tid} is already {row["status"].lower()}')
            if row['status'] == 'ABORTING':
                raise TaskConflict(f'{tid} is already being aborted')
            holder = next((rid for rid, (s, seen) in self.states.items()
                           if s.task_id == tid and now - seen <= self.offline_after),
                          row['robot'])
            if holder and self.diags.get(holder, {}).get('task_phase') == 'DROPOFF':
                raise TaskConflict(f'{tid} was already picked up by {holder}; '
                                   'it can only be aborted before pickup')
            if self.pub_cancel is None:
                raise TaskUnavailable(
                    'the fleet cannot cancel tasks yet: the TaskCancel message '
                    'arrives with the next amr_description build')
            row['status'], row['abort_t'] = 'ABORTING', now
        m = TaskCancel()
        m.header.stamp = self.get_clock().now().to_msg()
        m.task_id = tid
        m.requester_id = 'webapp'
        m.reason = 'aborted from the web app before pickup'
        self.pub_cancel.publish(m)
        self.get_logger().info(f'cancel requested for {tid} (holder={holder})')
        return {'task_id': tid, 'status': 'ABORTING', 'robot': holder}

    # ================================================================ HTTP ==
    def _handler_class(node):
        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def _send(self, code, body, content_type):
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _json(self, code, obj):
                self._send(code, json.dumps(obj).encode('utf-8'),
                           'application/json; charset=utf-8')

            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if path == '/api/map':
                    self._send(200, node._map_json,
                               'application/json; charset=utf-8')
                elif path == '/api/stream':
                    self._stream()
                elif path == '/api/state':
                    snap = node.snapshot()
                    snap['p2p'], snap['events'] = node.p2p.snapshot()
                    self._json(200, snap)
                elif path in ('/', '/index.html'):
                    self._file('index.html')
                elif path.startswith('/static/'):
                    self._file(path[len('/static/'):])
                else:
                    self._json(404, {'error': 'not found'})

            def do_POST(self):
                path = self.path.split('?', 1)[0]
                parts = path.strip('/').split('/')
                try:
                    length = min(int(self.headers.get('Content-Length', 0)), 65536)
                    raw = self.rfile.read(length) if length else b''
                    if path == '/api/tasks':
                        self._json(200, node.create_task(json.loads(raw or b'{}')))
                    elif (len(parts) == 4 and parts[:2] == ['api', 'tasks']
                          and parts[3] == 'abort'):
                        self._json(200, node.abort_task(unquote(parts[2])))
                    else:
                        self._json(404, {'error': 'not found'})
                except TaskRejected as exc:
                    self._json(exc.code, {'error': str(exc)})
                except ValueError:
                    self._json(400, {'error': 'body must be JSON'})

            def _file(self, name):
                full = os.path.normpath(os.path.join(node.web_root, name))
                if (not full.startswith(node.web_root + os.sep)
                        or not os.path.isfile(full)):
                    self._json(404, {'error': 'not found'})
                    return
                with open(full, 'rb') as f:
                    body = f.read()
                ext = os.path.splitext(full)[1].lower()
                self._send(200, body,
                           CONTENT_TYPES.get(ext, 'application/octet-stream'))

            def _stream(self):
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                self.close_connection = True
                since = None              # None = send the feed backlog first
                try:
                    while node._running:
                        snap = node.snapshot()
                        snap['p2p'], events = node.p2p.snapshot(since)
                        snap['events'] = events
                        if events:
                            since = events[-1]['id']
                        elif since is None:
                            since = 0
                        data = json.dumps(snap)
                        self.wfile.write(f'data: {data}\n\n'.encode('utf-8'))
                        self.wfile.flush()
                        time.sleep(node.stream_period)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass               # browser tab closed

            def log_message(self, _format, *_args):
                return

        return Handler

    def destroy_node(self):
        self._running = False
        self._server.shutdown()
        self._server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FleetWebAppNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
