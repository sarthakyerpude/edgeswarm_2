"""
p2p_monitor - what the robots tell each other, condensed for the web app.

READ-ONLY. It never publishes on the fleet bus; it listens to the shared
/fleet/* topics (whatever carries them: CycloneDDS, rmw_zenoh_cpp or a
zenoh-bridge-ros2dds) plus each robot's local permit, conflicts and
diagnostics, and keeps:

    links    each robot's OWN view of each peer (freshness, last-heard age,
             held intent seq, loss) - from that robot's fleet_diagnostics
    intents  the path cells each robot broadcast, with ETA per cell
    zones    who holds / is acquiring each mutex zone (Ricart-Agrawala)
    permits  each robot's current motion permit and who blocks it
    blocked  shared blockage cells from /fleet/map_update, with expiry
    events   a rolling feed of coordination messages (bids, zone requests
             and grants, new plans, blockages, conflicts, permit changes)
    stats    per-topic and per-robot message rate and bandwidth

The webapp hears each broadcast once, so rates are what a robot TRANSMITS;
with unicast fan-out (Zenoh peer mode, DDS without multicast) the on-air
volume is that times (N - 1).
"""
import collections
import os
import threading
import time

import rclpy
from rclpy.serialization import serialize_message

from amr_description.msg import (Conflict, Intent, MapUpdate, MotionPermit,
                                 TaskBid, ZoneGrant, ZoneRequest)
from amr_fleet.nodes.qos_profiles import (COORD_QOS, DIAGNOSTIC_QOS, MAP_QOS,
                                          PERMIT_QOS)

ACTIONS = ['GO', 'SLOW', 'STOP', 'YIELD', 'REROUTE']
HOLD_ACTIONS = ('STOP', 'YIELD', 'REROUTE')
CONFLICT_KINDS = ['CELL', 'SWAP', 'ZONE', 'TTC']
STATS_WINDOW_S = 5.0
MAX_EVENTS = 400
FIRST_EVENTS = 120          # backlog sent to a newly connected browser
CONFLICT_REPEAT_S = 5.0     # one feed line per (robot, peer, kind) per this
PERMIT_REPEAT_S = 1.0       # one permit-change line per robot per this
DEFAULT_BLOCK_S = 20.0
SIZE_SAMPLE_EVERY = 10     # weigh 1 message in N per topic (see observe)


def _stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class P2PMonitor:
    def __init__(self, node, grid, robot_ids):
        self.grid = grid
        self.robot_ids = list(robot_ids)
        self._lock = threading.Lock()
        self._events = collections.deque(maxlen=MAX_EVENTS)
        self._event_id = 0
        self._traffic = collections.deque()      # (t, topic, sender, bytes)
        self._started = time.time()
        self._stats, self._stats_t = {}, 0.0
        self._seen, self._size = {}, {}     # per-topic count / sampled size
        self.intents = {}       # rid -> broadcast plan
        self.permits = {}       # rid -> latest permit
        self.views = {}         # rid -> {peers, zones_held, zones_requesting, rx}
        self.blocked = {}       # (r, c) -> (wall deadline, reporter)
        self._permit_key = {}   # rid -> (key, wall time of last event)
        self._conflict_seen = {}
        self.transport = {
            'rmw': rclpy.get_rmw_implementation_identifier(),
            'zenoh_session_config': os.environ.get('ZENOH_SESSION_CONFIG_URI'),
            'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        }

        sub = node.create_subscription
        sub(Intent, '/fleet/intent', self._on_intent, COORD_QOS)
        sub(ZoneRequest, '/fleet/zone_request', self._on_zone_request, COORD_QOS)
        sub(ZoneGrant, '/fleet/zone_grant', self._on_zone_grant, COORD_QOS)
        sub(MapUpdate, '/fleet/map_update', self._on_map_update, MAP_QOS)
        sub(TaskBid, '/fleet/task_bid', self._on_bid, COORD_QOS)
        for rid in self.robot_ids:
            sub(MotionPermit, f'/{rid}/motion_permit',
                lambda m, rid=rid: self._on_permit(rid, m), PERMIT_QOS)
            sub(Conflict, f'/{rid}/conflicts', self._on_conflict, DIAGNOSTIC_QOS)

    # ============================================================ ingestion ==
    def observe(self, topic, msg, sender):
        """Count one received message for the traffic stats.

        Serializing every message just to weigh it cost the webapp ~1/4 of a
        core on a saturated host (robot_state alone is 30 msg/s). Weigh one
        in SIZE_SAMPLE_EVERY per topic and reuse that size for the rest:
        message sizes per topic are near-constant."""
        n = self._seen.get(topic, 0)
        self._seen[topic] = n + 1
        size = self._size.get(topic)
        if size is None or n % SIZE_SAMPLE_EVERY == 0:
            try:
                size = len(serialize_message(msg))
            except Exception:
                size = size or 0
            self._size[topic] = size
        with self._lock:
            self._traffic.append((time.time(), topic, sender, size))

    def event(self, kind, src, dst, text):
        """Append a feed line. dst '*' = broadcast to every peer."""
        with self._lock:
            self._event_id += 1
            self._events.append({'id': self._event_id, 't': time.time(),
                                 'kind': kind, 'src': src, 'dst': dst,
                                 'text': text})

    def on_diag(self, rid, d):
        """A robot's own view: peer freshness and its zone locks."""
        peers = d.get('peers') or {}
        held = list(d.get('zones_held') or [])
        req = dict(d.get('zones_requesting') or {})
        with self._lock:
            prev = self.views.get(rid)
            self.views[rid] = {'peers': peers, 'zones_held': held,
                               'zones_requesting': req, 'rx': time.time()}
        if prev is None:
            return
        for peer, view in peers.items():
            old = (prev['peers'].get(peer) or {}).get('fresh')
            new = view.get('fresh')
            # FRESH<->SUSPECT flaps with every heartbeat hiccup (the map
            # links show it); the feed only records losing or regaining a peer.
            if old and new and old != new and 'DEAD' in (old, new):
                self.event('peer', rid, peer, f'now sees {peer} as {new} '
                           f'(last heard {view.get("age_ms", 0)} ms ago)')
        for z in sorted(set(held) - set(prev['zones_held'])):
            self.event('zone', rid, '*', f'holds {z}')
        for z in sorted(set(prev['zones_held']) - set(held)):
            self.event('zone', rid, '*', f'released {z}')

    def _on_intent(self, m):
        self.observe('/fleet/intent', m, m.robot_id)
        stamp = _stamp(m.header)
        cells = []
        for i, (r, c) in enumerate(zip(m.path_rows, m.path_cols)):
            x, y = self.grid.cell_to_world((int(r), int(c)))
            eta = None
            if stamp > 0.0 and i < len(m.t_enter):
                eta = round(max(0.0, float(m.t_enter[i]) - stamp), 1)
            cells.append([round(x, 2), round(y, 2), eta])
        zones = [str(z) for z in m.zones_needed]
        with self._lock:
            prev = self.intents.get(m.robot_id)
            self.intents[m.robot_id] = {
                'seq': int(m.seq), 'cells': cells, 'zones': zones,
                'task_id': m.task_id or None,
                'goal': [round(m.goal.x, 2), round(m.goal.y, 2)],
                'rx': time.time()}
        if prev is None or prev['seq'] != int(m.seq):
            self.event('intent', m.robot_id, '*',
                       f'shares plan #{m.seq}: {len(cells)} cells'
                       + (f', needs {", ".join(zones)}' if zones else ''))

    def _on_zone_request(self, m):
        self.observe('/fleet/zone_request', m, m.robot_id)
        self.event('zone', m.robot_id, '*',
                   f'requests {m.zone_id} (score {m.priority_score:.2f}, '
                   f'clock {m.lamport_ts})')

    def _on_zone_grant(self, m):
        self.observe('/fleet/zone_grant', m, m.granter_id)
        self.event('zone', m.granter_id, m.requester_id,
                   f'{"grants" if m.granted else "denies"} {m.zone_id}')

    def _on_bid(self, m):
        self.observe('/fleet/task_bid', m, m.robot_id)
        self.event('task', m.robot_id, '*',
                   f'bids {m.bid:.4f} for {m.task_id} '
                   f'(ETA {m.est_completion_s:.0f}s, '
                   f'{m.battery_after_pct:.0f}% after)')

    def _on_map_update(self, m):
        self.observe('/fleet/map_update', m, m.reporter_id)
        now = time.time()
        stamp = _stamp(m.header)
        ttl = (float(m.expiry) - stamp) if stamp > 0.0 else DEFAULT_BLOCK_S
        ttl = min(120.0, max(0.0, ttl))
        blocked = list(zip(m.blocked_rows, m.blocked_cols))
        cleared = list(zip(m.cleared_rows, m.cleared_cols))
        with self._lock:
            for cell in blocked:
                self.blocked[(int(cell[0]), int(cell[1]))] = (now + ttl, m.reporter_id)
            for cell in cleared:
                self.blocked.pop((int(cell[0]), int(cell[1])), None)
        if blocked:
            self.event('map', m.reporter_id, '*',
                       f'reports {len(blocked)} blocked cell(s), '
                       f'confidence {m.confidence:.1f}, {ttl:.0f}s lease')
        if cleared:
            self.event('map', m.reporter_id, '*',
                       f'clears {len(cleared)} cell(s)')

    def _on_permit(self, rid, m):
        self.observe(f'/{rid}/motion_permit', m, rid)
        idx = int(m.action)
        action = ACTIONS[idx] if 0 <= idx < len(ACTIONS) else str(idx)
        permit = {'action': action, 'scale': round(float(m.speed_scale), 2),
                  'reason': m.reason, 'blocking': m.blocking_robot or None,
                  'zone': m.zone_id or None,
                  'deadlock': bool(m.deadlock_detected),
                  'cycle': list(m.deadlock_cycle), 'rx': time.time()}
        key = (action if action in HOLD_ACTIONS else 'GO',
               permit['blocking'], permit['zone'], permit['deadlock'])
        now = permit['rx']
        with self._lock:
            self.permits[rid] = permit
            last = self._permit_key.get(rid)
            changed = last is None or (last[0] != key and
                                       now - last[1] >= PERMIT_REPEAT_S)
            if changed:
                self._permit_key[rid] = (key, now)
        if changed and last is not None:
            text = f'{action}'
            if permit['blocking']:
                text += f' for {permit["blocking"]}'
            if permit['reason']:
                text += f': {permit["reason"]}'
            if permit['deadlock']:
                text += f' [deadlock {" -> ".join(permit["cycle"])}]'
            self.event('permit', rid, permit['blocking'] or '-', text)

    def _on_conflict(self, m):
        self.observe(f'/{m.robot_id}/conflicts', m, m.robot_id)
        idx = int(m.kind)
        kind = CONFLICT_KINDS[idx] if 0 <= idx < len(CONFLICT_KINDS) else str(idx)
        key = (m.robot_id, m.peer_id, kind)
        now = time.time()
        with self._lock:
            if now - self._conflict_seen.get(key, 0.0) < CONFLICT_REPEAT_S:
                return
            self._conflict_seen[key] = now
        where = f' in {m.zone_id}' if m.zone_id else (
            f' at cell ({m.cell_row},{m.cell_col})' if m.cell_row >= 0 else '')
        self.event('conflict', m.robot_id, m.peer_id,
                   f'{kind} conflict with {m.peer_id}{where}; '
                   f'{"has" if m.i_have_priority else "yields"} priority '
                   f'({m.my_priority:.2f} vs {m.peer_priority:.2f})')

    # ============================================================= snapshot ==
    def _stats_locked(self, now):
        if now - self._stats_t < 1.0:
            return self._stats
        cutoff = now - STATS_WINDOW_S
        while self._traffic and self._traffic[0][0] < cutoff:
            self._traffic.popleft()
        span = max(1.0, min(STATS_WINDOW_S, now - self._started))
        topics, senders = {}, {}
        for _, topic, sender, size in self._traffic:
            t = topics.setdefault(topic, [0, 0])
            t[0] += 1
            t[1] += size
            if sender in self.robot_ids:
                s = senders.setdefault(sender, [0, 0, 0, 0])   # all, bytes, bus
                s[0] += 1
                s[1] += size
                if topic.startswith('/fleet/'):
                    s[2] += 1
                    s[3] += size
        rows = [{'topic': k, 'scope': 'p2p' if k.startswith('/fleet/') else 'local',
                 'hz': round(v[0] / span, 1), 'kbps': round(v[1] / span / 1024.0, 2),
                 'avg_bytes': round(v[1] / v[0]) if v[0] else 0}
                for k, v in topics.items()]
        rows.sort(key=lambda r: (r['scope'] != 'p2p', -r['kbps']))
        robots = {rid: {'p2p_hz': round(s[2] / span, 1),
                        'p2p_kbps': round(s[3] / span / 1024.0, 2),
                        'local_kbps': round((s[1] - s[3]) / span / 1024.0, 2)}
                  for rid, s in senders.items()}
        self._stats = {'window_s': round(span, 1), 'topics': rows,
                       'robots': robots,
                       'p2p_kbps': round(sum(r['kbps'] for r in rows
                                             if r['scope'] == 'p2p'), 2)}
        self._stats_t = now
        return self._stats

    def snapshot(self, since_event=None):
        """(p2p view, new events). since_event None = backlog for a new client."""
        now = time.time()
        with self._lock:
            links = []
            for viewer, view in self.views.items():
                stale = now - view['rx'] > 3.0
                for peer, v in view['peers'].items():
                    links.append({'src': peer, 'dst': viewer,
                                  'fresh': 'DEAD' if stale else v.get('fresh'),
                                  'age_ms': v.get('age_ms'),
                                  'intent_seq': v.get('intent_seq'),
                                  'plan_unknown': bool(v.get('plan_unknown')),
                                  'loss': v.get('loss', 0.0),
                                  'view_stale': stale})
            zones = {}
            for rid, view in self.views.items():
                if now - view['rx'] > 3.0:
                    continue
                for z in view['zones_held']:
                    zones.setdefault(z, {'held': [], 'waiting': []})['held'].append(rid)
                for z, granters in view['zones_requesting'].items():
                    zones.setdefault(z, {'held': [], 'waiting': []})['waiting'].append(
                        {'robot': rid, 'granted_by': granters})
            intents = {rid: {k: v for k, v in i.items() if k != 'rx'}
                       | {'age_s': round(now - i['rx'], 1)}
                       for rid, i in self.intents.items()}
            permits = {rid: {k: v for k, v in p.items() if k != 'rx'}
                       | {'age_s': round(now - p['rx'], 1)}
                       for rid, p in self.permits.items()}
            for cell in [c for c, (dl, _) in self.blocked.items() if dl <= now]:
                del self.blocked[cell]
            blocked = []
            for cell, (dl, reporter) in self.blocked.items():
                x, y = self.grid.cell_to_world(cell)
                blocked.append([round(x, 2), round(y, 2), round(dl - now, 1), reporter])
            if since_event is None:
                events = list(self._events)[-FIRST_EVENTS:]
            else:
                events = [e for e in self._events if e['id'] > since_event]
            stats = self._stats_locked(now)
        return ({'transport': self.transport, 'links': links, 'zones': zones,
                 'intents': intents, 'permits': permits, 'blocked': blocked,
                 'stats': stats}, events)
