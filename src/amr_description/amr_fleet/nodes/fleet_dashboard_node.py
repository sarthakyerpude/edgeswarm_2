"""Read-only browser dashboard for fleet position, state, and battery."""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node

from amr_description.msg import RobotState
from amr_fleet.nodes.qos_profiles import STATE_QOS


STATUS = ['IDLE', 'MOVING', 'WAITING', 'YIELDING', 'CHARGING', 'FAULT']
PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>EdgeSwarm Fleet</title><style>
body{font:15px system-ui,sans-serif;margin:0;background:#f4f7fb;color:#172033}
header{padding:16px 24px;background:#14243a;color:white;display:flex;justify-content:space-between}
main{padding:18px;max-width:1200px;margin:auto}.card{background:white;padding:14px;border-radius:10px;box-shadow:0 2px 12px #14243a12}
canvas{width:100%;height:auto;display:block}.legend{display:flex;gap:14px;flex-wrap:wrap;padding:10px}
.dot{display:inline-block;width:12px;height:12px;border-radius:50%;margin-right:5px}
table{width:100%;border-collapse:collapse;margin-top:14px}td,th{text-align:left;padding:9px;border-bottom:1px solid #e4e9f0}
.muted{color:#aebbd0;font-size:13px}
</style></head><body><header><strong>EdgeSwarm · Fleet Dashboard</strong><span id="stamp" class="muted">Connecting…</span></header>
<main><section class="card"><canvas id="map" width="1000" height="620"></canvas>
<div class="legend"><span><i class="dot" style="background:#1685d1"></i>Moving</span><span><i class="dot" style="background:#e39a17"></i>Waiting / Yielding</span><span><i class="dot" style="background:#718096"></i>Idle / Offline</span><span><i class="dot" style="background:#d64545"></i>Fault</span></div></section>
<section class="card" style="margin-top:16px"><table><thead><tr><th>Robot</th><th>Status</th><th>Position (m)</th><th>Battery estimate</th><th>Message age</th></tr></thead><tbody id="robots"></tbody></table></section></main>
<script>
const colors={MOVING:'#1685d1',WAITING:'#e39a17',YIELDING:'#e39a17',FAULT:'#d64545',IDLE:'#718096',OFFLINE:'#718096',CHARGING:'#2b9b66'};
function esc(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function render(data){const c=document.getElementById('map'),x=c.getContext('2d'),w=c.width,h=c.height;
x.clearRect(0,0,w,h);x.fillStyle='#fbfcfe';x.fillRect(0,0,w,h);x.strokeStyle='#e7ecf2';x.lineWidth=1;
for(let i=0;i<=12;i++){let px=i*w/12;x.beginPath();x.moveTo(px,0);x.lineTo(px,h);x.stroke()}
for(let i=0;i<=10;i++){let py=i*h/10;x.beginPath();x.moveTo(0,py);x.lineTo(w,py);x.stroke()}
x.strokeStyle='#475569';x.lineWidth=3;x.strokeRect(1,1,w-2,h-2);
for(const r of data.robots){if(!r.known)continue;let px=(r.x+6)/12*w,py=h-(r.y+5)/10*h,col=colors[r.status]||'#718096';
x.save();x.translate(px,py);x.rotate(-r.theta);x.fillStyle=col;x.beginPath();x.roundRect(-19,-14,38,28,6);x.fill();
x.fillStyle='white';x.beginPath();x.moveTo(7,0);x.lineTo(-2,-6);x.lineTo(-2,6);x.closePath();x.fill();x.restore();
x.fillStyle='#14243a';x.font='bold 14px system-ui';x.fillText(`${r.id} · ${r.battery.toFixed(0)}%`,px-35,py-21)}
document.getElementById('robots').innerHTML=data.robots.map(r=>{const position=r.known?`${r.x.toFixed(2)}, ${r.y.toFixed(2)}`:'—',battery=r.known?`${r.battery.toFixed(1)}%`:'—';return `<tr><td>${esc(r.id)}</td><td>${esc(r.status)}</td><td>${position}</td><td>${battery}</td><td>${r.age_ms.toFixed(0)} ms</td></tr>`}).join('')||'<tr><td colspan="5">Waiting for robot state…</td></tr>';
document.getElementById('stamp').textContent=`${data.robots.length} robots · updated ${new Date().toLocaleTimeString()}`}
async function update(){try{const r=await fetch('/api/state',{cache:'no-store'});render(await r.json())}catch(e){document.getElementById('stamp').textContent='Dashboard connection lost'}}
update();setInterval(update,500);
</script></body></html>"""


class FleetDashboardNode(Node):
    def __init__(self):
        super().__init__('fleet_dashboard')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8080)
        self.declare_parameter('robot_ids', ['robot_1', 'robot_2', 'robot_3'])
        self.robot_ids = sorted({str(robot_id) for robot_id in
                                 self.get_parameter('robot_ids').value})
        self.states = {}
        self.received_at = {}
        self._lock = threading.Lock()
        self.create_subscription(RobotState, '/fleet/robot_state',
                                 self._on_state, STATE_QOS)
        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/api/state':
                    body = json.dumps(node.snapshot()).encode('utf-8')
                    content_type = 'application/json; charset=utf-8'
                elif self.path == '/':
                    body = PAGE.encode('utf-8')
                    content_type = 'text/html; charset=utf-8'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                return

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True)
        self._thread.start()
        self.get_logger().info(f'fleet dashboard available at http://{host}:{port}')

    def _on_state(self, message):
        with self._lock:
            self.states[message.robot_id] = message
            self.received_at[message.robot_id] = time.monotonic()

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            rows = []
            robot_ids = sorted(set(self.robot_ids) | set(self.states))
            for robot_id in robot_ids:
                state = self.states.get(robot_id)
                if state is None:
                    rows.append({
                        'id': robot_id, 'status': 'OFFLINE',
                        'x': 0.0, 'y': 0.0, 'theta': 0.0,
                        'battery': 0.0, 'age_ms': 999999999.0,
                        'known': False,
                    })
                    continue
                idx = int(state.status)
                age_ms = max(0.0, now - self.received_at[robot_id]) * 1000.0
                status = STATUS[idx] if 0 <= idx < len(STATUS) else 'UNKNOWN'
                if age_ms > 1500.0:
                    status = 'OFFLINE'
                rows.append({
                    'id': robot_id,
                    'status': status,
                    'x': float(state.pose.x), 'y': float(state.pose.y),
                    'theta': float(state.pose.theta),
                    'battery': float(state.battery_pct),
                    'age_ms': age_ms,
                    'known': True,
                })
            return {'robots': rows}

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FleetDashboardNode()
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
