// Canvas renderer for the warehouse: static layout, robots, paths, tasks and
// the pickup/drop draft. World frame is the ROS map frame (x right, y up,
// metres); the view transform maps it to CSS pixels.

const ROBOT_COLORS = ['#2f6fed', '#e8590c', '#0f9d76', '#a855f7', '#db2777', '#0891b2'];
const BODY_L = 0.42;          // chassis length (m), drawn size
const BODY_W = 0.32;
const CARRY_RED = '#dc2626';  // heading triangle while carrying a picked object
const SNAP_RADIUS_M = 0.5;    // must match the node's snap_radius_m
const SLOT_SNAP_M = 0.35;     // a click this close to a slot's approach point picks the slot

export function robotColor(map, id) {
  const i = map.robots.indexOf(id);
  return ROBOT_COLORS[(i >= 0 ? i : map.robots.length) % ROBOT_COLORS.length];
}

// Merge equal horizontal runs of `ch` across consecutive rows into rects.
function runsToRects(map, ch) {
  const { occupancy, resolution: res, origin } = map;
  const rects = [];
  let open = new Map();                       // "c0:c1" -> rect
  for (let r = 0; r < occupancy.length; r++) {
    const row = occupancy[r];
    const next = new Map();
    let c = 0;
    while (c < row.length) {
      if (row[c] !== ch) { c++; continue; }
      const c0 = c;
      while (c < row.length && row[c] === ch) c++;
      const key = `${c0}:${c - 1}`;
      let rect = open.get(key);
      if (rect) {
        rect.h += res;
      } else {
        rect = { x: origin[0] + c0 * res, y: origin[1] + r * res, w: (c - c0) * res, h: res };
        rects.push(rect);
      }
      next.set(key, rect);
    }
    open = next;
  }
  return rects;
}

export class MapView {
  constructor(canvas, map) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.map = map;
    this.res = map.resolution;
    this.world = {
      x0: map.origin[0], y0: map.origin[1],
      w: map.width * map.resolution, h: map.height * map.resolution,
    };
    this.racks = runsToRects(map, '#');
    this.tight = runsToRects(map, ':');
    this.state = null;
    this.display = new Map();                 // robot id -> smoothed pose
    this.hoverTask = null;
    this.hoverSlot = null;                    // slot label under the cursor
    this.slots = map.slots || [];
    this.slotByLabel = new Map(this.slots.map((sl) => [sl.label, sl]));
    this.draft = null;                        // {mode, pickup, drop, cursor}
    // Extra layers drawn under the robots (after the static map) and over
    // them; each is fn(view, t). The P2P layers register here.
    this.overlays = { under: [], over: [] };
    this.view = { s: 1, tx: 0, ty: 0 };
    this.userMoved = false;                   // refit on resize until zoom/pan
    this.colors = {};
    this._readColors();
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => this._readColors());
    new ResizeObserver(() => this._resize()).observe(canvas);
    this._resize();
    this._last = performance.now();
    requestAnimationFrame((t) => this._frame(t));
  }

  _readColors() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of ['floor', 'grid', 'rack', 'tight', 'zone', 'zone-edge', 'road', 'road-edge', 'pick', 'drop',
      'ok', 'warn', 'bad', 'idle', 'ink', 'muted', 'card']) {
      this.colors[k] = cs.getPropertyValue(`--${k}`).trim();
    }
  }

  // ------------------------------------------------------------ transform
  _resize() {
    const dpr = window.devicePixelRatio || 1;
    const { clientWidth: w, clientHeight: h } = this.canvas;
    if (!w || !h) return;
    this.canvas.width = Math.round(w * dpr);
    this.canvas.height = Math.round(h * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.cssW = w; this.cssH = h;
    if (!this.userMoved) this.fit();
  }

  fit() {
    const m = 12, { w, h, x0, y0 } = this.world;
    const s = Math.min((this.cssW - 2 * m) / w, (this.cssH - 2 * m) / h);
    this.view = { s, tx: this.cssW / 2 - (x0 + w / 2) * s, ty: this.cssH / 2 + (y0 + h / 2) * s };
    this.userMoved = false;
  }

  toScreen(x, y) { return [this.view.tx + x * this.view.s, this.view.ty - y * this.view.s]; }
  toWorld(px, py) { return [(px - this.view.tx) / this.view.s, (this.view.ty - py) / this.view.s]; }

  zoomAt(px, py, factor) {
    const [wx, wy] = this.toWorld(px, py);
    const s = Math.min(600, Math.max(15, this.view.s * factor));
    this.view = { s, tx: px - wx * s, ty: py + wy * s };
    this.userMoved = true;
  }

  pan(dx, dy) { this.view.tx += dx; this.view.ty += dy; this.userMoved = true; }

  // -------------------------------------------------------- cell queries
  cellAt(x, y) {
    const r = Math.floor((y - this.world.y0) / this.res);
    const c = Math.floor((x - this.world.x0) / this.res);
    const row = this.map.occupancy[r];
    return row === undefined || c < 0 || c >= row.length ? null : { r, c, ch: row[c] };
  }

  // 'ok' | 'snap' (the server will nudge it into the aisle) | 'blocked'
  placeability(x, y) {
    const cell = this.cellAt(x, y);
    if (!cell) return 'blocked';
    if (cell.ch === '.') return 'ok';
    const span = Math.ceil(SNAP_RADIUS_M / this.res);
    for (let dr = -span; dr <= span; dr++) {
      const row = this.map.occupancy[cell.r + dr];
      if (!row) continue;
      for (let dc = -span; dc <= span; dc++) {
        if (row[cell.c + dc] !== '.') continue;
        const cx = this.world.x0 + (cell.c + dc + 0.5) * this.res;
        const cy = this.world.y0 + (cell.r + dr + 0.5) * this.res;
        if (Math.hypot(cx - x, cy - y) <= SNAP_RADIUS_M) return 'snap';
      }
    }
    return 'blocked';
  }

  // Nearest rack slot by its approach point (where a robot stops for it).
  slotNear(x, y, maxD = SLOT_SNAP_M) {
    let best = null, bestD = maxD;
    for (const sl of this.slots) {
      const d = Math.hypot(sl.approach[0] - x, sl.approach[1] - y);
      if (d <= bestD) { best = sl; bestD = d; }
    }
    return best;
  }

  // Slot for a click: on the rack face section itself (within 0.3 m of the
  // face line, either side), else the nearest approach point.
  slotAt(x, y) {
    let best = null, bestD = 0.3;
    for (const sl of this.slots) {
      if (x < sl.x0 || x >= sl.x1) continue;
      const d = Math.abs(y - sl.face_y);
      if (d <= bestD) { best = sl; bestD = d; }
    }
    return best || this.slotNear(x, y);
  }

  stationNear(x, y, tol = 0.3) {
    for (const st of this.map.stations) {
      if (Math.hypot(st.x - x, st.y - y) <= tol) return st.id;
    }
    for (const d of this.map.docks) {
      if (Math.hypot(d.x - x, d.y - y) <= tol) return `dock ${d.robot.replace('robot_', '')}`;
    }
    return null;
  }

  setState(state) { this.state = state; }

  addOverlay(where, fn) { this.overlays[where].push(fn); }

  // Smoothed on-screen position of a robot, or null if not known yet.
  robotScreen(id) {
    const d = this.display.get(id);
    return d ? this.toScreen(d.x, d.y) : null;
  }

  // ------------------------------------------------------------- drawing
  _frame(t) {
    const dt = Math.min(0.1, (t - this._last) / 1000);
    this._last = t;
    this._smooth(dt);
    this._draw(t);
    requestAnimationFrame((tt) => this._frame(tt));
  }

  _smooth(dt) {
    if (!this.state) return;
    const k = Math.min(1, dt * 10);
    for (const r of this.state.robots) {
      if (!r.known) continue;
      const d = this.display.get(r.id);
      if (!d || Math.hypot(d.x - r.x, d.y - r.y) > 1.5) {
        this.display.set(r.id, { x: r.x, y: r.y, theta: r.theta });
        continue;
      }
      d.x += (r.x - d.x) * k;
      d.y += (r.y - d.y) * k;
      let da = r.theta - d.theta;
      da = Math.atan2(Math.sin(da), Math.cos(da));
      d.theta += da * k;
    }
  }

  _rect(r, fill) {
    const [x, y] = this.toScreen(r.x, r.y + r.h);
    this.ctx.fillStyle = fill;
    this.ctx.fillRect(x, y, r.w * this.view.s, r.h * this.view.s);
  }

  _draw(t) {
    const { ctx, colors: C } = this;
    if (!this.cssW) return;
    ctx.clearRect(0, 0, this.cssW, this.cssH);
    const { x0, y0, w, h } = this.world;

    // floor + 1 m grid
    this._rect({ x: x0, y: y0, w, h }, C.floor);
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(x0); x <= x0 + w; x++) {
      const [px, py0] = this.toScreen(x, y0), [, py1] = this.toScreen(x, y0 + h);
      ctx.moveTo(px, py0); ctx.lineTo(px, py1);
    }
    for (let y = Math.ceil(y0); y <= y0 + h; y++) {
      const [px0, py] = this.toScreen(x0, y), [px1] = this.toScreen(x0 + w, y);
      ctx.moveTo(px0, py); ctx.lineTo(px1, py);
    }
    ctx.stroke();

    // zones
    ctx.font = '11px system-ui, sans-serif';
    for (const z of this.map.zones) {
      const [r0, c0, r1, c1] = z.rect;
      const rect = { x: x0 + c0 * this.res, y: y0 + r0 * this.res,
        w: (c1 - c0 + 1) * this.res, h: (r1 - r0 + 1) * this.res };
      this._rect(rect, C.zone);
      const [px, py] = this.toScreen(rect.x, rect.y + rect.h);
      ctx.setLineDash([4, 4]); ctx.strokeStyle = C['zone-edge'];
      ctx.strokeRect(px, py, rect.w * this.view.s, rect.h * this.view.s);
      ctx.setLineDash([]);
    }

    for (const r of this.tight) this._rect(r, C.tight);
    for (const r of this.racks) this._rect(r, C.rack);

    // zone names last so racks never cover them; placed in the zone's
    // bottom-left corner, which is open floor for every zone in this map
    ctx.fillStyle = C['zone-edge'];
    for (const z of this.map.zones) {
      const [r0, c0] = z.rect;
      const [px, py] = this.toScreen(x0 + c0 * this.res, y0 + r0 * this.res);
      ctx.fillText(z.id, px + 4, py - 5);
    }

    this._drawRacks();
    this._drawStations();
    this._drawDocks();
    if (this.state) {
      for (const fn of this.overlays.under) fn(this, t);
      this._drawTasks();
      this._drawPaths();
      this._drawRobots(t);
      for (const fn of this.overlays.over) fn(this, t);
    }
    this._drawDraft();
  }

  // Rack names, and the 20 sections of every face as '<side>R<section>'.
  // Detail follows zoom: names only when far out, every 5th label at the
  // default view, every label when zoomed in.
  _drawRacks() {
    const { ctx, colors: C } = this;
    const s = this.view.s;
    for (const r of this.map.racks || []) {
      const [cx, cy] = this.toScreen((r.x_min + r.x_max) / 2, (r.y_min + r.y_max) / 2);
      ctx.fillStyle = C.floor; ctx.globalAlpha = 0.9;
      ctx.font = `700 ${s >= 60 ? 11 : 9}px system-ui, sans-serif`;
      ctx.textAlign = 'center'; ctx.fillText(`Rack ${r.id}`, cx, cy + 4); ctx.textAlign = 'left';
    }
    if (s < 45 || !this.slots.length) { ctx.globalAlpha = 1; return; }
    const every = s >= 140 ? 1 : 5;
    ctx.font = `600 ${s >= 140 ? 9 : 8}px system-ui, sans-serif`;
    ctx.textAlign = 'center';
    for (const sl of this.slots) {
      const inward = sl.face === 'N' ? -1 : 1;     // rack body is south of a N face
      const [ax, ay] = this.toScreen(sl.x0, sl.face_y);
      const [, ty] = this.toScreen(sl.x0, sl.face_y + inward * 0.07);
      ctx.strokeStyle = C.floor; ctx.globalAlpha = 0.55; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(ax, ty); ctx.stroke();
      const hot = this.hoverSlot === sl.label;
      if (hot) {
        const [hx0, hy0] = this.toScreen(sl.x0, sl.face_y);
        const [hx1] = this.toScreen(sl.x1, sl.face_y);
        ctx.globalAlpha = 1; ctx.strokeStyle = C.pick; ctx.lineWidth = 4;
        ctx.beginPath(); ctx.moveTo(hx0, hy0); ctx.lineTo(hx1, hy0); ctx.stroke();
      }
      if (hot || sl.section === 1 || sl.section % every === 0) {
        const [lx, ly] = this.toScreen((sl.x0 + sl.x1) / 2, sl.face_y + inward * 0.15);
        ctx.globalAlpha = hot ? 1 : 0.8;
        ctx.fillStyle = hot ? C.pick : C.floor;
        ctx.fillText(sl.label, lx, ly + 3);
      }
    }
    ctx.textAlign = 'left'; ctx.globalAlpha = 1;
  }

  _marker(x, y, glyph, color, label, size = 9) {
    const { ctx } = this;
    const [px, py] = this.toScreen(x, y);
    ctx.fillStyle = color;
    ctx.beginPath();
    if (glyph === 'up') { ctx.moveTo(px, py - size); ctx.lineTo(px + size, py + size * 0.7); ctx.lineTo(px - size, py + size * 0.7); }
    else { ctx.moveTo(px, py + size); ctx.lineTo(px + size, py - size * 0.7); ctx.lineTo(px - size, py - size * 0.7); }
    ctx.closePath(); ctx.fill();
    if (label) {
      ctx.font = '600 11px system-ui, sans-serif';
      ctx.fillText(label, px + size + 3, py + 4);
    }
  }

  _drawStations() {
    for (const st of this.map.stations) {
      this._marker(st.x, st.y, st.kind === 'pickup' ? 'up' : 'down',
        st.kind === 'pickup' ? this.colors.pick : this.colors.drop, st.id);
    }
  }

  _drawDocks() {
    const { ctx } = this;
    const half = 0.28 * this.view.s;
    for (const d of this.map.docks) {
      const col = robotColor(this.map, d.robot);
      const [px, py] = this.toScreen(d.x, d.y);
      ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.setLineDash([5, 3]);
      ctx.beginPath(); ctx.roundRect(px - half, py - half, 2 * half, 2 * half, 6); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = col;
      ctx.font = '600 10px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`⚡ dock ${d.robot.replace('robot_', '')}`, px, py + half + 12);
      ctx.textAlign = 'left';
    }
  }

  _drawTasks() {
    const { ctx, colors: C } = this;
    for (const t of this.state.tasks) {
      if (t.status === 'DONE' || t.status === 'ABORTED' || !t.pickup || !t.drop) continue;
      const col = t.robot ? robotColor(this.map, t.robot) : C.idle;
      const hot = this.hoverTask === t.id;
      const [ax, ay] = this.toScreen(t.pickup[0], t.pickup[1]);
      const [bx, by] = this.toScreen(t.drop[0], t.drop[1]);
      ctx.strokeStyle = col; ctx.lineWidth = hot ? 3 : 1.5; ctx.globalAlpha = hot ? 1 : 0.7;
      ctx.setLineDash([6, 5]);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      ctx.setLineDash([]);
      // arrow head at the drop end
      const a = Math.atan2(by - ay, bx - ax), L = 10;
      ctx.fillStyle = col;
      ctx.beginPath();
      ctx.moveTo(bx - 9 * Math.cos(a), by - 9 * Math.sin(a));
      ctx.lineTo(bx - (9 + L) * Math.cos(a) + 5 * Math.sin(a), by - (9 + L) * Math.sin(a) - 5 * Math.cos(a));
      ctx.lineTo(bx - (9 + L) * Math.cos(a) - 5 * Math.sin(a), by - (9 + L) * Math.sin(a) + 5 * Math.cos(a));
      ctx.closePath(); ctx.fill();
      this._badge(ax, ay, 'P', C.pick, hot);
      this._badge(bx, by, 'D', C.drop, hot);
      ctx.globalAlpha = 1;
      if (hot) {
        ctx.fillStyle = C.ink; ctx.font = '600 11px system-ui, sans-serif';
        ctx.fillText(t.id, ax + 12, ay - 10);
      }
    }
  }

  _badge(px, py, letter, color, big) {
    const { ctx } = this;
    const r = big ? 10 : 8;
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(px, py, r, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#fff'; ctx.font = `700 ${big ? 11 : 10}px system-ui, sans-serif`;
    ctx.textAlign = 'center'; ctx.fillText(letter, px, py + 4); ctx.textAlign = 'left';
  }

  _drawPaths() {
    const { ctx } = this;
    for (const [id, pts] of Object.entries(this.state.paths)) {
      if (pts.length < 2) continue;
      const pose = this.display.get(id);
      // Start the polyline at the point nearest the robot: the published
      // path still holds the cells it already drove through.
      let start = 0;
      if (pose) {
        let best = Infinity;
        pts.forEach((p, i) => {
          const d = (p[0] - pose.x) ** 2 + (p[1] - pose.y) ** 2;
          if (d < best) { best = d; start = i; }
        });
      }
      ctx.strokeStyle = robotColor(this.map, id); ctx.lineWidth = 3; ctx.globalAlpha = 0.55;
      ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      if (pose) ctx.moveTo(...this.toScreen(pose.x, pose.y));
      else ctx.moveTo(...this.toScreen(pts[start][0], pts[start][1]));
      for (let i = start; i < pts.length; i++) ctx.lineTo(...this.toScreen(pts[i][0], pts[i][1]));
      ctx.stroke();
      ctx.globalAlpha = 1;
      const end = pts[pts.length - 1];
      const [ex, ey] = this.toScreen(end[0], end[1]);
      ctx.fillStyle = robotColor(this.map, id);
      ctx.beginPath(); ctx.arc(ex, ey, 4, 0, Math.PI * 2); ctx.fill();
    }
  }

  _drawRobots(t) {
    const { ctx, colors: C } = this;
    const s = this.view.s;
    const L = Math.max(16, BODY_L * s), W = Math.max(12, BODY_W * s);
    for (const r of this.state.robots) {
      const pose = this.display.get(r.id);
      if (!r.known || !pose) continue;
      const col = robotColor(this.map, r.id);
      const offline = r.status === 'OFFLINE';
      const [px, py] = this.toScreen(pose.x, pose.y);

      // status halo
      const halo = { FAULT: C.bad, WAITING: C.warn, YIELDING: C.warn, CHARGING: C.ok }[r.status];
      if (halo) {
        const pulse = r.status === 'CHARGING' ? 0.5 + 0.5 * Math.sin(t / 350) : 1;
        ctx.strokeStyle = halo; ctx.lineWidth = 3; ctx.globalAlpha = 0.35 + 0.5 * pulse;
        ctx.beginPath(); ctx.arc(px, py, L * 0.75, 0, Math.PI * 2); ctx.stroke();
        ctx.globalAlpha = 1;
      }

      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-pose.theta);
      ctx.globalAlpha = offline ? 0.35 : 1;
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.roundRect(-L / 2, -W / 2, L, W, 5); ctx.fill();
      // Heading triangle: white when empty, RED once the object is picked
      // (task phase DROPOFF = carrying it to the drop station).
      const carrying = r.task_id && r.task_phase === 'DROPOFF';
      ctx.fillStyle = carrying ? CARRY_RED : '#fff';
      ctx.beginPath(); ctx.moveTo(L * 0.32, 0); ctx.lineTo(-L * 0.05, -W * 0.28); ctx.lineTo(-L * 0.05, W * 0.28); ctx.closePath(); ctx.fill();
      if (carrying) { ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.2; ctx.stroke(); }
      ctx.restore();
      ctx.globalAlpha = offline ? 0.5 : 1;

      // Strict lanes: a gated U-turn / against-lane retreat is a legal
      // manoeuvre - say so on the map so it never reads as a fault.
      const move = r.uturn_active ? ' · U-TURN' : r.overtake_active ? ' · OVERTAKING'
        : r.reverse_active ? ' · REVERSING' : '';
      const label = `${r.id.replace('robot_', 'R')} · ${r.battery.toFixed(0)}%${r.status === 'CHARGING' ? ' ⚡' : ''}${move}`;
      ctx.font = '600 11px system-ui, sans-serif';
      const tw = ctx.measureText(label).width;
      ctx.fillStyle = C.card; ctx.globalAlpha *= 0.85;
      ctx.beginPath(); ctx.roundRect(px - tw / 2 - 5, py - L * 0.75 - 20, tw + 10, 16, 4); ctx.fill();
      ctx.globalAlpha = offline ? 0.5 : 1;
      ctx.fillStyle = move ? C.warn : C.ink; ctx.textAlign = 'center';
      ctx.fillText(label, px, py - L * 0.75 - 8);
      ctx.textAlign = 'left'; ctx.globalAlpha = 1;
    }
  }

  _drawDraft() {
    const d = this.draft;
    if (!d) return;
    const { ctx, colors: C } = this;
    if (d.pickup && (d.drop || d.cursor)) {
      const b = d.drop || d.cursor;
      const [ax, ay] = this.toScreen(...d.pickup), [bx, by] = this.toScreen(b[0], b[1]);
      ctx.strokeStyle = C.ink; ctx.lineWidth = 2; ctx.setLineDash([3, 4]);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
    }
    for (const [pt, letter, col, slot] of [[d.pickup, 'P', C.pick, d.pickupSlot],
      [d.drop, 'D', C.drop, d.dropSlot]]) {
      if (!pt) continue;
      const [bx, by] = this.toScreen(...pt);
      this._badge(bx, by, letter, col, true);
      if (slot) {
        ctx.font = '700 11px system-ui, sans-serif';
        const tw = ctx.measureText(slot).width;
        ctx.fillStyle = C.card; ctx.fillRect(bx + 12, by - 8, tw + 8, 16);
        ctx.fillStyle = col; ctx.fillText(slot, bx + 16, by + 4);
      }
    }
    if (d.cursor && d.mode !== 'confirm') {
      const [px, py] = this.toScreen(d.cursor[0], d.cursor[1]);
      const ok = d.cursor[2];
      ctx.strokeStyle = ok === 'ok' ? C.ok : ok === 'snap' ? C.warn : C.bad;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, 0.25 * this.view.s, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(px - 6, py); ctx.lineTo(px + 6, py); ctx.moveTo(px, py - 6); ctx.lineTo(px, py + 6); ctx.stroke();
    }
  }
}
