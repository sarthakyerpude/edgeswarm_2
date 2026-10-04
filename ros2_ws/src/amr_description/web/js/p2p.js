// P2P view: what the robots tell each other, straight off the shared bus.
// Map layers (peer links, broadcast intents, zone locks + waits, shared
// blockages) and the sidebar panel (live message feed, link matrix, traffic).
import { robotColor } from './map.js';
import { esc } from './ui.js';

const LAYERS = { roads: true, hitboxes: true, links: true, intents: true, locks: true, blocked: true };
const KINDS = ['task', 'zone', 'intent', 'permit', 'conflict', 'map', 'peer'];
const KIND_COLOR = {
  task: '#2f6fed', zone: '#7c3aed', intent: '#16a34a', permit: '#d97706',
  conflict: '#dc2626', map: '#b45309', peer: '#64748b',
};
const FRESH_RANK = { FRESH: 0, SUSPECT: 1, DEAD: 2 };
const HOLD = new Set(['STOP', 'YIELD', 'REROUTE']);
const FLIGHT_MS = 900;
// Hitboxes - wiki/Priority-RightOfWay-Hitbox.md (judge's merged spec).
// SAFETY footprint: oriented rect, half-length 0.20 m, half-width 0.205 m
// (wheel-inclusive); a robot spinning faster than 0.3 rad/s is its 0.286 m
// circumscribed disc. BODY outline: 0.40 x 0.32 chassis plus wheel bumps.
// Robots i, j must stop when their safety footprints are closer than
//   g_ij = G_MARGIN + 2*sqrt(si^2 + sj^2) + min(1.5, v_max*age) (if stale)
//   G_MARGIN = 0.12 + 0.30*min(1, v_close/1.0)   (rescaled for 1.0 m/s cruise)
// with sigma floored at 0.05 (>= 0.20 when DEGRADED, 0.6 if non-finite).
const HB_HALF_L = 0.20, HB_HALF_W = 0.205;          // safety rect (wheels)
const HB_BODY_L = 0.40, HB_BODY_W = 0.32;           // chassis
const HB_WHEEL_HALF_L = 0.05;                       // wheel bump, along x
const HB_DISC_R = 0.286;                            // circumscribed radius
const HB_SPIN_W = 0.3;                              // rad/s -> treat as disc
const HB_SIGMA_FLOOR = 0.05, HB_SIGMA_DEGRADED = 0.20, HB_SIGMA_BAD = 0.6;
// = core/hitbox.py (SAFETY_SCALED 2026-10-04): no overall cap; only the
// stale-age reach is capped at REACH_CAP_M.
const HB_G0 = 0.12, HB_G_CLOSE = 0.30, HB_V_REF = 1.0, HB_REACH_CAP = 1.5;
const HB_V_MAX = 1.0;                               // m/s, stale-age growth (= hitbox.V_MAX)
const HB_STALE_S = 0.3;                             // FRESH threshold (peers.py)
const HB_SHOW_PAIR_M = 0.6;                         // show a pair gap within g + this

function hbSigma(r) {
  let sg = Number.isFinite(r.sigma) ? r.sigma : HB_SIGMA_BAD;
  sg = Math.max(sg, HB_SIGMA_FLOOR);
  if (r.loc_health === 1) sg = Math.max(sg, HB_SIGMA_DEGRADED);
  return sg;
}
const hbAge = (r) => { const a = (r.age_ms || 0) / 1000; return a > HB_STALE_S ? a : 0; };
// The robot's own verdict (spinning OR about to turn > 45 deg) when its
// agent publishes one; otherwise the |w| test alone.
const hbSpinning = (r) => (typeof r.disc_mode === 'boolean'
  ? r.disc_mode : Math.abs(r.w || 0) > HB_SPIN_W);

// Safety footprint corners in world coordinates (null for a disc).
function hbCorners(r) {
  const c = Math.cos(r.theta), s = Math.sin(r.theta);
  return [[1, 1], [-1, 1], [-1, -1], [1, -1]].map(([a, b]) => {
    const lx = a * HB_HALF_L, ly = b * HB_HALF_W;
    return [r.x + lx * c - ly * s, r.y + lx * s + ly * c];
  });
}

function segPoint(p, a, b) {          // closest point on segment ab to p
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const t = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy || 1)));
  return [a[0] + t * dx, a[1] + t * dy];
}

function inPoly(p, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > p[1]) !== (yj > p[1]) && p[0] < ((xj - xi) * (p[1] - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

// Gap between two safety footprints and the closest points [gap, pa, pb].
function hbGap(ra, rb) {
  const discA = hbSpinning(ra), discB = hbSpinning(rb);
  const ca = [ra.x, ra.y], cb = [rb.x, rb.y];
  if (discA && discB) {
    const d = Math.hypot(cb[0] - ca[0], cb[1] - ca[1]) || 1e-9;
    const u = [(cb[0] - ca[0]) / d, (cb[1] - ca[1]) / d];
    return [d - 2 * HB_DISC_R, [ca[0] + u[0] * HB_DISC_R, ca[1] + u[1] * HB_DISC_R],
      [cb[0] - u[0] * HB_DISC_R, cb[1] - u[1] * HB_DISC_R]];
  }
  if (discA || discB) {                // disc vs rectangle
    const [disc, rect, flip] = discA ? [ra, rb, false] : [rb, ra, true];
    const poly = hbCorners(rect), c = [disc.x, disc.y];
    let best = null, bestD = Infinity;
    for (let i = 0; i < 4; i++) {
      const q = segPoint(c, poly[i], poly[(i + 1) % 4]);
      const d = Math.hypot(q[0] - c[0], q[1] - c[1]);
      if (d < bestD) { bestD = d; best = q; }
    }
    const inside = inPoly(c, poly);
    const u = [(best[0] - c[0]) / (bestD || 1), (best[1] - c[1]) / (bestD || 1)];
    const pd = [c[0] + u[0] * HB_DISC_R, c[1] + u[1] * HB_DISC_R];
    const gap = inside ? -(bestD + HB_DISC_R) : bestD - HB_DISC_R;
    return flip ? [gap, best, pd] : [gap, pd, best];
  }
  const A = hbCorners(ra), B = hbCorners(rb);
  if (A.some((p) => inPoly(p, B)) || B.some((p) => inPoly(p, A))) return [-1, ca, cb];
  let best = [Infinity, ca, cb];
  for (const [P, Q, swap] of [[A, B, false], [B, A, true]]) {
    for (const p of P) {
      for (let i = 0; i < 4; i++) {
        const q = segPoint(p, Q[i], Q[(i + 1) % 4]);
        const d = Math.hypot(q[0] - p[0], q[1] - p[1]);
        if (d < best[0]) best = swap ? [d, q, p] : [d, p, q];
      }
    }
  }
  return best;
}

// Pairwise stop distance g_ij from the spec.
function hbStopDist(ra, rb) {
  const dx = rb.x - ra.x, dy = rb.y - ra.y, d = Math.hypot(dx, dy) || 1e-9;
  const va = [(ra.v || 0) * Math.cos(ra.theta), (ra.v || 0) * Math.sin(ra.theta)];
  const vb = [(rb.v || 0) * Math.cos(rb.theta), (rb.v || 0) * Math.sin(rb.theta)];
  const vClose = Math.max(0, -((vb[0] - va[0]) * dx + (vb[1] - va[1]) * dy) / d);
  const g = HB_G0 + HB_G_CLOSE * Math.min(1, vClose / HB_V_REF)
    + 2 * Math.hypot(hbSigma(ra), hbSigma(rb))
    + Math.min(HB_REACH_CAP, HB_V_MAX * Math.max(hbAge(ra), hbAge(rb)));
  return g;
}
const MAX_FEED = 400;
const FEED_ROWS = 60;           // newest lines shown; the sidebar is the only scroller
const RMW_NAMES = {
  rmw_zenoh_cpp: 'Zenoh', rmw_cyclonedds_cpp: 'CycloneDDS', rmw_fastrtps_cpp: 'Fast DDS',
};

const short = (id) => String(id).replace(/^robot_/, 'R');
const lerp = (a, b, f) => [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];

function loadLayers() {
  try {
    return { ...LAYERS, ...JSON.parse(localStorage.getItem('p2pLayers') || '{}') };
  } catch { return { ...LAYERS }; }
}

export class P2PView {
  constructor(view, els) {
    this.view = view;
    this.els = els;
    this.layers = loadLayers();
    this.filters = new Set(KINDS);
    this.paused = false;
    this.feed = [];
    this.flights = [];
    this.p2p = null;
    this.pane = 'feed';
    view.addOverlay('under', (v, t) => this._drawUnder(v, t));
    view.addOverlay('over', (v, t) => this._drawOver(v, t));
    this._wire();
  }

  // ------------------------------------------------------------- controls
  _wire() {
    const { els } = this;
    els.layers.querySelectorAll('input[data-layer]').forEach((cb) => {
      cb.checked = !!this.layers[cb.dataset.layer];
      cb.addEventListener('change', () => {
        this.layers[cb.dataset.layer] = cb.checked;
        try { localStorage.setItem('p2pLayers', JSON.stringify(this.layers)); } catch { /* private mode */ }
      });
    });
    els.filters.innerHTML = KINDS.map((k) => `<button class="fchip on" data-kind="${k}"
      style="--c:${KIND_COLOR[k]}">${k}</button>`).join('')
      + '<button class="btn ghost small" id="feedPause">Pause</button>';
    els.filters.addEventListener('click', (e) => {
      const chip = e.target.closest('[data-kind]');
      if (chip) {
        const k = chip.dataset.kind;
        if (this.filters.has(k)) this.filters.delete(k); else this.filters.add(k);
        chip.classList.toggle('on', this.filters.has(k));
        this.renderPanel(true);
      } else if (e.target.id === 'feedPause') {
        this.paused = !this.paused;
        e.target.textContent = this.paused ? 'Resume' : 'Pause';
        this.renderPanel(true);
      }
    });
    els.tabs.forEach((b) => b.addEventListener('click', () => {
      this.pane = b.dataset.ptab;
      els.tabs.forEach((x) => x.classList.toggle('active', x === b));
      els.feedPane.hidden = this.pane !== 'feed';
      els.netPane.hidden = this.pane !== 'net';
      this.renderPanel(true);
    }));
  }

  // --------------------------------------------------------------- data
  update(data) {
    this.p2p = data.p2p || null;
    const robots = new Set(this.view.map.robots);
    const now = performance.now();
    for (const ev of data.events || []) {
      this.feed.push(ev);
      // Animate robot-to-robot traffic: a broadcast flies to every peer.
      if (!robots.has(ev.src) || ev.kind === 'peer' || ev.kind === 'permit') continue;
      const dsts = ev.dst === '*' ? [...robots].filter((r) => r !== ev.src)
        : robots.has(ev.dst) ? [ev.dst] : [];
      for (const d of dsts) this.flights.push({ src: ev.src, dst: d, t0: now, kind: ev.kind });
    }
    if (this.feed.length > MAX_FEED) this.feed.splice(0, this.feed.length - MAX_FEED);
    if (this.flights.length > 80) this.flights.splice(0, this.flights.length - 80);
  }

  linkSummary() {
    if (!this.p2p) return null;
    const links = this.p2p.links;
    return { fresh: links.filter((l) => l.fresh === 'FRESH').length, total: links.length };
  }

  // -------------------------------------------------------------- map layers
  _drawUnder(v, t) {
    if (this.layers.roads) this._drawRoads(v);
    if (!this.p2p) return;
    if (this.layers.locks) this._drawZoneLocks(v);
    if (this.layers.blocked) this._drawBlocked(v);
    if (this.layers.intents) this._drawIntents(v);
    if (this.layers.links) this._drawLinks(v, t);
  }

  _drawOver(v) {
    if (this.layers.hitboxes) this._drawHitboxes(v);
    if (!this.p2p) return;
    if (this.layers.locks) this._drawWaits(v);
    if (this.layers.links) this._drawFlights(v);
  }

  // Two-way roads drawn like streets: road surface, edge lines, a double
  // yellow centre line between the opposite lanes, direction arrows, and
  // yellow cross-hatched BOX junctions where roads cross (don't block the
  // box). Junction boxes come from the yaml when given, plus any other
  // road crossing derived from the lane geometry.
  _roadGeometry(map) {
    if (this._roads && this._roads.map === map) return this._roads;
    const t = map.traffic || {};
    const half = (map.aisle_width_m || 1.6) / 2;
    const groups = new Map();
    for (const ln of t.lanes || []) {
      const key = ln.id.replace(/_(NB|SB|EB|WB)$/, '');
      if (!groups.has(key)) groups.set(key, []);
      groups.get(key).push(ln);
    }
    const roads = [];
    for (const [id, lanes] of groups) {
      const pts = lanes.flatMap((l) => l.polyline);
      const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
      const vertical = (Math.max(...xs) - Math.min(...xs)) < (Math.max(...ys) - Math.min(...ys));
      const cx = (Math.min(...xs) + Math.max(...xs)) / 2, cy = (Math.min(...ys) + Math.max(...ys)) / 2;
      roads.push(vertical
        ? { id, vertical, x0: cx - half, x1: cx + half, y0: Math.min(...ys), y1: Math.max(...ys), c: cx, lanes }
        : { id, vertical, x0: Math.min(...xs), x1: Math.max(...xs), y0: cy - half, y1: cy + half, c: cy, lanes });
    }
    // Geometry comes from the actual road crossings (every vertical x
    // horizontal road; a T-junction counts when the vertical road ends within
    // one road-width of the horizontal one). Yaml junctions lend their ids;
    // a yaml junction that matches no crossing is kept as given.
    const yamlJ = (t.junctions || []).map((j) => ({ id: j.id, cx: j.center[0], cy: j.center[1],
      x0: j.center[0] - j.half_size[0], x1: j.center[0] + j.half_size[0],
      y0: j.center[1] - j.half_size[1], y1: j.center[1] + j.half_size[1], used: false }));
    const boxes = [];
    for (const v of roads.filter((r) => r.vertical)) {
      for (const h of roads.filter((r) => !r.vertical)) {
        const overlapX = v.x0 < h.x1 && v.x1 > h.x0;
        const overlapY = v.y0 - 2 * half < h.y1 && v.y1 + 2 * half > h.y0;
        if (!overlapX || !overlapY) continue;
        const box = { id: `${v.id}×${h.id}`, x0: v.x0, x1: v.x1, y0: h.y0, y1: h.y1 };
        const cxb = (box.x0 + box.x1) / 2, cyb = (box.y0 + box.y1) / 2;
        const named = yamlJ.find((j) => !j.used && Math.hypot(j.cx - cxb, j.cy - cyb) < 1.0);
        if (named) { named.used = true; box.id = named.id; }
        boxes.push(box);
      }
    }
    for (const j of yamlJ) if (!j.used) boxes.push(j);
    // Strict-lane crossing zones (yaml traffic.turnarounds): aisle dead-end
    // turnarounds and the throat / spur-mouth aprons - the only places
    // outside junction boxes where a robot may cross into the other lane.
    const turns = (t.turnarounds || []).map((z) => ({ id: z.id,
      x0: z.center[0] - z.half_size[0], x1: z.center[0] + z.half_size[0],
      y0: z.center[1] - z.half_size[1], y1: z.center[1] + z.half_size[1] }));
    this._roads = { map, roads, boxes, turns };
    return this._roads;
  }

  _drawRoads(v) {
    const { roads, boxes, turns } = this._roadGeometry(v.map);
    if (!roads.length) return;
    const { ctx, colors: C } = v;
    const rect = (x0, y0, x1, y1) => {
      const [a, b] = v.toScreen(x0, y1), [c, d] = v.toScreen(x1, y0);
      return [a, b, c - a, d - b];
    };
    // 1. road surface + edge lines
    for (const r of roads) {
      ctx.globalAlpha = 0.8; ctx.fillStyle = C.road; ctx.fillRect(...rect(r.x0, r.y0, r.x1, r.y1));
      ctx.globalAlpha = 0.9; ctx.strokeStyle = C['road-edge']; ctx.lineWidth = 1.2;
      ctx.beginPath();
      if (r.vertical) {
        for (const x of [r.x0, r.x1]) { ctx.moveTo(...v.toScreen(x, r.y0)); ctx.lineTo(...v.toScreen(x, r.y1)); }
      } else {
        for (const y of [r.y0, r.y1]) { ctx.moveTo(...v.toScreen(r.x0, y)); ctx.lineTo(...v.toScreen(r.x1, y)); }
      }
      ctx.stroke();
    }
    // 2. double yellow centre line (separates the two directions)
    ctx.strokeStyle = '#eab308'; ctx.globalAlpha = 0.9; ctx.lineWidth = 1.4;
    const gap = 2;
    for (const r of roads) {
      const [ax, ay] = r.vertical ? v.toScreen(r.c, r.y0) : v.toScreen(r.x0, r.c);
      const [bx, by] = r.vertical ? v.toScreen(r.c, r.y1) : v.toScreen(r.x1, r.c);
      for (const o of [-gap, gap]) {
        ctx.beginPath();
        if (r.vertical) { ctx.moveTo(ax + o, ay); ctx.lineTo(bx + o, by); }
        else { ctx.moveTo(ax, ay + o); ctx.lineTo(bx, by + o); }
        ctx.stroke();
      }
    }
    // 3. lane direction arrows
    const dirCol = { NB: '#0ea5e9', EB: '#0ea5e9', SB: '#f59e0b', WB: '#f59e0b' };
    const step = Math.max(28, 0.9 * v.view.s);
    for (const r of roads) {
      for (const ln of r.lanes) {
        const pts = ln.polyline.map(([x, y]) => v.toScreen(x, y));
        ctx.fillStyle = dirCol[ln.direction] || C.idle; ctx.globalAlpha = 0.75;
        for (let i = 0; i + 1 < pts.length; i++) {
          const [ax, ay] = pts[i], [bx, by] = pts[i + 1];
          const len = Math.hypot(bx - ax, by - ay); if (len < 1) continue;
          const ux = (bx - ax) / len, uy = (by - ay) / len;
          for (let d = step / 2; d < len; d += step) {
            const cx = ax + ux * d, cy = ay + uy * d;
            ctx.beginPath();
            ctx.moveTo(cx + ux * 6, cy + uy * 6);
            ctx.lineTo(cx - ux * 4 - uy * 5, cy - uy * 4 + ux * 5);
            ctx.lineTo(cx - ux * 1, cy - uy * 1);
            ctx.lineTo(cx - ux * 4 + uy * 5, cy - uy * 4 - ux * 5);
            ctx.closePath(); ctx.fill();
          }
        }
      }
    }
    // 4. box junctions: road-coloured (hides the centre line), yellow
    //    cross-hatch + border; red if two robots are inside at once.
    const live = (v.state.robots || []).filter((rb) => rb.known && rb.status !== 'OFFLINE');
    ctx.font = '700 10px system-ui, sans-serif';
    for (const bx of boxes) {
      const [px, py, pw, ph] = rect(bx.x0, bx.y0, bx.x1, bx.y1);
      const inside = live.filter((rb) => rb.x >= bx.x0 && rb.x <= bx.x1 && rb.y >= bx.y0 && rb.y <= bx.y1);
      const col = inside.length > 1 ? C.bad : '#eab308';
      ctx.globalAlpha = 1; ctx.fillStyle = C.road; ctx.fillRect(px, py, pw, ph);
      ctx.save();
      ctx.beginPath(); ctx.rect(px, py, pw, ph); ctx.clip();
      ctx.strokeStyle = col; ctx.globalAlpha = 0.55; ctx.lineWidth = 1.5;
      const sp = 9;
      ctx.beginPath();
      for (let k = -ph; k < pw + ph; k += sp) {
        ctx.moveTo(px + k, py); ctx.lineTo(px + k + ph, py + ph);
        ctx.moveTo(px + k + ph, py); ctx.lineTo(px + k, py + ph);
      }
      ctx.stroke();
      ctx.restore();
      ctx.globalAlpha = 1; ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.strokeRect(px, py, pw, ph);
      if (inside.length) {
        const label = inside.map((x) => short(x.id)).join(', ');
        const tw = ctx.measureText(label).width;
        ctx.fillStyle = C.card; ctx.fillRect(px + pw / 2 - tw / 2 - 3, py - 15, tw + 6, 13);
        ctx.fillStyle = col; ctx.textAlign = 'center'; ctx.fillText(label, px + pw / 2, py - 5); ctx.textAlign = 'left';
      }
    }
    // 5. U-turn / crossing zones: dashed teal outline + a U-turn arrow, so
    //    they read differently from the yellow box junctions.
    const teal = '#14b8a6';
    for (const z of turns || []) {
      const [px, py, pw, ph] = rect(z.x0, z.y0, z.x1, z.y1);
      ctx.globalAlpha = 0.12; ctx.fillStyle = teal; ctx.fillRect(px, py, pw, ph);
      ctx.globalAlpha = 0.9; ctx.strokeStyle = teal; ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]); ctx.strokeRect(px, py, pw, ph); ctx.setLineDash([]);
      const cx = px + pw / 2, cy = py + ph / 2, rr = Math.min(pw, ph) * 0.22;
      if (rr >= 4) {
        ctx.beginPath(); ctx.arc(cx, cy, rr, Math.PI, 0); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(cx + rr - 4, cy - 1); ctx.lineTo(cx + rr, cy + 5); ctx.lineTo(cx + rr + 4, cy - 1);
        ctx.stroke();
      }
      if (v.view.s > 45) {
        ctx.font = '600 9px system-ui, sans-serif'; ctx.fillStyle = teal; ctx.textAlign = 'center';
        ctx.fillText(z.id, cx, py + ph - 4); ctx.textAlign = 'left';
      }
    }
    ctx.globalAlpha = 1;
  }

  _drawHitboxes(v) {
    const { ctx, colors: C } = v;
    const sc = v.view.s;
    const healthCol = [null, C.warn, C.bad];          // loc_health OK/DEGRADED/LOST
    const live = v.state.robots.filter((r) => r.known && r.status !== 'OFFLINE');
    ctx.font = '600 10px system-ui, sans-serif';
    for (const r of live) {
      const col = robotColor(v.map, r.id);
      // Each robot's share of a pairwise stop distance with an equally
      // uncertain peer: g/2 = 0.06 + sqrt(2)*sigma (+ stale growth).
      const share = HB_G0 / 2 + Math.SQRT2 * hbSigma(r)
        + Math.min(HB_REACH_CAP, HB_V_MAX * hbAge(r)) / 2;
      const spin = hbSpinning(r);
      // Drawn at the pose the robot BROADCAST (not the smoothed icon), so
      // any gap between the two is visible.
      const [px, py] = v.toScreen(r.x, r.y);
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-r.theta);
      const outline = healthCol[r.loc_health] || col;
      ctx.strokeStyle = outline; ctx.lineWidth = 1.5;
      if (spin) {                        // spinning: the circumscribed disc
        ctx.globalAlpha = 0.9; ctx.beginPath(); ctx.arc(0, 0, HB_DISC_R * sc, 0, Math.PI * 2); ctx.stroke();
        ctx.globalAlpha = 0.6; ctx.setLineDash([5, 4]);
        ctx.beginPath(); ctx.arc(0, 0, (HB_DISC_R + share) * sc, 0, Math.PI * 2); ctx.stroke();
      } else {                           // wheel-inclusive safety rect + share
        const hl = HB_HALF_L * sc, hw = HB_HALF_W * sc, m = share * sc;
        ctx.globalAlpha = 0.9; ctx.strokeRect(-hl, -hw, 2 * hl, 2 * hw);
        ctx.globalAlpha = 0.6; ctx.setLineDash([5, 4]);
        ctx.beginPath(); ctx.roundRect(-hl - m, -hw - m, 2 * (hl + m), 2 * (hw + m), m); ctx.stroke();
      }
      ctx.setLineDash([]);
      // body: chassis + wheel bumps, contrasting halo then outline
      const L = HB_BODY_L * sc, W = HB_BODY_W * sc;
      const wl = HB_WHEEL_HALF_L * sc, wy0 = (HB_BODY_W / 2) * sc, wy1 = HB_HALF_W * sc;
      ctx.globalAlpha = 1;
      for (const [w, stroke] of [[3.5, C.card], [1.5, C.ink]]) {
        ctx.lineWidth = w; ctx.strokeStyle = stroke;
        ctx.strokeRect(-L / 2, -W / 2, L, W);
        ctx.strokeRect(-wl, wy0, 2 * wl, wy1 - wy0);
        ctx.strokeRect(-wl, -wy1, 2 * wl, wy1 - wy0);
      }
      // front edge + heading arrow
      ctx.strokeStyle = col; ctx.lineWidth = 3.5;
      ctx.beginPath(); ctx.moveTo(L / 2, -W / 2); ctx.lineTo(L / 2, W / 2); ctx.stroke();
      const tip = L / 2 + Math.max(10, 0.18 * sc);
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(L / 2, 0); ctx.lineTo(tip, 0); ctx.stroke();
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.moveTo(tip + 5, 0); ctx.lineTo(tip - 2, -4); ctx.lineTo(tip - 2, 4); ctx.closePath(); ctx.fill();
      ctx.restore();
      const label = `${short(r.id)} σ ${hbSigma(r).toFixed(2)} m${spin ? ' · spinning' : ''}`;
      const reach = (spin ? HB_DISC_R : Math.hypot(HB_HALF_L, HB_HALF_W)) * sc + share * sc;
      const tw = ctx.measureText(label).width;
      ctx.globalAlpha = 0.85; ctx.fillStyle = C.card;
      ctx.fillRect(px - tw / 2 - 3, py + reach + 2, tw + 6, 13);
      ctx.globalAlpha = 1; ctx.fillStyle = outline;
      ctx.textAlign = 'center'; ctx.fillText(label, px, py + reach + 12); ctx.textAlign = 'left';
    }
    // Pairs close to their stop distance: exact gap vs g_ij between the
    // closest points of the two safety footprints.
    ctx.font = '700 10px system-ui, sans-serif';
    for (let i = 0; i < live.length; i++) {
      for (let j = i + 1; j < live.length; j++) {
        const [gap, pa, pb] = hbGap(live[i], live[j]);
        const g = hbStopDist(live[i], live[j]);
        if (gap > g + HB_SHOW_PAIR_M) continue;
        const col = gap < g ? C.bad : C.warn;
        const [ax, ay] = v.toScreen(pa[0], pa[1]), [bx, by] = v.toScreen(pb[0], pb[1]);
        ctx.strokeStyle = col; ctx.lineWidth = 2.5; ctx.setLineDash([2, 3]);
        ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); ctx.setLineDash([]);
        const label = `gap ${Math.max(0, gap).toFixed(2)} m · stop < ${g.toFixed(2)} m`;
        const tw = ctx.measureText(label).width;
        const mx = (ax + bx) / 2, my = (ay + by) / 2 - 10;
        ctx.fillStyle = C.card; ctx.globalAlpha = 0.9;
        ctx.fillRect(mx - tw / 2 - 4, my - 9, tw + 8, 14);
        ctx.globalAlpha = 1; ctx.fillStyle = col; ctx.textAlign = 'center';
        ctx.fillText(label, mx, my + 2); ctx.textAlign = 'left';
      }
    }
    ctx.globalAlpha = 1;
  }

  _drawZoneLocks(v) {
    const { ctx } = v;
    const geom = new Map(v.map.zones.map((z) => [z.id, z]));
    ctx.font = '600 11px system-ui, sans-serif';
    for (const [zid, z] of Object.entries(this.p2p.zones)) {
      const g = geom.get(zid);
      if (!g) continue;
      const [r0, c0, r1, c1] = g.rect;
      const x = v.world.x0 + c0 * v.res, y = v.world.y0 + r0 * v.res;
      const w = (c1 - c0 + 1) * v.res, h = (r1 - r0 + 1) * v.res;
      const [px, py] = v.toScreen(x, y + h);
      const pw = w * v.view.s, ph = h * v.view.s;
      const holder = z.held[0];
      if (holder) {
        const col = robotColor(v.map, holder);
        ctx.fillStyle = col; ctx.globalAlpha = 0.16; ctx.fillRect(px, py, pw, ph);
        ctx.globalAlpha = 0.9; ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.strokeRect(px, py, pw, ph);
      } else {
        ctx.globalAlpha = 0.9; ctx.strokeStyle = v.colors.warn; ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]); ctx.strokeRect(px, py, pw, ph); ctx.setLineDash([]);
      }
      const peers = Math.max(1, v.map.robots.length - 1);
      // "zone" up front: these are shared mutex areas, not robot bodies.
      let label = holder ? `🔒 zone ${zid} · ${z.held.map(short).join(', ')}` : `zone ${zid}`;
      if (z.waiting.length) {
        label += '  ⏳ ' + z.waiting.map((w) => `${short(w.robot)} ${w.granted_by.length}/${peers}`).join(', ');
      }
      const tw = ctx.measureText(label).width;
      ctx.globalAlpha = 0.9; ctx.fillStyle = v.colors.card;
      ctx.fillRect(px + 3, py + 3, tw + 8, 16);
      ctx.globalAlpha = 1; ctx.fillStyle = holder ? robotColor(v.map, holder) : v.colors.warn;
      ctx.fillText(label, px + 7, py + 15);
    }
    ctx.globalAlpha = 1;
  }

  _drawBlocked(v) {
    const { ctx } = v;
    const s = v.res * v.view.s;
    for (const [x, y, ttl] of this.p2p.blocked) {
      const [px, py] = v.toScreen(x, y);
      ctx.globalAlpha = Math.min(0.8, 0.25 + ttl / 30);
      ctx.fillStyle = v.colors.bad;
      ctx.fillRect(px - s / 2, py - s / 2, s, s);
    }
    ctx.globalAlpha = 1;
  }

  _drawIntents(v) {
    const { ctx } = v;
    const r = Math.max(1.6, 0.035 * v.view.s);
    ctx.font = '600 9px system-ui, sans-serif';
    for (const [rid, it] of Object.entries(this.p2p.intents)) {
      if (!it.cells.length || it.age_s > 5) continue;
      const col = robotColor(v.map, rid);
      ctx.fillStyle = col;
      let bucket = 0;
      for (const [x, y, eta] of it.cells) {
        const [px, py] = v.toScreen(x, y);
        ctx.globalAlpha = eta == null ? 0.45 : Math.max(0.12, 0.85 * (1 - eta / 8));
        ctx.beginPath(); ctx.arc(px, py, r, 0, Math.PI * 2); ctx.fill();
        // ETA ticks every 2 s along the shared plan
        if (eta != null && Math.floor(eta / 2) > bucket) {
          bucket = Math.floor(eta / 2);
          ctx.globalAlpha = 0.85;
          ctx.fillText(`+${bucket * 2}s`, px + 5, py - 4);
        }
      }
    }
    ctx.globalAlpha = 1;
  }

  _linkMap() {
    const m = new Map();
    for (const l of this.p2p.links) m.set(`${l.src}>${l.dst}`, l);
    return m;
  }

  _drawLinks(v, t) {
    const { ctx, colors: C } = v;
    const links = this._linkMap();
    const ids = v.map.robots;
    const freshCol = [C.ok, C.warn, C.bad];
    ctx.font = '600 10px system-ui, sans-serif';
    for (let i = 0; i < ids.length; i++) {
      for (let j = i + 1; j < ids.length; j++) {
        const a = ids[i], b = ids[j];
        const pa = v.robotScreen(a), pb = v.robotScreen(b);
        if (!pa || !pb) continue;
        const ab = links.get(`${a}>${b}`), ba = links.get(`${b}>${a}`);
        const worst = Math.max(FRESH_RANK[ab?.fresh] ?? 2, FRESH_RANK[ba?.fresh] ?? 2);
        const len = Math.hypot(pb[0] - pa[0], pb[1] - pa[1]) || 1;
        const nx = -(pb[1] - pa[1]) / len, ny = (pb[0] - pa[0]) / len;
        ctx.strokeStyle = freshCol[worst]; ctx.lineWidth = 2; ctx.globalAlpha = 0.5;
        ctx.setLineDash(worst === 2 ? [3, 6] : worst === 1 ? [9, 5] : []);
        ctx.beginPath(); ctx.moveTo(...pa); ctx.lineTo(...pb); ctx.stroke();
        ctx.setLineDash([]);
        // Packets flowing both ways, offset to either side of the line.
        for (const [s, d, l, side] of [[a, b, ab, 1], [b, a, ba, -1]]) {
          if (!l || l.fresh === 'DEAD') continue;
          const ps = v.robotScreen(s), pd = v.robotScreen(d);
          ctx.fillStyle = robotColor(v.map, s); ctx.globalAlpha = 0.9;
          for (let k = 0; k < 3; k++) {
            const f = ((t / 1000) * 0.45 + k / 3) % 1;
            const [x, y] = lerp(ps, pd, f);
            ctx.beginPath(); ctx.arc(x + nx * 4 * side, y + ny * 4 * side, 2.4, 0, Math.PI * 2); ctx.fill();
          }
        }
        // Last-heard ages at the midpoint: "a→b ms · b→a ms"
        const fmt = (l) => (l ? `${l.age_ms} ms` : '—');
        const label = `${fmt(ab)} ⇄ ${fmt(ba)}`;
        const [mx, my] = lerp(pa, pb, 0.5);
        const tw = ctx.measureText(label).width;
        ctx.globalAlpha = 0.85; ctx.fillStyle = C.card;
        ctx.fillRect(mx - tw / 2 - 4, my - 8, tw + 8, 15);
        ctx.globalAlpha = 1; ctx.fillStyle = freshCol[worst];
        ctx.textAlign = 'center'; ctx.fillText(label, mx, my + 3); ctx.textAlign = 'left';
      }
    }
    ctx.globalAlpha = 1;
  }

  _drawFlights(v) {
    const { ctx } = v;
    const now = performance.now();
    this.flights = this.flights.filter((f) => now - f.t0 < FLIGHT_MS);
    for (const f of this.flights) {
      const ps = v.robotScreen(f.src), pd = v.robotScreen(f.dst);
      if (!ps || !pd) continue;
      const k = (now - f.t0) / FLIGHT_MS;
      const e = k < 0.5 ? 2 * k * k : 1 - ((-2 * k + 2) ** 2) / 2;     // ease in-out
      const [x, y] = lerp(ps, pd, e);
      ctx.globalAlpha = 1 - k * 0.6;
      ctx.fillStyle = KIND_COLOR[f.kind] || '#64748b';
      ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  _drawWaits(v) {
    const { ctx, colors: C } = v;
    ctx.font = '700 10px system-ui, sans-serif';
    for (const [rid, p] of Object.entries(this.p2p.permits)) {
      if (!p.blocking || !(HOLD.has(p.action) || p.action === 'SLOW') || p.age_s > 3) continue;
      const pa = v.robotScreen(rid), pb = v.robotScreen(p.blocking);
      if (!pa || !pb) continue;
      const len = Math.hypot(pb[0] - pa[0], pb[1] - pa[1]);
      if (len < 30) continue;
      const ux = (pb[0] - pa[0]) / len, uy = (pb[1] - pa[1]) / len;
      const s = [pa[0] + ux * 20, pa[1] + uy * 20], e = [pb[0] - ux * 22, pb[1] - uy * 22];
      const col = HOLD.has(p.action) ? C.bad : C.warn;
      ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 2.5; ctx.setLineDash([7, 4]);
      ctx.beginPath(); ctx.moveTo(...s); ctx.lineTo(...e); ctx.stroke(); ctx.setLineDash([]);
      ctx.beginPath();
      ctx.moveTo(...e);
      ctx.lineTo(e[0] - ux * 10 + uy * 5, e[1] - uy * 10 - ux * 5);
      ctx.lineTo(e[0] - ux * 10 - uy * 5, e[1] - uy * 10 + ux * 5);
      ctx.closePath(); ctx.fill();
      const [mx, my] = lerp(s, e, 0.5);
      const label = `${short(rid)} ${p.action === 'SLOW' ? 'slows for' : 'waits for'} ${short(p.blocking)}`;
      const tw = ctx.measureText(label).width;
      ctx.fillStyle = C.card; ctx.globalAlpha = 0.9;
      ctx.fillRect(mx - tw / 2 - 4, my - 18, tw + 8, 14);
      ctx.globalAlpha = 1; ctx.fillStyle = col; ctx.textAlign = 'center';
      ctx.fillText(label, mx, my - 7); ctx.textAlign = 'left';
    }
  }

  // ----------------------------------------------------------------- panel
  renderPanel() {
    const p = this.p2p;
    const { els } = this;
    if (p) {
      const rmw = p.transport.rmw;
      els.transport.textContent = `${RMW_NAMES[rmw] || rmw} · domain ${p.transport.domain_id}`
        + (p.transport.zenoh_session_config ? ' · zenoh config' : '');
      els.transport.title = `RMW: ${rmw}`;
    }
    if (this.pane === 'feed') {
      if (!this.paused) this._renderFeed();
    } else if (p) {
      this._renderNet(p);
    }
  }

  _who(id) {
    if (!id || id === '-') return '';
    if (id === '*') return '<span class="muted">all peers</span>';
    const isRobot = this.view.map.robots.includes(id);
    return isRobot ? `<b style="color:${robotColor(this.view.map, id)}">${esc(short(id))}</b>`
      : `<b>${esc(id)}</b>`;
  }

  _renderFeed() {
    const rows = [];
    for (let i = this.feed.length - 1; i >= 0 && rows.length < FEED_ROWS; i--) {
      const e = this.feed[i];
      if (!this.filters.has(e.kind)) continue;
      const time = new Date(e.t * 1000).toLocaleTimeString([], { hour12: false });
      const route = e.dst && e.dst !== '-' ? `${this._who(e.src)} → ${this._who(e.dst)}` : this._who(e.src);
      rows.push(`<div class="ev"><span class="ev-t">${time}</span>
        <span class="kind" style="--c:${KIND_COLOR[e.kind]}">${e.kind}</span>
        <span class="ev-body"><span class="ev-who">${route}</span> ${esc(e.text)}</span></div>`);
    }
    this.els.feed.innerHTML = rows.join('')
      || '<div class="empty muted">No P2P messages yet for these filters.</div>';
  }

  _renderNet(p) {
    const ids = this.view.map.robots;
    const links = this._linkMap();
    const freshVar = { FRESH: 'var(--ok)', SUSPECT: 'var(--warn)', DEAD: 'var(--bad)' };
    let html = `<h3>Who hears whom <span class="muted">(row = listener, column = sender)</span></h3>
      <table class="matrix"><tr><th></th>${ids.map((s) => `<th>${this._who(s)}</th>`).join('')}</tr>`;
    for (const dst of ids) {
      html += `<tr><th>${this._who(dst)}</th>`;
      for (const src of ids) {
        if (src === dst) { html += '<td class="muted">—</td>'; continue; }
        const l = links.get(`${src}>${dst}`);
        if (!l) { html += '<td class="muted">no data</td>'; continue; }
        const extra = [l.loss > 0 ? `${(l.loss * 100).toFixed(1)}% loss` : '',
          l.plan_unknown ? 'plan?' : '', l.intent_seq != null ? `plan #${l.intent_seq}` : '']
          .filter(Boolean).join(' · ');
        html += `<td><span class="dot" style="background:${freshVar[l.fresh] || 'var(--idle)'}"></span>
          ${esc(l.fresh || '?')} <span class="muted">${l.age_ms} ms</span>
          <div class="sub muted">${esc(extra)}</div></td>`;
      }
      html += '</tr>';
    }
    html += '</table>';

    const st = p.stats || {};
    const robots = st.robots || {};
    const maxK = Math.max(0.01, ...Object.values(robots).map((r) => r.p2p_kbps));
    html += `<h3>Each robot transmits <span class="muted">(last ${st.window_s || 0}s)</span></h3><div class="tx">`;
    for (const rid of ids) {
      const r = robots[rid] || { p2p_hz: 0, p2p_kbps: 0, local_kbps: 0 };
      html += `<div class="txrow">${this._who(rid)}
        <div class="bar"><i style="width:${(100 * r.p2p_kbps / maxK).toFixed(0)}%;background:${robotColor(this.view.map, rid)}"></i></div>
        <span>${r.p2p_hz} msg/s · <b>${r.p2p_kbps} kB/s</b></span></div>`;
    }
    const fan = Math.max(1, ids.length - 1);
    html += `</div><p class="note muted">Fleet bus total ${st.p2p_kbps || 0} kB/s as sent. With unicast fan-out
      (Zenoh peer mode, DDS without multicast) each message goes to ${fan} peer${fan > 1 ? 's' : ''},
      ≈ ${((st.p2p_kbps || 0) * fan).toFixed(1)} kB/s on air.</p>`;

    html += `<h3>Topics</h3><table class="topics"><tr><th>Topic</th><th></th><th>msg/s</th><th>kB/s</th><th>avg B</th></tr>`;
    for (const t of st.topics || []) {
      html += `<tr><td class="mono">${esc(t.topic)}</td><td><span class="chip" style="--c:${t.scope === 'p2p'
        ? 'var(--accent)' : 'var(--idle)'}">${t.scope}</span></td><td>${t.hz}</td><td>${t.kbps}</td><td>${t.avg_bytes}</td></tr>`;
    }
    html += '</table>';
    this.els.netPane.innerHTML = html;
  }
}
