// Entry point: load the layout, open the live stream, wire the task draft.
import { abortTask, createTask, fetchMap, openStream } from './api.js';
import { MapView } from './map.js';
import { P2PView } from './p2p.js';
import { FINISHED, renderRobots, renderSummary, renderTasks, toast } from './ui.js';

const $ = (id) => document.getElementById(id);
const el = {
  canvas: $('map'), wrap: $('mapWrap'), loading: $('loading'), conn: $('conn'),
  summary: $('summary'), robots: $('robots'), tasks: $('tasks'), toasts: $('toasts'),
  newTask: $('newTask'), draft: $('draft'), hint: $('draftHint'), priority: $('priority'),
  confirm: $('confirm'), cancel: $('cancel'), cursor: $('cursor'), fit: $('fit'),
  pickSlot: $('pickSlot'), dropSlot: $('dropSlot'), slotList: $('slotList'),
  nActive: $('nActive'), nDone: $('nDone'), sideBody: $('sideBody'),
  cStatus: $('cStatus'), cNetwork: $('cNetwork'), cTasks: $('cTasks'),
};
const PANES = ['status', 'network', 'tasks'];

let view = null;
let p2p = null;
let latest = null;
let tab = 'active';
let pane = 'status';
let armed = null;            // {id, until}: Abort clicked once, awaiting confirm
let lastPanel = 0;

const HINTS = {
  pickup: 'Click the <span class="p">PICKUP</span> point',
  drop: 'Now click the <span class="d">DROP</span> point',
  confirm: 'Ready — set priority and send (click again to move the drop)',
};

// ---------------------------------------------------------------- panels
function renderPanels(force = false) {
  if (!latest || !view) return;
  const now = performance.now();
  if (!force && now - lastPanel < 250) return;
  lastPanel = now;
  const links = p2p && p2p.linkSummary();
  renderSummary(el.summary, latest.robots, latest.tasks, links);
  // Tab badges are always live; only the visible pane is re-rendered.
  const online = latest.robots.filter((r) => r.known && r.status !== 'OFFLINE').length;
  const done = latest.tasks.filter((t) => FINISHED.has(t.status)).length;
  el.cStatus.textContent = `${online}/${latest.robots.length}`;
  el.cNetwork.textContent = links && links.total ? `${links.fresh}/${links.total}` : '';
  el.cTasks.textContent = String(latest.tasks.length - done);
  el.nActive.textContent = latest.tasks.length - done;
  el.nDone.textContent = done;
  if (pane === 'status') {
    renderRobots(el.robots, view.map, latest.robots, (latest.p2p && latest.p2p.permits) || {});
  } else if (pane === 'tasks') {
    renderTasks(el.tasks, view, latest.tasks, tab, armed);
  } else if (p2p) {
    p2p.renderPanel();
  }
}

function showPane(name) {
  pane = PANES.includes(name) ? name : 'status';
  document.querySelectorAll('.stab').forEach((b) => {
    const on = b.dataset.pane === pane;
    b.classList.toggle('active', on);
    b.setAttribute('aria-selected', String(on));
  });
  for (const p of PANES) $(`pane-${p}`).hidden = p !== pane;
  el.sideBody.scrollTop = 0;
  try { localStorage.setItem('sidePane', pane); } catch { /* private mode */ }
  renderPanels(true);
}

async function onAbort(id) {
  if (!armed || armed.id !== id || performance.now() > armed.until) {
    armed = { id, until: performance.now() + 3000 };     // first click arms
    renderPanels(true);
    return;
  }
  armed = null;
  try {
    await abortTask(id);
    toast(el.toasts, `Abort sent for ${id}.`, 'warn');
  } catch (err) {
    toast(el.toasts, `Cannot abort ${id}: ${err.message}`, 'err', 6000);
  }
  renderPanels(true);
}

function setConn(state) {
  el.conn.dataset.state = state;
  el.conn.textContent = state === 'live' ? 'Live' : state === 'down' ? 'Offline' : 'Reconnecting…';
}

// ----------------------------------------------------------- task draft
function startDraft() {
  if (!view) return;
  view.draft = { mode: 'pickup', pickup: null, drop: null, cursor: null,
    pickupSlot: null, dropSlot: null };
  el.pickSlot.value = ''; el.dropSlot.value = '';
  el.pickSlot.classList.remove('bad'); el.dropSlot.classList.remove('bad');
  el.draft.hidden = false;
  el.newTask.hidden = true;
  el.confirm.disabled = true;
  el.hint.innerHTML = HINTS.pickup;
  el.wrap.classList.add('picking');
}

function endDraft() {
  if (view) view.draft = null;
  el.draft.hidden = true;
  el.newTask.hidden = false;
  el.wrap.classList.remove('picking');
}

// Put the pickup or drop (whichever is next) at a point; `slot` = rack slot
// label when the point is that slot's approach pose.
function setDraftPoint(kind, pt, slot) {
  const d = view.draft;
  const other = kind === 'pickup' ? d.drop : d.pickup;
  if (other && Math.hypot(pt[0] - other[0], pt[1] - other[1]) < 0.5) {
    toast(el.toasts, 'Pickup and drop must be at least 0.5 m apart.', 'err');
    return false;
  }
  d[kind] = pt;
  d[`${kind}Slot`] = slot || null;
  (kind === 'pickup' ? el.pickSlot : el.dropSlot).value = slot || '';
  d.mode = !d.pickup ? 'pickup' : !d.drop ? 'drop' : 'confirm';
  el.confirm.disabled = d.mode !== 'confirm';
  el.hint.innerHTML = HINTS[d.mode];
  return true;
}

function draftClick(x, y) {
  const d = view.draft;
  const kind = d.mode === 'pickup' ? 'pickup' : 'drop';
  // Clicking a rack section (or next to it) picks that slot: the robot
  // stops at the slot's approach point in the aisle.
  const sl = view.slotAt(x, y);
  if (sl) {
    setDraftPoint(kind, [sl.approach[0], sl.approach[1]], sl.label);
    return;
  }
  if (view.placeability(x, y) === 'blocked') {
    toast(el.toasts, 'That point is inside or right next to a rack/wall — pick a spot in an aisle or a rack section.', 'err');
    return;
  }
  setDraftPoint(kind, [x, y], null);
}

function typedSlot(kind, input) {
  const raw = input.value.trim().toUpperCase().replace(/\s+/g, '');
  if (!raw) return;
  const sl = view.slotByLabel.get(raw);
  input.classList.toggle('bad', !sl);
  if (!sl) {
    toast(el.toasts, `No rack slot "${input.value.trim()}" — use <side>R<section>, e.g. 1R6 (sides 1–12, sections 1–20).`, 'err');
    return;
  }
  setDraftPoint(kind, [sl.approach[0], sl.approach[1]], sl.label);
}

async function sendDraft() {
  const d = view && view.draft;
  if (!d || !d.pickup || !d.drop) return;
  el.confirm.disabled = true;
  try {
    const end = (pt, slot) => (slot ? { slot } : { x: pt[0], y: pt[1] });
    const res = await createTask(end(d.pickup, d.pickupSlot), end(d.drop, d.dropSlot),
      Number(el.priority.value));
    const route = res.pickup_slot || res.drop_slot
      ? ` (${res.pickup_slot || 'point'} → ${res.drop_slot || 'point'})` : '';
    toast(el.toasts, `${res.task_id}${route} sent — the robots are bidding for it.`);
    if (res.warning) toast(el.toasts, res.warning, 'warn', 6000);
    endDraft();
  } catch (err) {
    toast(el.toasts, `Task rejected: ${err.message}`, 'err', 6000);
    el.confirm.disabled = false;
  }
}

// --------------------------------------------------------- map pointer
function wirePointer() {
  const c = el.canvas;
  let press = null;
  const local = (e) => {
    const r = c.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  };

  c.addEventListener('pointerdown', (e) => {
    const [px, py] = local(e);
    press = { px, py, lx: px, ly: py, drag: false };
    c.setPointerCapture(e.pointerId);
  });
  c.addEventListener('pointermove', (e) => {
    const [px, py] = local(e);
    const [x, y] = view.toWorld(px, py);
    const fit = view.placeability(x, y);
    const sl = view.slotAt(x, y);
    view.hoverSlot = sl ? sl.label : null;
    el.cursor.textContent = `x ${x.toFixed(2)}  y ${y.toFixed(2)}`
      + (sl ? ` · slot ${sl.label}` : fit === 'blocked' ? ' · blocked' : '');
    if (view.draft) view.draft.cursor = [x, y, fit];
    if (!press) return;
    if (!press.drag && Math.hypot(px - press.px, py - press.py) > 4) press.drag = true;
    if (press.drag) {
      view.pan(px - press.lx, py - press.ly);
      press.lx = px; press.ly = py;
    }
  });
  c.addEventListener('pointerup', (e) => {
    if (press && !press.drag && view.draft) {
      const [px, py] = local(e);
      draftClick(...view.toWorld(px, py));
    }
    press = null;
  });
  c.addEventListener('pointerleave', () => {
    el.cursor.textContent = '';
    view.hoverSlot = null;
    if (view.draft) view.draft.cursor = null;
  });
  c.addEventListener('wheel', (e) => {
    e.preventDefault();
    const [px, py] = local(e);
    view.zoomAt(px, py, Math.exp(-e.deltaY * 0.0015));
  }, { passive: false });
  c.addEventListener('dblclick', () => { if (!view.draft) view.fit(); });
}

function wireControls() {
  el.newTask.addEventListener('click', startDraft);
  for (const [kind, input] of [['pickup', el.pickSlot], ['drop', el.dropSlot]]) {
    input.addEventListener('change', () => typedSlot(kind, input));
    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); typedSlot(kind, input); }
      if (e.key === 'Escape') endDraft();
    });
  }
  el.cancel.addEventListener('click', endDraft);
  el.confirm.addEventListener('click', sendDraft);
  el.fit.addEventListener('click', () => view && view.fit());
  document.addEventListener('keydown', (e) => {
    if (e.target.closest('input, select, textarea')) return;
    if (e.key === 'Escape') endDraft();
    else if ((e.key === 'n' || e.key === 'N') && !view?.draft) startDraft();
    else if (e.key === 'Enter' && !el.confirm.disabled && view?.draft) sendDraft();
  });
  const taskTabs = document.querySelectorAll('#taskTabs .tab');
  taskTabs.forEach((b) => b.addEventListener('click', () => {
    tab = b.dataset.tab;
    taskTabs.forEach((x) => x.classList.toggle('active', x === b));
    renderPanels(true);
  }));
  document.querySelectorAll('.stab').forEach((b) => b.addEventListener('click', () => showPane(b.dataset.pane)));
  // pointerdown, not click: the table re-renders 4x/s and a click needs the
  // same button node under both mousedown and mouseup.
  el.tasks.addEventListener('pointerdown', (e) => {
    const b = e.target.closest('button[data-abort]');
    if (b) { e.preventDefault(); onAbort(b.dataset.abort); }
  });
  el.tasks.addEventListener('mouseover', (e) => {
    const tr = e.target.closest('tr[data-task]');
    if (view) view.hoverTask = tr ? tr.dataset.task : null;
  });
  el.tasks.addEventListener('mouseleave', () => { if (view) view.hoverTask = null; });
}

// ------------------------------------------------------------------ boot
async function boot() {
  wireControls();
  let saved = null;
  try { saved = localStorage.getItem('sidePane'); } catch { /* private mode */ }
  showPane(saved || 'status');
  let map;
  for (;;) {
    try { map = await fetchMap(); break; } catch (err) {
      el.loading.textContent = `Waiting for the fleet webapp node… (${err.message})`;
      setConn('down');
      await new Promise((r) => setTimeout(r, 2000));
    }
  }
  el.loading.remove();
  view = new MapView(el.canvas, map);
  el.slotList.innerHTML = (map.slots || []).map((sl) => `<option value="${sl.label}">`).join('');
  p2p = new P2PView(view, {
    layers: $('layers'), filters: $('feedFilters'), feed: $('feed'),
    feedPane: $('feedPane'), netPane: $('netPane'), transport: $('transport'),
    tabs: [...document.querySelectorAll('#p2pTabs .tab')],
  });
  wirePointer();
  openStream((data) => {
    latest = data;
    view.setState(data);
    p2p.update(data);
    renderPanels();
  }, setConn);
}

boot();
