// Side panels (robots, tasks), header summary and toasts. Pure rendering
// from the latest stream snapshot.
import { robotColor } from './map.js';

export function esc(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

const STATUS_COLOR = {
  MOVING: 'var(--accent)', WAITING: 'var(--warn)', YIELDING: 'var(--warn)',
  CHARGING: 'var(--ok)', FAULT: 'var(--bad)', IDLE: 'var(--idle)', OFFLINE: 'var(--idle)',
  PENDING: 'var(--idle)', ASSIGNED: 'var(--accent)', TO_PICKUP: 'var(--pick)',
  TO_DROP: 'var(--drop)', DONE: 'var(--ok)', ABORTING: 'var(--warn)', ABORTED: 'var(--bad)',
};
const STATUS_LABEL = { TO_PICKUP: 'to pickup', TO_DROP: 'to drop' };

// Finished = shown under "Done". Abortable = the robot has not loaded yet.
export const FINISHED = new Set(['DONE', 'ABORTED']);
export const ABORTABLE = new Set(['PENDING', 'ASSIGNED', 'TO_PICKUP']);

function chip(status) {
  const label = STATUS_LABEL[status] || status.toLowerCase();
  return `<span class="chip" style="--c:${STATUS_COLOR[status] || 'var(--idle)'}">${esc(label)}</span>`;
}

function fmtAge(s) {
  if (s < 60) return `${Math.round(s)}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${Math.round(s % 60)}s`;
  return `${Math.floor(s / 3600)}h ${Math.floor((s % 3600) / 60)}m`;
}

function robotActivity(r) {
  if (!r.known) return 'Waiting for the robot to come online…';
  if (r.status === 'OFFLINE') return `No state for ${(r.age_ms / 1000).toFixed(1)} s`;
  if (r.loc_health === 2) {
    return r.reloc_attempts
      ? `Localization lost — re-seeded by odometry (attempt ${r.reloc_attempts}/3), waiting to re-converge`
      : 'Localization lost — motion stopped';
  }
  if (r.status === 'FAULT') return r.alive ? 'Fault' : 'Fault / not localised — motion stopped';
  if (r.task_id) {
    const phase = r.task_phase === 'DROPOFF' ? 'heading to drop' : 'heading to pickup';
    return `${esc(r.task_id)} · ${phase}`;
  }
  const err = r.dock_err_m != null ? ` · ${(r.dock_err_m * 100).toFixed(0)} cm, ${Math.abs(r.dock_err_deg).toFixed(1)}° off` : '';
  if (r.dock_phase === 'DOCKED') {
    const warn = r.dock_misaligned ? ` — docked misaligned${err}` : '';
    return (r.charge_hold ? 'Charging at dock — low battery, resumes work at 80 %'
      : r.battery >= 99.5 ? 'Parked at dock · full' : 'Charging at dock') + warn;
  }
  if (r.dock_phase === 'ALIGNING') {
    return r.dock_hold ? `Aligning on dock — waiting for a peer to clear before turning${err}`
      : `Aligning on dock${err}${r.dock_retries ? ` · retry ${r.dock_retries}` : ''}`;
  }
  if (r.dock_phase === 'BACKING_OFF') {
    return `Backing off to re-approach the dock (retry ${r.dock_retries})${err}`;
  }
  if (r.dock_phase === 'RETURNING') {
    return r.charge_hold ? 'Low battery — returning to dock' : 'No tasks left — returning to dock';
  }
  return 'Idle';
}

// Legal lane-crossing manoeuvres under strict lanes (coordinator flags).
function manoeuvre(r) {
  if (!r.known || r.status === 'OFFLINE') return '';
  if (r.uturn_active) return ' <span class="chip move" title="Gated U-turn across into the opposite lane (both lanes checked clear)">U-TURN</span>';
  if (r.overtake_active) return ' <span class="chip move" title="Gated pass of a robot frozen in my lane, using the opposite lane while it is clear">OVERTAKING</span>';
  if (r.reverse_active) return ' <span class="chip move" title="Retreat / make-way driving against its own lane direction">REVERSING</span>';
  return '';
}

// One line on what the coordination layer is doing to this robot, from its
// latest motion permit (only when it is not a plain GO).
function permitLine(p) {
  if (!p || p.action === 'GO' || p.age_s > 3) return '';
  const hold = ['STOP', 'YIELD', 'REROUTE'].includes(p.action);
  const who = p.blocking ? ` for ${esc(p.blocking)}` : '';
  const zone = p.zone ? ` · zone ${esc(p.zone)}` : '';
  const why = p.reason ? ` — ${esc(p.reason)}` : '';
  return `<div class="detail permit${hold ? ' hold' : ''}">${esc(p.action)}${who}${zone}${why}</div>`;
}

export function renderRobots(el, map, robots, permits = {}) {
  el.innerHTML = robots.map((r) => {
    const batt = r.known ? r.battery : 0;
    const cls = [batt < 30 ? 'low' : batt < 50 ? 'mid' : '',
      r.status === 'CHARGING' && batt < 99.5 ? 'charging' : ''].join(' ');
    return `<div class="robot">
      <span class="swatch" style="background:${robotColor(map, r.id)}"></span>
      <span class="name">${esc(r.id)}</span>${chip(r.status)}${manoeuvre(r)}
      <div class="battery"><div class="bar ${cls}"><i style="width:${batt.toFixed(1)}%"></i></div>
        <span>${r.known ? `${batt.toFixed(0)} %` : '—'}</span></div>
      <div class="detail">${robotActivity(r)}</div>
      ${r.known && r.status !== 'OFFLINE' ? permitLine(permits[r.id]) : ''}
    </div>`;
  }).join('');
}

function place(view, xy, slot) {
  if (!xy) return '?';
  const station = view.stationNear(xy[0], xy[1]);
  if (station && slot) return `${station} · ${slot}`;
  return station || slot || `(${xy[0].toFixed(1)}, ${xy[1].toFixed(1)})`;
}

// Deadline countdown (generator tasks carry one; web tasks have none).
function deadline(t) {
  const left = t.deadline_left_s;
  if (left == null) return '';
  const col = left < 0 ? 'var(--bad)' : left < 30 ? 'var(--warn)' : 'var(--muted)';
  const text = left < 0 ? `overdue ${fmtAge(-left)}` : `⏱ ${fmtAge(left)} left`;
  return `<span class="src" style="color:${col}">${text}</span>`;
}

// armed = {id, until}: the Abort button the user clicked once (needs a second
// click within a few seconds); kept outside so 4 Hz re-renders don't reset it.
function abortCell(t, armed) {
  if (t.status === 'ABORTING') return '<span class="muted">aborting…</span>';
  if (!ABORTABLE.has(t.status)) return '';
  const hot = armed && armed.id === t.id && performance.now() < armed.until;
  return `<button class="abort${hot ? ' confirm' : ''}" data-abort="${esc(t.id)}"
    title="Abort this task (allowed until pickup)">${hot ? 'Confirm' : 'Abort'}</button>`;
}

export function renderTasks(el, view, tasks, tab, armed = null) {
  const rows = tasks.filter((t) => (tab === 'done') === FINISHED.has(t.status));
  if (!rows.length) {
    el.innerHTML = `<tr><td colspan="6" class="empty">${tab === 'done'
      ? 'No completed tasks yet.' : 'No active tasks. Press <b>+ New task</b> (or N) and click two points on the map.'}</td></tr>`;
    return;
  }
  el.innerHTML = rows.map((t) => `<tr data-task="${esc(t.id)}">
    <td class="id">${esc(t.id)}<span class="src">${t.source === 'web' ? 'web app' : 'generator'} · ${fmtAge(t.age_s)}</span>${deadline(t)}</td>
    <td>${esc(place(view, t.pickup, t.pickup_slot))} → ${esc(place(view, t.drop, t.drop_slot))}</td>
    <td>${t.priority}</td>
    <td>${chip(t.status)}</td>
    <td>${t.robot ? `<span style="color:${robotColor(view.map, t.robot)};font-weight:600">${esc(t.robot.replace('robot_', 'R'))}</span>` : '—'}</td>
    <td>${abortCell(t, armed)}</td>
  </tr>`).join('');
}

export function renderSummary(el, robots, tasks, links = null) {
  const online = robots.filter((r) => r.known && r.status !== 'OFFLINE').length;
  const active = tasks.filter((t) => !FINISHED.has(t.status)).length;
  const pending = tasks.filter((t) => t.status === 'PENDING').length;
  const done = tasks.length - active;
  el.innerHTML = `<span>${online}/${robots.length} robots online</span>
    <span>${active} active${pending ? ` (${pending} waiting)` : ''}</span><span>${done} done</span>`
    + (links && links.total ? `<span>P2P links ${links.fresh}/${links.total} fresh</span>` : '');
}

export function toast(container, text, kind = 'ok', ms = 4000) {
  const div = document.createElement('div');
  div.className = `toast ${kind}`;
  div.textContent = text;
  container.appendChild(div);
  setTimeout(() => div.remove(), ms);
}
