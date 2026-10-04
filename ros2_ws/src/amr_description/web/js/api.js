// HTTP + Server-Sent Events client for the fleet_webapp node.

export async function fetchMap() {
  const r = await fetch('/api/map', { cache: 'no-store' });
  if (!r.ok) throw new Error(`map request failed (HTTP ${r.status})`);
  return r.json();
}

// onStatus receives 'live' | 'reconnecting'. EventSource retries by itself.
export function openStream(onData, onStatus) {
  const es = new EventSource('/api/stream');
  es.onopen = () => onStatus('live');
  es.onmessage = (e) => {
    let data;
    try { data = JSON.parse(e.data); } catch { return; }
    onData(data);
  };
  es.onerror = () => onStatus('reconnecting');
  return es;
}

export async function createTask(pickup, drop, priority) {
  const r = await fetch('/api/tasks', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ pickup, drop, priority }),
  });
  const body = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(body.error || `HTTP ${r.status}`);
  return body;
}

// Ask the fleet to drop a task. Refused (409) once the robot has loaded.
export async function abortTask(taskId) {
  const r = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/abort`, { method: 'POST' });
  const body = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(body.error || `HTTP ${r.status}`);
  return body;
}
