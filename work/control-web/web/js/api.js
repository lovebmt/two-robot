// REST + WebSocket tới Control Server.

function fmtErr(d) {
  if (!d) return '';
  const x = d.detail ?? d;
  if (typeof x === 'string') return x;
  if (x.errors) {
    return Array.isArray(x.errors)
      ? x.errors.join(' · ')
      : Object.entries(x.errors).map(([k, v]) => `${k}: ${v}`).join(' · ');
  }
  if (x.err) return x.err;
  return JSON.stringify(x);
}

export async function api(method, url, body) {
  const opt = { method, headers: {} };
  if (body !== undefined) {
    opt.headers['Content-Type'] = 'application/json';
    opt.body = JSON.stringify(body);
  }
  const r = await fetch(url, opt);
  let data = null;
  try { data = await r.json(); } catch { /* không có body */ }
  if (!r.ok) {
    const e = new Error(fmtErr(data) || `${r.status} ${r.statusText}`);
    e.status = r.status;
    e.data = data;
    throw e;
  }
  return data;
}

export function connectWS(onMsg, onState) {
  let ws = null;
  let retry = 500;
  const open = () => {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${proto}://${location.host}/ws/ui`);
    ws.onopen = () => { retry = 500; onState(true); };
    ws.onmessage = (ev) => {
      try { onMsg(JSON.parse(ev.data)); } catch (e) { console.error(e); }
    };
    ws.onclose = () => {
      onState(false);
      setTimeout(open, retry);
      retry = Math.min(retry * 2, 5000);
    };
  };
  open();
}

export function toast(msg, kind = '') {
  const box = document.getElementById('toasts');
  const el = document.createElement('div');
  el.className = `toast ${kind}`;
  el.textContent = msg;
  box.appendChild(el);
  setTimeout(() => el.remove(), kind === 'err' ? 6000 : 3000);
}

export async function guard(fn, okMsg) {
  try {
    const r = await fn();
    if (okMsg) toast(okMsg, 'ok');
    return r;
  } catch (e) {
    toast(e.message, 'err');
    return null;
  }
}
