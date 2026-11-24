// Tab Cấu hình: form sinh tự động từ schema chung, Apply (RAM) / Save (flash) / Revert...
import { api, guard, toast } from './api.js';
import { gapClearance } from './config-gap.js?v=gap-view-11';
import { cssVar, drawRobot, fitCanvas } from './draw.js';
import { downloadJSON, emit, on, store } from './store.js';

const $ = (id) => document.getElementById(id);
const SHARED = '__shared__';

let target = SHARED;  // robot id hoặc SHARED
let current = {};     // giá trị đang có trên robot (hoặc shared trên PC)
let edits = {};       // chỉnh sửa chưa Apply
let errors = {};
let lastCfg = null;
let pairGeometry = { leader: null, follower: null };

function robotState(id) {
  const r = (store.frame?.robots || []).find((x) => x.id === id);
  return r?.state || '?';
}

function visibleParams() {
  const scope = target === SHARED ? 'shared' : 'robot';
  return store.schema.params.filter((p) => (p.scope || 'robot') === scope);
}

function renderTargets() {
  const box = $('cfgTargets');
  const items = [[SHARED, 'Tham số chung (2 xe)'], ...store.robots.map((r) => [r.id, `${r.name}${r.role ? ' · ' + r.role : ''}`])];
  box.innerHTML = items.map(([id, label]) => `<button data-t="${id}" class="${id === target ? 'active' : ''}">${label}</button>`).join('');
  box.querySelectorAll('button').forEach((b) => { b.onclick = () => select(b.dataset.t); });
}

async function select(t) {
  target = t;
  edits = {};
  errors = {};
  renderTargets();
  await pull();
}

async function pull() {
  if (target === SHARED) {
    current = await guard(() => api('GET', '/api/shared')) || {};
    const leader = store.robots.find((r) => r.role === 'leader');
    const follower = store.robots.find((r) => r.role === 'follower');
    const loadRobot = async (r) => r ? (await api('GET', `/api/robots/${r.id}/config`).catch(() => null))?.params || null : null;
    const [leaderCfg, followerCfg] = await Promise.all([loadRobot(leader), loadRobot(follower)]);
    pairGeometry = { leader: leaderCfg, follower: followerCfg };
    lastCfg = null;
    $('cfgInfo').textContent = 'Tham số chung được lưu trên PC (project) và đẩy xuống cả 2 robot khi Apply.';
  } else {
    lastCfg = await guard(() => api('GET', `/api/robots/${target}/config`));
    current = lastCfg?.params || {};
    if (lastCfg) {
      const flags = lastCfg.flags.length ? ' · cờ: ' + lastCfg.flags.join(', ') : '';
      $('cfgInfo').innerHTML = `cfg_rev <b>${lastCfg.cfg_rev}</b> · crc ${lastCfg.crc} · `
        + (lastCfg.dirty ? '<b class="danger-text">RAM khác flash (chưa Save)</b>' : 'RAM = flash') + flags;
    }
  }
  renderForm();
  renderSide();
}

function fmt(v, p) {
  if (p.type === 'bool') return v;
  if (typeof v !== 'number') return v;
  return p.type === 'int' ? String(v) : String(Math.round(v * 10000) / 10000);
}

function gapVisualHtml() {
  const target = Number('gap.target_mm' in edits ? edits['gap.target_mm'] : current['gap.target_mm']);
  const followerX = Number(pairGeometry.follower?.['range.x_mm']);
  const leaderRear = Number(pairGeometry.leader?.['body.axle_to_rear_mm']);
  const clear = gapClearance(target, followerX, leaderRear);
  if (clear === null) {
    return '<div class="gap-figure gap-figure-missing">Cần kết nối leader và follower để tính khoảng hở thực tế.</div>';
  }
  const bad = clear < 0;
  return `<div class="gap-figure ${bad ? 'bad' : ''}">
    <div class="gap-figure-head"><b>Khoảng hở thực tế</b><span class="gap-clear">${clear.toFixed(1)} mm</span></div>
    <svg viewBox="0 0 680 205" role="img" aria-label="Sơ đồ khoảng cách thực tế giữa follower và leader">
      <defs>
        <marker id="gap-arrow" markerWidth="8" markerHeight="8" refX="7" refY="3" orient="auto">
          <path d="M0,0 L0,6 L7,3 z" fill="currentColor"/>
        </marker>
      </defs>
      <rect x="45" y="55" width="160" height="75" rx="10" class="gap-body"/>
      <rect x="475" y="55" width="160" height="75" rx="10" class="gap-body"/>
      <text x="82" y="99" class="gap-label">follower</text>
      <text x="510" y="99" class="gap-label">leader</text>
      <rect x="198" y="78" width="9" height="30" rx="3" class="gap-sensor"/>
      <rect x="470" y="64" width="7" height="58" rx="2" class="gap-rear"/>
      <line x1="212" y1="93" x2="465" y2="93" class="gap-measure" marker-start="url(#gap-arrow)" marker-end="url(#gap-arrow)"/>
      <text x="269" y="82" class="gap-value">${clear.toFixed(1)} mm</text>
      <text x="279" y="119" class="gap-subtitle">mặt ToF → đuôi leader</text>
      <line x1="125" y1="158" x2="555" y2="158" class="gap-center" marker-start="url(#gap-arrow)" marker-end="url(#gap-arrow)"/>
      <line x1="125" y1="149" x2="125" y2="167" class="gap-center"/>
      <line x1="555" y1="149" x2="555" y2="167" class="gap-center"/>
      <text x="290" y="187" class="gap-subtitle">tâm trục → tâm trục: ${target.toFixed(1)} mm</text>
    </svg>
    <div class="gap-formula"><span>${target.toFixed(1)}</span> − <span>${followerX.toFixed(1)}</span> − <span>${leaderRear.toFixed(1)}</span> = <b>${clear.toFixed(1)} mm</b></div>
    <p class="muted small">Đo từ mặt kính ToF của follower đến mặt đuôi của leader.</p>
  </div>`;
}

function updateGapVisual() {
  const box = $('gapVisual');
  if (box) box.innerHTML = gapVisualHtml();
}

function renderForm() {
  const form = $('cfgForm');
  const params = visibleParams();
  const running = target !== SHARED && ['RUNNING', 'PAUSED', 'JOG'].includes(robotState(target));
  const groups = store.schema.groups.filter((g) => params.some((p) => p.group === g.id));
  form.innerHTML = groups.map((g) => `
    <div class="cfg-group"><h4>${g.label}</h4>
      ${params.filter((p) => p.group === g.id).map((p) => {
        const val = p.key in edits ? edits[p.key] : current[p.key];
        const dis = running && p.apply === 'idle' ? 'disabled title="Chỉ đổi khi xe đứng yên"' : '';
        const input = p.type === 'bool'
          ? `<input type="checkbox" data-k="${p.key}" ${val ? 'checked' : ''} ${dis}>`
          : `<input type="number" step="any" data-k="${p.key}" value="${fmt(val, p)}" class="${p.key in edits ? 'dirty' : ''}" ${dis}>`;
        return `<div class="cfg-row">
          <div class="lbl">${p.desc}<span class="key">${p.key}${p.apply === 'idle' ? ' · idle' : ''}</span></div>
          ${input}<span class="unit">${p.unit || ''}</span>
          ${errors[p.key] ? `<div class="perr">${errors[p.key]}</div>` : ''}
        </div>`;
      }).join('')}
      ${target === SHARED && g.id === 'gap' ? `<div id="gapVisual">${gapVisualHtml()}</div>` : ''}
    </div>`).join('');
  form.querySelectorAll('input[data-k]').forEach((inp) => {
    inp.oninput = () => {
      const k = inp.dataset.k;
      const v = inp.type === 'checkbox' ? inp.checked : inp.value;
      if (String(v) === String(current[k])) delete edits[k]; else edits[k] = inp.type === 'checkbox' ? v : Number(v);
      inp.classList.toggle('dirty', k in edits);
      if (k === 'gap.target_mm') updateGapVisual();
      renderSide();
    };
  });
}

function renderSide() {
  const dl = $('cfgDerived');
  const d = lastCfg?.derived;
  dl.innerHTML = d ? Object.entries(d).map(([k, v]) => `<dt>${k}</dt><dd>${typeof v === 'number' ? v.toFixed(4).replace(/\.?0+$/, '') : v}</dd>`).join('')
    : '<dt class="muted">—</dt><dd></dd>';
  const p = { ...(lastCfg?.params || {}), ...edits };
  const cv = $('cfgPreview');
  const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);
  if (!p['body.length_mm']) {
    ctx.fillStyle = cssVar('--muted');
    ctx.font = '12px ' + cssVar('--font');
    ctx.fillText('Chọn 1 robot để xem hình xe', 12, h / 2);
    return;
  }
  const geom = { length: p['body.length_mm'], width: p['body.width_mm'], front: p['body.axle_to_front_mm'],
    rear: p['body.axle_to_rear_mm'], tof_x: p['range.x_mm'], tof_b: p['range.baseline_mm'],
    track: p['wheel.track_mm'], wheel_d: p['wheel.diameter_mm'] };
  const rsw = Math.hypot(Math.max(geom.front, geom.rear), geom.width / 2);
  const span = 2 * rsw + 80;
  const s = Math.min(w, h) / span;
  const T = { s, toC: (x, y) => [w / 2 + x * s, h / 2 - y * s] };
  ctx.strokeStyle = cssVar('--muted');
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.arc(w / 2, h / 2, rsw * s, 0, Math.PI * 2); ctx.stroke();
  ctx.setLineDash([]);
  drawRobot(ctx, T, { x: 0, y: 0, yaw: Math.PI / 2 }, geom, { fill: cssVar('--surface'), stroke: cssVar('--leader') });
  ctx.fillStyle = cssVar('--muted');
  ctx.font = '11px ' + cssVar('--font');
  ctx.fillText(`R quét ${rsw.toFixed(0)} mm`, 8, h - 8);
}

async function apply(persist) {
  if (!Object.keys(edits).length && !persist) { toast('Không có thay đổi'); return; }
  errors = {};
  if (target === SHARED) {
    try {
      const r = await api('PUT', '/api/shared', { patch: edits, persist });
      const bad = Object.entries(r.robots).filter(([, v]) => v && v.ok === false);
      if (bad.length) toast(`Một số robot từ chối: ${bad.map(([k, v]) => `${k}: ${JSON.stringify(v.errors || v.err)}`).join('; ')}`, 'err');
      else toast('Đã áp dụng tham số chung cho các robot', 'ok');
      edits = {};
      store.shared = r.shared;
      emit('shared', r.shared);
    } catch (e) {
      errors = e.data?.detail?.errors || {};
      toast(e.message, 'err');
    }
  } else {
    try {
      const r = await api('PUT', `/api/robots/${target}/config`, { patch: edits, persist });
      if (r.ok) { toast(`Đã áp dụng (rev ${r.cfg_rev})${persist ? ' + lưu flash' : ''}`, 'ok'); edits = {}; }
      else { errors = r.errors || {}; toast(r.busy ? 'Robot đang chạy: tham số idle bị khoá' : 'Tham số không hợp lệ', 'err'); }
    } catch (e) { toast(e.message, 'err'); }
  }
  await pull();
}

export function initConfig() {
  $('cfgPull').onclick = () => pull();
  $('cfgApply').onclick = () => apply(false);
  $('cfgApplySave').onclick = () => apply(true);
  $('cfgSave').onclick = async () => {
    if (target === SHARED) return apply(true);
    await guard(() => api('POST', `/api/robots/${target}/config/save`), 'Đã lưu flash');
    pull();
  };
  $('cfgRevert').onclick = async () => {
    if (target === SHARED) { edits = {}; return pull(); }
    await guard(() => api('POST', `/api/robots/${target}/config/reload`), 'Đã nạp lại từ flash');
    edits = {};
    pull();
  };
  $('cfgFactory').onclick = async () => {
    if (target === SHARED || !confirm('Đưa robot về cấu hình mặc định (RAM)?')) return;
    await guard(() => api('POST', `/api/robots/${target}/config/factory_reset`), 'Đã về mặc định (chưa lưu flash)');
    pull();
  };
  $('cfgCopy').onclick = async () => {
    if (target === SHARED) return;
    const other = store.robots.find((r) => r.id !== target);
    if (!other) { toast('Không có robot kia', 'err'); return; }
    const patch = {};
    for (const p of store.schema.params) if ((p.scope || 'robot') === 'robot') patch[p.key] = (p.key in edits ? edits[p.key] : current[p.key]);
    await guard(() => api('PUT', `/api/robots/${other.id}/config`, { patch, persist: false }), `Đã copy sang ${other.name} (RAM)`);
  };
  $('cfgExport').onclick = () => downloadJSON(`config-${target === SHARED ? 'shared' : target}.json`, { ...current, ...edits });
  on('robots', () => renderTargets());
  on('tab', (t) => { if (t === 'config') { renderTargets(); pull(); } });
}
