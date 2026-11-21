// Tab Chạy / Xem: view realtime dùng chung cho SIM, REAL, REPLAY (wiki/simulation.md §5).
import { api, guard, toast } from './api.js';
import { Chart } from './charts.js';
import { cssVar, drawGrid, drawPath, drawRays, drawRobot, fitCanvas, makeTransform } from './draw.js';
import { on, store } from './store.js';

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const PHYS = [
  ['motor_scale_l', 'Motor trái ×', 0.6, 1.2, 0.01, 1.0],
  ['motor_scale_r', 'Motor phải ×', 0.6, 1.2, 0.01, 1.0],
  ['fric_scale', 'Ma sát ×', 0.5, 2.5, 0.05, 1.0],
  ['slip_turn', 'Trượt khi quay', 0, 0.15, 0.005, 0.03],
  ['gyro_bias_dps', 'Bias gyro bật nguồn °/s', -5, 5, 0.1, 1.5],
  ['gyro_warm_dps', 'Trôi nóng máy °/s', -2, 2, 0.05, 0.6],
  ['gyro_rw_dps', 'Bias random walk', 0, 0.05, 0.002, 0.01],
  ['gyro_scale_err', 'Sai scale gyro', -0.05, 0.05, 0.002, 0.01],
  ['gyro_noise_dps', 'Nhiễu trắng gyro °/s', 0, 1, 0.01, 0.05],
  ['gyro_vib', 'Nhiễu rung motor', 0, 0.01, 0.0005, 0.002],
  ['diam_err_l', 'Sai đường kính bánh trái', -0.05, 0.05, 0.002, 0],
  ['diam_err_r', 'Sai đường kính bánh phải', -0.05, 0.05, 0.002, 0],
  ['tof_sigma', 'Nhiễu ToF mm', 0.5, 15, 0.5, 3.0],
  ['tof_dx_l', 'Mắt trái nhô trước mm', -5, 5, 0.5, 0],
  ['tof_dx_r', 'Mắt phải nhô trước mm', -5, 5, 0.5, 0],
  ['tof_off_l', 'Offset module trái mm', -15, 15, 0.5, 0],
  ['tof_off_r', 'Offset module phải mm', -15, 15, 0.5, 0],
  ['tof_yaw_l', 'Mắt trái quay lệch °', -10, 10, 0.5, 0],
  ['tof_yaw_r', 'Mắt phải quay lệch °', -10, 10, 0.5, 0],
];

const trails = new Map();
let charts = null;
let lastT = null;
let dirty = false;
let lastPanel = 0;
let physRobots = '';
let wheelRobot = null;
let lastChecksKey = '';
let checkedKey = null; // đường đã "Kiểm tra" đạt → Start không đặt lại xe ảo (giữ đúng vị trí đã kiểm tra)

function roleColor(role) {
  return role === 'leader' ? cssVar('--leader') : role === 'follower' ? cssVar('--follower') : cssVar('--muted');
}

function n(v, d = 0) { return v === null || v === undefined || Number.isNaN(v) ? '—' : Number(v).toFixed(d); }

export function onFrame(f) {
  store.frame = f;
  if (lastT !== null && f.t < lastT - 0.5) { trails.clear(); charts && Object.values(charts).forEach((c) => c.clear()); }
  lastT = f.t;
  for (const r of f.robots) {
    if (!r.est) continue;
    if (!trails.has(r.id)) trails.set(r.id, { est: [], truth: [] });
    const tr = trails.get(r.id);
    const push = (arr, p) => {
      const l = arr[arr.length - 1];
      if (!l || Math.hypot(l[0] - p.x, l[1] - p.y) > 6) { arr.push([p.x, p.y]); if (arr.length > 3000) arr.shift(); }
    };
    push(tr.est, r.est);
    if (r.truth) push(tr.truth, r.truth);
  }
  const F = f.robots.find((r) => r.role === 'follower');
  const L = f.robots.find((r) => r.role === 'leader');
  if (charts) {
    if (F?.gap) charts.gap.push(f.t, { est: F.gap.est, meas: F.gap.meas, target: F.gap.target });
    const W = f.robots.find((r) => r.id === wheelRobot) || F || L;
    if (W?.wheels) {
      charts.wheel.title = `Tốc độ bánh · ${W.name}`;
      charts.wheel.push(f.t, { lc: W.wheels.l.v_cmd, lv: W.wheels.l.v, rc: W.wheels.r.v_cmd, rv: W.wheels.r.v });
    }
    const yerr = (r) => (r?.truth && r?.est ? (((r.est.yaw - r.truth.yaw + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI) * DEG : null);
    charts.psi.push(f.t, { dpsi: F?.dpsi != null ? F.dpsi * DEG : null, el: yerr(L), ef: yerr(F) });
  }
  dirty = true;
  const now = performance.now();
  if (now - lastPanel > 200) { lastPanel = now; renderPanels(f); }
  for (const e of f.events || []) addEvent(e);
}

function addEvent(e) {
  const ul = $('eventLog');
  const li = document.createElement('li');
  li.className = e.kind || '';
  let txt = e.msg || '';
  if (e.kind === 'prim_done') txt = `${e.prim} #${e.id} ${e.ok ? 'xong' : 'lỗi ' + e.err}`;
  else if (e.kind === 'calib_done') txt = e.ok
    ? `calib IMU xong: bias ${n(e.result.bias_dps, 2)}°/s, nhiễu ${n(e.result.std_dps, 3)}°/s${e.result.restarts ? `, làm lại ${e.result.restarts} lần` : ''}`
    : `calib IMU LỖI: ${e.err}`;
  else if (e.kind === 'fault') txt = `FAULT ${e.code}`;
  else if (e.kind === 'nack') txt = `NACK ${e.err}`;
  if (e.kind === 'prim_done' && e.err === 'preempted') return;
  li.textContent = `${n(e.t, 1).padStart(6)} ${e.robot}: ${txt}`;
  ul.prepend(li);
  while (ul.children.length > 200) ul.lastChild.remove();
}

function renderPanels(f) {
  const c = f.coord || {};
  const badge = $('coordState');
  badge.textContent = c.state || 'IDLE';
  badge.className = `badge ${c.state}`;
  const solo = c.mode === 'solo';
  $('coordSeg').textContent = c.phase === 'CALIB_IMU' ? `Đang calib IMU: giữ ${solo ? 'xe' : '2 xe'} đứng yên…`
    : c.phase === 'CHECK' ? (solo ? 'Đang kiểm tra xe…' : 'Đang kiểm tra vị trí 2 xe…')
    : c.n_segments ? `Đoạn ${c.seg + 1}/${c.n_segments} · ${c.n_corners} góc cua` : 'Chưa chạy';
  const steps = c.steps || [];
  $('stepper').innerHTML = steps.map((s, k) => {
    const idx = k + 1;
    const cls = c.state === 'DONE' ? 'done' : idx < c.step ? 'done' : idx === c.step ? 'cur' : '';
    return `<li class="${cls}">${s}</li>`;
  }).join('');
  $('coordErr').textContent = c.error || '';
  const checks = c.checks || [];
  const key = JSON.stringify(checks);
  if (key !== lastChecksKey) {
    lastChecksKey = key;
    $('checkList').innerHTML = checks.map((x) => `<li class="${x.level}"><b>${x.name}</b> — ${x.detail}</li>`).join('');
  }

  $('robotCards').innerHTML = f.robots.length ? f.robots.map((r) => {
    const t = r.truth, e = r.est;
    const perr = t && e ? Math.hypot(t.x - e.x, t.y - e.y) : null;
    const flags = (r.flags || []).map((x) => `<span class="chip ${['SAT', 'SLIP'].includes(x) ? 'warn' : ['LEADER_LOST', 'LINK_LOST'].includes(x) ? 'bad' : ''}">${x}</span>`).join('');
    return `<div class="rcard ${r.role || ''}">
      <div class="name">${r.name} <span class="muted small">${r.role || 'chưa gán'} · ${r.source}</span>
        <span class="badge ${r.state}">${r.state || '?'}</span></div>
      <div class="rgrid">
        <div><span>prim</span> <b>${r.prim?.name || '—'}</b></div>
        <div><span>v</span> <b>${n(e?.v)} mm/s</b></div>
        <div><span>s</span> <b>${n(e?.s)} mm</b></div>
        <div><span>gap</span> <b>${n(r.gap?.est)}</b></div>
        <div><span>đo</span> <b>${n(r.gap?.meas)}</b></div>
        <div><span>Δψ</span> <b>${r.dpsi != null ? n(r.dpsi * DEG, 1) + '°' : '—'}</b></div>
        <div><span>ToF L/R</span> <b>${r.tof ? (r.tof.l.st ? '×' : n(r.tof.l.mm)) + '/' + (r.tof.r.st ? '×' : n(r.tof.r.mm)) : '—'}</b></div>
        <div><span>pin</span> <b>${n(r.vbat, 1)} V</b></div>
        <div><span>cfg</span> <b>${r.cfg_rev ?? '—'}</b></div>
        ${perr !== null ? `<div><span>sai vị trí</span> <b>${n(perr)} mm</b></div>` : ''}
        <div><span>học FF</span> <b>${r.wheels ? n(r.wheels.l.adapt * 100) + '/' + n(r.wheels.r.adapt * 100) + '%' : '—'}</b></div>
      </div>
      ${imuLine(r)}
      <div>${flags}${r.fault ? `<span class="chip bad">${r.fault}</span>` : ''}</div>
    </div>`;
  }).join('') : '<p class="muted">Chưa có robot. Bật mô phỏng ở tab Thiết bị.</p>';

  const sim = f.sim;
  $('simPanel').style.display = sim ? '' : 'none';
  if (sim) {
    $('simClock').textContent = `t=${n(sim.t, 1)}s · ×${sim.scale} · va chạm ${sim.collisions}`;
    $('simPlay').textContent = sim.running ? '⏸ Dừng' : '▶ Chạy';
    const ids = Object.keys(sim.mismatch).join(',');
    if (ids !== physRobots) { physRobots = ids; renderPhys(sim); }
  }
  $('recBtn').textContent = f.recording ? '■ Dừng ghi' : '● Ghi log';
  $('srcBadge').textContent = f.source;
  $('srcBadge').className = `badge ${f.source}`;
}

function imuLine(r) {
  const m = r.imu;
  if (!m) return '';
  const st = m.calib != null ? `<b class="mono">calib ${n(m.calib * 100)}%</b>`
    : m.ok ? '<b>✓</b>' : '<b class="danger-text">chưa calib</b>';
  const truth = r.truth?.gyro_bias != null ? ` <span class="muted">(thật ${n(r.truth.gyro_bias * DEG, 2)})</span>` : '';
  return `<div class="imu small">
    <span class="muted">IMU</span> ${st} · bias <b class="mono">${n(m.bias_dps, 2)}°/s</b>${truth}
    · ${n(m.temp_c, 1)}°C · k <b class="mono">${n(m.k_dps_c, 3)}</b>${m.k_learned ? ' <span class="chip">học</span>' : ''}
    ${m.zupt ? '<span class="chip">ZUPT</span>' : ''}
    <div class="row gap"><button class="btn small" data-act="calib" data-id="${r.id}">Calib IMU</button>
    <button class="btn small" data-act="savek" data-id="${r.id}" ${m.k_learned ? '' : 'disabled'}>Lưu k nhiệt</button>
    <button class="btn small" data-act="caltof" data-id="${r.id}" data-src="${r.source}">Calib ToF</button></div>
  </div>`;
}

// Hiệu chuẩn 2 mắt ToF: xe quay ±40° trước mặt phẳng, đo lệch lắp đặt → hỏi rồi Apply + Save.
export async function calibTof(id, source) {
  if (source !== 'SIM' && !confirm('Đặt xe trước một mặt phẳng rộng (tường/hộp ≥ 40 cm), cách 15–30 cm, '
    + 'mặt xe gần vuông góc (±10°), không vật cản 2 bên. Xe sẽ tự quay ±40° trong ~25 s. Bắt đầu?')) return;
  toast('Đang hiệu chuẩn ToF (~25 s)…');
  const r = await guard(() => api('POST', `/api/robots/${id}/calib_tof`, {}));
  if (!r) return;
  const x = r.result;
  const msg = `Kết quả hiệu chuẩn ToF:\n`
    + `• Lệch 2 mắt khi vuông góc: ${n(x.c_mm, 1)} mm → Δψ đang lệch ${n(x.dpsi_bias_deg, 1)}°\n`
    + `• Offset mới: trái ${n(x.offset_l_mm, 1)} mm, phải ${n(x.offset_r_mm, 1)} mm (cũ ${n(x.old_offset_l_mm, 1)} / ${n(x.old_offset_r_mm, 1)})\n`
    + `• Baseline hiệu dụng: ${n(x.baseline_mm, 0)} mm (chỉ để tham khảo; lệch >15% so với cấu hình → kiểm tra lắp)\n`
    + `• Nhiễu fit: ${n(x.resid_mm, 1)} mm · hướng vuông góc lúc đặt: ${n(x.theta0_deg, 1)}°\n\nÁp dụng offset mới và lưu flash?`;
  if (!confirm(msg)) return;
  await guard(() => api('PUT', `/api/robots/${id}/config`,
    { patch: { 'range.offset_l_mm': Math.round(x.offset_l_mm * 100) / 100, 'range.offset_r_mm': Math.round(x.offset_r_mm * 100) / 100 }, persist: true }),
  'Đã lưu offset ToF vào flash');
}

function renderPhys(sim) {
  const ids = Object.keys(sim.mismatch);
  $('faultRobot').innerHTML = ids.map((id) => {
    const r = store.frame.robots.find((x) => x.id === id);
    return `<option value="${id}">${r?.name || id}${r?.role ? ' (' + r.role + ')' : ''}</option>`;
  }).join('');
  $('physForm').innerHTML = ids.map((id) => `
    <h3>${store.frame.robots.find((x) => x.id === id)?.name || id}</h3>
    ${PHYS.map(([k, label, lo, hi, step, def]) => {
      const v = sim.mismatch[id][k] ?? def;
      return `<div class="phys-row"><span>${label}</span>
        <input type="range" min="${lo}" max="${hi}" step="${step}" value="${v}" data-r="${id}" data-k="${k}">
        <span class="mono">${v}</span></div>`;
    }).join('')}`).join('');
  $('physForm').querySelectorAll('input[type=range]').forEach((inp) => {
    inp.oninput = () => { inp.nextElementSibling.textContent = inp.value; };
    inp.onchange = () => guard(() => api('POST', '/api/sim/physics',
      { robot: inp.dataset.r, mismatch: { [inp.dataset.k]: Number(inp.value) } }));
  });
}

function draw() {
  requestAnimationFrame(draw);
  if (!dirty || !document.querySelector('#tab-run.active')) return;
  dirty = false;
  const f = store.frame;
  const canvas = $('runCanvas');
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  if (!f) return;
  const grid = f.grid || store.grid;
  let view = null;
  if ($('layFollow').checked && f.robots.length) {
    const pts = f.robots.filter((r) => r.est).map((r) => r.est);
    if (pts.length) view = { cx: pts.reduce((a, p) => a + p.x, 0) / pts.length, cy: pts.reduce((a, p) => a + p.y, 0) / pts.length, zoom: 2.5 };
  }
  const T = makeTransform(w, h, grid, 24, view);
  drawGrid(ctx, T, grid);
  const path = f.path || store.plan?.path;
  if (path) drawPath(ctx, T, path, { alpha: 0.55, sweep: path.formation ? Math.max(path.formation.r_sweep_leader, path.formation.r_sweep_follower) : 0 });

  if ($('layTrail').checked) {
    for (const r of f.robots) {
      const tr = trails.get(r.id);
      if (!tr) continue;
      for (const [arr, dash] of [[tr.truth, false], [tr.est, true]]) {
        if (arr.length < 2 || (!dash && !$('layTruth').checked)) continue;
        ctx.save();
        ctx.strokeStyle = roleColor(r.role);
        ctx.globalAlpha = dash ? 0.5 : 0.8;
        ctx.setLineDash(dash ? [3, 4] : []);
        ctx.lineWidth = 1.3;
        ctx.beginPath();
        arr.forEach(([x, y], k) => { const [cx, cy] = T.toC(x, y); if (k) ctx.lineTo(cx, cy); else ctx.moveTo(cx, cy); });
        ctx.stroke();
        ctx.restore();
      }
    }
  }
  const blink = Math.floor(performance.now() / 250) % 2 === 0;
  for (const r of f.robots) {
    if (!r.est || !r.geom) continue;
    const col = roleColor(r.role);
    if (r.truth && $('layTruth').checked) {
      drawRobot(ctx, T, r.truth, r.geom, { stroke: cssVar('--truth'), dashed: true, outlineOnly: true, alpha: 0.7,
        fill: r.truth.collide ? cssVar('--danger') : null });
    }
    const ident = (r.flags || []).includes('IDENT') && blink;
    const pose = r.truth && $('layTruth').checked ? r.truth : r.est;
    if ($('layRays').checked) drawRays(ctx, T, pose, r.geom, r.tof);
    drawRobot(ctx, T, r.est, r.geom, { stroke: col, fill: ident ? cssVar('--warn') : col, alpha: 0.85,
      label: `${r.name}${r.prim ? ' · ' + r.prim.name : ''}`, labelPos: r.role === 'follower' ? 'below' : 'above' });
  }
  ctx.fillStyle = cssVar('--muted');
  ctx.font = '11px ' + cssVar('--mono');
  ctx.fillText(`${f.source} · t=${n(f.t, 2)}s${f.replay ? ` · replay ${f.replay.i}/${f.replay.n}` : ''}`, 10, h - 10);
  if ($('layTruth').checked && f.robots.some((r) => r.truth)) {
    ctx.fillText('nét liền đặc = ước lượng của robot · viền đứt = vị trí thật (sim)', 10, 16);
  }
  for (const c of Object.values(charts)) c.draw();
}

async function loadLogs() {
  const files = await guard(() => api('GET', '/api/logs')) || [];
  $('replayFile').innerHTML = '<option value="">Replay…</option>' + files.map((x) => `<option>${x}</option>`).join('');
}

export async function startRun() {
  if (store.mode === 'follow') {  // 1 xe bám tấm phẳng cầm tay: không cần đường vẽ
    const r = await guard(() => api('POST', '/api/follow/start', {}));
    if (r) {
      trails.clear();
      Object.values(charts).forEach((c) => c.clear());
      toast(`Đang bám tấm: giữ ${r.d_target} mm. Cầm tấm phẳng trước 2 mắt ToF; bấm Stop để dừng`, 'ok');
    }
    return !!r;
  }
  if (store.points.length < 2) { toast('Vẽ đường ở tab Bản đồ trước', 'err'); return false; }
  const key = JSON.stringify([store.grid, store.points, store.mode]);
  const place = checkedKey !== key;
  checkedKey = null;
  const r = await guard(() => api('POST', '/api/run/start', { grid: store.grid, points: store.points, mode: store.mode, place }));
  if (r) {
    trails.clear();
    Object.values(charts).forEach((c) => c.clear());
    (r.warnings || []).forEach((w) => toast(w));
    toast(`Bắt đầu chạy đường #${r.path.path_id}`, 'ok');
  }
  return !!r;
}

export function initView() {
  charts = {
    gap: new Chart($('chGap'), { title: 'Gap follower', unit: 'mm', series: [
      { key: 'est', label: 'ước lượng', color: '--chart-1' },
      { key: 'meas', label: 'ToF', color: '--chart-3' },
      { key: 'target', label: 'mục tiêu', color: '--chart-2', dash: true }] }),
    wheel: new Chart($('chWheel'), { title: 'Tốc độ bánh', unit: 'mm/s', series: [
      { key: 'lc', label: 'L lệnh', color: '--chart-1', dash: true }, { key: 'lv', label: 'L đo', color: '--chart-1' },
      { key: 'rc', label: 'R lệnh', color: '--chart-2', dash: true }, { key: 'rv', label: 'R đo', color: '--chart-2' }] }),
    psi: new Chart($('chPsi'), { title: 'Góc', unit: '°', series: [
      { key: 'dpsi', label: 'Δψ ToF', color: '--chart-3' },
      { key: 'el', label: 'sai hướng L', color: '--chart-1' },
      { key: 'ef', label: 'sai hướng F', color: '--chart-2' }] }),
  };
  $('runStart').onclick = () => startRun();
  $('runCheck').onclick = async () => {
    if (store.mode === 'follow') {
      const r = (store.frame?.robots || []).find((x) => x.role === 'follower') || (store.frame?.robots || [])[0];
      const t = r?.tof || {};
      const eye = (k) => (t[k] && t[k].st === 0 ? `${t[k].mm} mm` : 'không thấy');
      $('checkList').innerHTML = r ? `<li class="${t.l?.st === 0 && t.r?.st === 0 ? 'ok' : 'warn'}"><b>${r.name}: ToF thấy tấm</b> — trái ${eye('l')} · phải ${eye('r')}</li>`
        + `<li class="${r.imu?.ok ? 'ok' : 'err'}"><b>IMU đã calib</b> — ${r.imu?.ok ? 'ok' : 'chưa: để xe đứng yên vài giây'}</li>` : '<li class="err">Chưa có robot</li>';
      return;
    }
    if (store.points.length < 2) { toast('Vẽ đường ở tab Bản đồ trước', 'err'); return; }
    toast(store.mode === 'solo' ? 'Đang kiểm tra xe…' : 'Đang kiểm tra vị trí 2 xe…');
    const r = await guard(() => api('POST', '/api/run/check', { grid: store.grid, points: store.points, mode: store.mode }));
    if (!r) return;
    $('checkList').innerHTML = r.checks.map((x) => `<li class="${x.level}"><b>${x.name}</b> — ${x.detail}</li>`).join('');
    lastChecksKey = JSON.stringify(r.checks);
    checkedKey = r.ok ? JSON.stringify([store.grid, store.points, store.mode]) : null;
    toast(r.ok ? 'Kiểm tra ĐẠT — bấm Start để chạy' : 'Kiểm tra KHÔNG đạt — xem danh sách', r.ok ? 'ok' : 'err');
  };
  for (const id of ['placeMm', 'placeDeg']) {
    $(id).onchange = () => guard(() => api('POST', '/api/sim/control',
      { place_noise_mm: Number($('placeMm').value) || 0, place_noise_deg: Number($('placeDeg').value) || 0 }));
  }
  $('runPause').onclick = () => guard(() => api('POST', '/api/run/pause'));
  $('runResume').onclick = () => guard(() => api('POST', '/api/run/resume'));
  $('runStop').onclick = () => guard(() => api('POST', '/api/run/stop'), 'Đã dừng');
  $('runReset').onclick = () => guard(() => api('POST', '/api/run/reset'), 'Đã reset lỗi');
  $('runEstop').onclick = () => guard(() => api('POST', '/api/run/estop'), 'E-STOP');
  $('simPlay').onclick = () => guard(() => api('POST', '/api/sim/control', { running: !store.frame?.sim?.running }));
  $('simScale').onchange = (e) => guard(() => api('POST', '/api/sim/control', { scale: Number(e.target.value) }));
  $('simStep').onclick = () => guard(() => api('POST', '/api/sim/control', { step: 0.1 }));
  $('recBtn').onclick = () => guard(() => api('POST', `/api/record/${store.frame?.recording ? 'stop' : 'start'}`))
    .then(() => loadLogs());
  $('replayBtn').onclick = () => {
    const file = $('replayFile').value;
    if (!file) { toast('Chọn file log', 'err'); return; }
    trails.clear();
    guard(() => api('POST', '/api/replay/start', { file, speed: 1 }));
  };
  $('replayStop').onclick = () => guard(() => api('POST', '/api/replay/stop'));
  document.querySelectorAll('[data-fault]').forEach((b) => {
    b.onclick = () => {
      const robot = $('faultRobot').value;
      const t = b.dataset.fault;
      const calls = t === 'tof_both'
        ? [{ type: 'tof_block', side: 'l', dur: 1 }, { type: 'tof_block', side: 'r', dur: 1 }]
        : [{ type: t, side: b.dataset.side, dur: t === 'caster' ? 2 : t === 'bump' ? 0.3 : 1 }];
      calls.forEach((c) => guard(() => api('POST', '/api/sim/fault', { robot, ...c })));
    };
  });
  // Thẻ robot vẽ lại 5 lần/s → dùng event delegation cho nút trong thẻ.
  $('robotCards').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-act]');
    if (!b) return;
    if (b.dataset.act === 'calib') guard(() => api('POST', `/api/robots/${b.dataset.id}/calib_imu`), 'Đang calib IMU — giữ xe đứng yên');
    if (b.dataset.act === 'savek') guard(() => api('POST', `/api/robots/${b.dataset.id}/save_temp_coeff`), 'Đã lưu hệ số nhiệt vào flash');
    if (b.dataset.act === 'caltof') calibTof(b.dataset.id, b.dataset.src);
  });
  $('runCanvas').addEventListener('click', () => {
    const ids = (store.frame?.robots || []).map((r) => r.id);
    if (ids.length) wheelRobot = ids[(ids.indexOf(wheelRobot) + 1) % ids.length];
  });
  window.addEventListener('keydown', (e) => {
    if (e.code === 'Space' && document.querySelector('#tab-run.active') && document.activeElement?.tagName !== 'INPUT') {
      e.preventDefault();
      guard(() => api('POST', '/api/run/estop'), 'E-STOP');
    }
  });
  ['layTruth', 'layRays', 'layTrail', 'layFollow'].forEach((id) => { $(id).onchange = () => { dirty = true; }; });
  new ResizeObserver(() => { dirty = true; }).observe($('runCanvas'));
  on('tab', (t) => { if (t === 'run') { dirty = true; loadLogs(); } });
  requestAnimationFrame(draw);
}
