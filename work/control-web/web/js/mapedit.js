// Tab Bản đồ & Đường: vẽ đường trên lưới (ép ngang/dọc), kiểm tra qua /api/plan.
import { api, toast } from './api.js';
import { cssVar, drawGrid, drawPath, drawRobot, fitCanvas, makeTransform } from './draw.js';
import { downloadJSON, emit, on, saveProject, store } from './store.js';

const $ = (id) => document.getElementById(id);
const PRESETS = {
  line: [[5, 4], [28, 4]],
  L: [[5, 3], [21, 3], [21, 14]],
  rect: [[6, 3], [26, 3], [26, 13], [6, 13], [6, 6]],
  zigzag: [[5, 2], [12, 2], [12, 7], [20, 7], [20, 12], [28, 12]],
};

let canvas;
let hover = null;
let planTimer = null;

function T() {
  const { w, h } = { w: canvas.clientWidth, h: canvas.clientHeight };
  return makeTransform(w, h, store.grid);
}

function snap(cx, cy) {
  const t = T();
  const [x, y] = t.toW(cx, cy);
  const c = store.grid.cell_mm;
  let i = Math.round(x / c), j = Math.round(y / c);
  i = Math.max(0, Math.min(store.grid.cols, i));
  j = Math.max(0, Math.min(store.grid.rows, j));
  const last = store.points[store.points.length - 1];
  if (last) {
    // Ép ngang/dọc theo trục có độ lệch lớn hơn
    if (Math.abs(i - last[0]) >= Math.abs(j - last[1])) j = last[1]; else i = last[0];
  }
  return [i, j];
}

// Đoạn mới (a→b) có chạm phần đường đã vẽ không (trừ đỉnh a nối với đoạn ngay trước)?
function crossesPath(a, b) {
  if (!a || store.points.length < 2) return false;
  const lo = (u, v) => Math.min(u, v), hi = (u, v) => Math.max(u, v);
  const pts = store.points;
  for (let k = 0; k < pts.length - 1; k++) {
    const [p, q] = [pts[k], pts[k + 1]];
    const adjacent = k === pts.length - 2;
    // Hộp bao của 2 đoạn ngang/dọc giao nhau ⇒ chạm nhau
    const touch = lo(a[0], b[0]) <= hi(p[0], q[0]) && lo(p[0], q[0]) <= hi(a[0], b[0])
      && lo(a[1], b[1]) <= hi(p[1], q[1]) && lo(p[1], q[1]) <= hi(a[1], b[1]);
    if (!touch) continue;
    if (!adjacent) return true;
    // Đoạn kề: chỉ được chung đỉnh a, không được chồng ngược lại (quay đầu 180°)
    const back = (b[0] - a[0]) * (p[0] - a[0]) + (b[1] - a[1]) * (p[1] - a[1]) > 0;
    if (back) return true;
  }
  return false;
}

function changed() {
  saveProject();
  draw();
  clearTimeout(planTimer);
  planTimer = setTimeout(validate, 200);
}

export async function validate() {
  try {
    store.plan = await api('POST', '/api/plan', { grid: store.grid, points: store.points });
  } catch (e) {
    store.plan = { ok: false, errors: [e.message], warnings: [] };
  }
  renderPlan();
  draw();
  emit('plan', store.plan);
}

function renderPlan() {
  const p = store.plan;
  const st = $('planStatus');
  const stats = $('planStats');
  const msgs = $('planMsgs');
  if (!p) { st.innerHTML = ''; stats.innerHTML = ''; msgs.innerHTML = ''; return; }
  st.innerHTML = p.ok ? '<span class="ok-text">✓ Đường hợp lệ</span>' : '<span class="err">✗ Chưa hợp lệ</span>';
  const path = p.path;
  const f = p.formation || {};
  const sh = store.shared || {};
  const rows = [];
  if (path) {
    const vc = sh['motion.v_cruise'] || 150;
    rows.push(['Tổng dài', `${(path.total_len / 1000).toFixed(2)} m`], ['Số đoạn', path.segments.length],
      ['Số góc cua', path.corners_s.length],
      ['Ước tính', `~${Math.round(path.total_len / vc + path.corners_s.length * 16 + 3)} s`]);
  }
  if (f.g_target) {
    rows.push(['Gap đi thẳng', `${f.g_target.toFixed(0)} mm`], ['Gap chờ tối thiểu', `${f.g_hold_min.toFixed(0)} mm`],
      ['Khoảng thoát góc', `${f.exit_dist.toFixed(0)} mm`], ['Gap an toàn', `${f.gap_min_safe.toFixed(0)} mm`],
      ['R quét (L/F)', `${f.r_sweep_leader.toFixed(0)} / ${f.r_sweep_follower.toFixed(0)} mm`]);
  }
  stats.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  msgs.innerHTML = [...(p.errors || []).map((e) => `<li class="e">${e}</li>`),
    ...(p.warnings || []).map((w) => `<li class="w">${w}</li>`)].join('');
}

function draw() {
  if (!canvas) return;
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  const t = makeTransform(w, h, store.grid);
  drawGrid(ctx, t, store.grid);
  const c = store.grid.cell_mm;
  const p = store.plan;
  if (p?.path) {
    drawPath(ctx, t, p.path, { sweep: Math.max(p.formation.r_sweep_leader, p.formation.r_sweep_follower),
      color: p.ok ? cssVar('--path') : cssVar('--danger') });
    const g = p.geom;
    const st = p.path.start, fs = p.path.follower.start;
    drawRobot(ctx, t, { x: st.x, y: st.y, yaw: st.yaw_deg * Math.PI / 180 }, g.leader,
      { stroke: cssVar('--leader'), fill: cssVar('--surface'), alpha: 0.9, label: 'Đầu' });
    drawRobot(ctx, t, { x: fs.x, y: fs.y, yaw: fs.yaw_deg * Math.PI / 180 }, g.follower,
      { stroke: cssVar('--follower'), fill: cssVar('--surface'), alpha: 0.9, label: 'Đuôi' });
  } else if (store.points.length) {
    ctx.strokeStyle = cssVar('--path');
    ctx.lineWidth = 3;
    ctx.beginPath();
    store.points.forEach(([i, j], k) => {
      const [x, y] = t.toC(i * c, j * c);
      if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
  ctx.fillStyle = cssVar('--path');
  for (const [i, j] of store.points) {
    const [x, y] = t.toC(i * c, j * c);
    ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill();
  }
  if (hover) {
    const [x, y] = t.toC(hover[0] * c, hover[1] * c);
    const last = store.points[store.points.length - 1];
    if (last) {
      const [lx, ly] = t.toC(last[0] * c, last[1] * c);
      ctx.save();
      ctx.setLineDash([6, 4]);
      ctx.strokeStyle = cssVar('--muted');
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(lx, ly); ctx.lineTo(x, y); ctx.stroke();
      ctx.restore();
      const len = (Math.abs(hover[0] - last[0]) + Math.abs(hover[1] - last[1])) * c;
      ctx.fillStyle = cssVar('--text');
      ctx.font = '11px ' + cssVar('--mono');
      ctx.fillText(`${len} mm`, x + 8, y - 8);
    }
    ctx.strokeStyle = cssVar('--accent');
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.stroke();
  }
}

function syncGridInputs() {
  $('gridCell').value = store.grid.cell_mm;
  $('gridCols').value = store.grid.cols;
  $('gridRows').value = store.grid.rows;
}

export function initMap() {
  canvas = $('mapCanvas');
  syncGridInputs();
  canvas.addEventListener('mousemove', (e) => {
    const r = canvas.getBoundingClientRect();
    hover = snap(e.clientX - r.left, e.clientY - r.top);
    draw();
  });
  canvas.addEventListener('mouseleave', () => { hover = null; draw(); });
  canvas.addEventListener('click', (e) => {
    const r = canvas.getBoundingClientRect();
    const q = snap(e.clientX - r.left, e.clientY - r.top);
    const last = store.points[store.points.length - 1];
    if (last && last[0] === q[0] && last[1] === q[1]) return;
    if (crossesPath(last, q)) {
      toast('Không cho vẽ đường khép kín hoặc tự cắt: điểm mới chạm lại đường đã vẽ', 'err');
      return;
    }
    store.points.push(q);
    changed();
  });
  canvas.addEventListener('contextmenu', (e) => { e.preventDefault(); store.points.pop(); changed(); });
  window.addEventListener('keydown', (e) => {
    if (!document.querySelector('#tab-map.active')) return;
    if (e.key === 'Backspace' && !['INPUT', 'SELECT'].includes(document.activeElement?.tagName)) {
      e.preventDefault(); store.points.pop(); changed();
    }
  });
  $('mapUndo').onclick = () => { store.points.pop(); changed(); };
  $('mapClear').onclick = () => { store.points = []; changed(); };
  $('mapPreset').onchange = (e) => {
    if (PRESETS[e.target.value]) store.points = PRESETS[e.target.value].map((p) => [...p]);
    e.target.value = '';
    changed();
  };
  for (const id of ['gridCell', 'gridCols', 'gridRows']) {
    $(id).onchange = () => {
      store.grid = { cell_mm: Number($('gridCell').value) || 100, cols: Number($('gridCols').value) || 30,
        rows: Number($('gridRows').value) || 18 };
      changed();
    };
  }
  $('mapExport').onclick = () => downloadJSON('path-project.json', { grid: store.grid, points: store.points });
  $('mapImport').onchange = async (e) => {
    const f = e.target.files[0];
    if (!f) return;
    try {
      const p = JSON.parse(await f.text());
      if (p.grid && Array.isArray(p.points)) { store.grid = p.grid; store.points = p.points; syncGridInputs(); changed(); }
    } catch { /* file lỗi */ }
    e.target.value = '';
  };
  $('mapRun').onclick = () => emit('run-request');
  new ResizeObserver(() => draw()).observe(canvas);
  on('tab', (t) => { if (t === 'map') { syncGridInputs(); validate(); } });
  on('shared', () => validate());
  validate();
}
