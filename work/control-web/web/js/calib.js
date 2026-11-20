// Tab Hiệu chuẩn: IMU, đi thẳng (đường kính bánh), quay (gyro scale + track), ToF quét tường, khoảng cách ToF.
// Luồng: Bước 1 robot thực hiện → hiện số đo của robot (+ số đo thật nếu là robot ảo) →
//        người dùng nhập số đo thực tế → "Tính & lưu" (server tính, ghi config, lưu flash).
import { api, guard, toast } from './api.js';
import { on, store } from './store.js';
import { calibTof } from './view.js';

const $ = (id) => document.getElementById(id);
let target = null;

const f = (v, d = 1) => (v === null || v === undefined ? '—' : Number(v).toFixed(d));
const kv = (rows) => `<dl class="kv">${rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('')}</dl>`;

function renderTargets() {
  const box = $('calTargets');
  if (!store.robots.length) { box.innerHTML = '<span class="muted">Chưa có robot</span>'; target = null; return; }
  if (!store.robots.some((r) => r.id === target)) target = store.robots[0].id;
  box.innerHTML = store.robots.map((r) => `<button data-t="${r.id}" class="${r.id === target ? 'active' : ''}">${r.name}${r.role ? ' · ' + r.role : ''}</button>`).join('');
  box.querySelectorAll('button').forEach((b) => { b.onclick = () => { target = b.dataset.t; renderTargets(); }; });
}

// Bước 2: ô nhập số đo thật + nút Tính & lưu
function measureForm(kind, label, unit, prefill, extra = '') {
  return `<div class="row gap wrap">
    <label>${label} <input type="number" step="any" id="calMeas-${kind}" class="num" value="${prefill ?? ''}"> ${unit}</label>
    <button class="btn primary" id="calApply-${kind}">Tính &amp; lưu</button></div>${extra}
    <div id="calRes-${kind}"></div>`;
}

async function apply(kind, extraBody = {}) {
  const measured = Number($(`calMeas-${kind}`).value);
  if (!Number.isFinite(measured) || !$(`calMeas-${kind}`).value) { toast('Nhập số đo thực tế', 'err'); return; }
  const r = await guard(() => api('POST', `/api/robots/${target}/cal/${kind}/apply`, { measured, ...extraBody }));
  if (!r) return;
  const i = r.info;
  let rows = [];
  if (kind === 'drive') rows = [['Sai trước hiệu chuẩn', `${f(i.err_before_pct, 2)} %`], ['Đường kính', `${f(i.diameter_old_mm, 2)} → <b>${f(i.diameter_new_mm, 2)}</b> mm`]];
  if (kind === 'turn') rows = [['Sai mỗi 90°', `${f(i.err_before_deg_per_90, 2)}°`], ['gyro_scale_z', `${f(i.gyro_scale_old, 4)} → <b>${f(i.gyro_scale_new, 4)}</b>`],
    ['track_eff', `${f(i.track_eff_old_mm)} → <b>${f(i.track_eff_new_mm)}</b> mm`], ...(i.track_warning ? [['!', i.track_warning]] : [])];
  if (kind === 'gap') rows = [['Sai trước hiệu chuẩn', `${f(i.err_before_mm)} mm`], ['Offset trái/phải', `${i.offset_old.map((x) => f(x)).join(' / ')} → <b>${i.offset_new.map((x) => f(x)).join(' / ')}</b> mm`]];
  $(`calRes-${kind}`).innerHTML = kv(rows) + '<span class="ok-text">Đã lưu flash</span>';
  toast('Đã hiệu chuẩn và lưu flash', 'ok');
}

async function run(kind) {
  if (!target) { toast('Chưa có robot', 'err'); return; }
  const out = $(`calOut-${kind}`);
  if (kind === 'imu') {
    await guard(() => api('POST', `/api/robots/${target}/calib_imu`), 'Đang calib IMU — giữ xe đứng yên (xem kết quả ở tab Chạy / Sự kiện)');
    return;
  }
  if (kind === 'tofwall') {
    const src = store.robots.find((r) => r.id === target)?.source;
    await calibTof(target, src);
    return;
  }
  const body = kind === 'drive' ? { mm: Number($('calDriveMm').value) }
    : kind === 'turn' ? { deg: Number($('calTurnDeg').value) }
      : { known_mm: Number($('calGapMm').value) };
  out.innerHTML = '<span class="muted">Đang chạy…</span>';
  const r = await guard(() => api('POST', `/api/robots/${target}/cal/${kind}`, body));
  if (!r) { out.innerHTML = ''; return; }
  const x = r.result;
  const simNote = r.sim_measured !== null && r.sim_measured !== undefined
    ? `<p class="small muted">Robot ảo: đã điền sẵn số đo thật từ mô phỏng (thay cho thước).</p>` : '';
  if (kind === 'drive') {
    out.innerHTML = kv([['Lệnh', `${f(x.cmd_mm, 0)} mm`], ['Encoder đo', `${f(x.enc_mm)} mm`], ['Hướng đổi (gyro)', `${f(x.yaw_change_deg, 2)}°`]])
      + simNote + measureForm('drive', 'Quãng thật (vạch A → B)', 'mm', r.sim_measured);
    $('calApply-drive').onclick = () => apply('drive');
  } else if (kind === 'turn') {
    out.innerHTML = kv([['Lệnh', `${f(x.cmd_deg, 0)}°`], ['Gyro ước lượng', `${f(x.est_deg, 2)}°`],
      ['Cung bánh trái/phải', `${f(x.arc_l_mm, 0)} / ${f(x.arc_r_mm, 0)} mm`]])
      + simNote + measureForm('turn', 'Góc thật', '°', r.sim_measured, `
      <details class="small"><summary>Tính góc từ thước (L, d)</summary>
        <div class="row gap wrap">
          <label>L <input type="number" id="calTL" class="num" value="150"> mm</label>
          <label>d <input type="number" id="calTd" class="num" value="0"> mm</label>
          <select id="calTdir"><option value="1">quay lố qua vạch</option><option value="-1">quay chưa tới vạch</option></select>
          <button class="btn small" id="calTcalc">→ điền góc thật</button>
        </div>
        <p class="muted">L = khoảng cách giữa 2 điểm đo dọc mặt bên thân xe; d = chênh lệch khoảng cách từ vạch tới mép xe giữa 2 điểm đó. Góc lệch = atan(d / L).</p>
      </details>`);
    $('calApply-turn').onclick = () => apply('turn');
    $('calTcalc').onclick = () => {
      const L = Number($('calTL').value), d = Number($('calTd').value), dir = Number($('calTdir').value);
      if (!(L > 0)) { toast('L phải > 0', 'err'); return; }
      const dev = Math.atan(Math.abs(d) / L) * 180 / Math.PI;
      $('calMeas-turn').value = (Number(x.cmd_deg) + dir * dev).toFixed(2);
    };
  } else {
    const known = Number($('calGapMm').value);
    out.innerHTML = kv([['Mắt trái / phải đo', `${f(x.dl_mm)} / ${f(x.dr_mm)} mm`], ['Δψ', `${f(x.dpsi_deg, 2)}°`],
      ['Nhiễu (σ gap)', `${f(x.std)} mm`]])
      + (Math.abs(x.dpsi_deg) > 2 ? '<p class="err small">Δψ > 2°: 2 xe chưa thẳng hàng — nếu bỏ "Giữ chênh lệch 2 mắt" kết quả sẽ sai.</p>' : '')
      + simNote + measureForm('gap', 'Khoảng cách thật (mặt kính ToF → mặt đuôi xe trước)', 'mm', r.sim_measured ?? known);
    $('calApply-gap').onclick = () => apply('gap', { keep_diff: $('calGapKeep').checked });
  }
}

export function initCalib() {
  document.querySelectorAll('[data-cal]').forEach((b) => { b.onclick = () => run(b.dataset.cal); });
  on('robots', () => renderTargets());
  on('tab', (t) => { if (t === 'calib') renderTargets(); });
  renderTargets();
}
