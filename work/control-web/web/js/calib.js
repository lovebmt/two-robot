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

// Bước 0: jog 1 s (gửi lặp mỗi 150 ms vì lệnh jog tự hết hạn sau 300 ms), so trước/sau từ telemetry.
async function dirTest(kind) {
  if (!target) { toast('Chưa có robot', 'err'); return; }
  const out = $('calOut-dir');
  const tel = () => (store.frame?.robots || []).find((r) => r.id === target);
  const r0 = tel();
  if (!r0?.est) { toast('Chưa có telemetry của robot này', 'err'); return; }
  const s0 = r0.est.s, y0 = r0.est.yaw;
  const v = kind === 'fwd' ? 80 : 0, w = kind === 'fwd' ? 0 : 1.2;
  let maxL = 0, maxR = 0;
  out.innerHTML = '<span class="muted">Đang chạy 1 s…</span>';
  for (let i = 0; i < 7; i++) {
    const ok = await guard(() => api('POST', `/api/robots/${target}/jog`, { v, w }));
    if (!ok) return;
    await new Promise((res) => setTimeout(res, 150));
    const t = tel();
    if (t?.wheels) {
      if (Math.abs(t.wheels.l.v) > Math.abs(maxL)) maxL = t.wheels.l.v;
      if (Math.abs(t.wheels.r.v) > Math.abs(maxR)) maxR = t.wheels.r.v;
    }
  }
  await new Promise((res) => setTimeout(res, 500));
  const r1 = tel();
  const ds = r1.est.s - s0;
  let dy = (r1.est.yaw - y0) * 180 / Math.PI;
  dy = ((dy + 540) % 360) - 180;
  const mark = (ok) => (ok ? '<span class="ok-text">✓</span>' : '<b class="err">✗</b>');
  if ((r1.fault || '').startsWith('WHEEL_DIR')) {
    const side = r1.fault.endsWith('L') ? 'trái' : 'phải';
    out.innerHTML = `<p class="err">Firmware đã PHANH: bánh ${side} quay NGƯỢC chiều lệnh (${r1.fault}). `
      + `Motor hoặc encoder bánh ${side} đấu ngược. Nhìn bánh khi bấm lại: bánh quay lùi → bật <code>motor.invert_${side === 'trái' ? 'l' : 'r'}</code>; `
      + `bánh quay tiến → bật <code>enc.invert_${side === 'trái' ? 'l' : 'r'}</code>. Đã tự Reset lỗi.</p>`;
    await guard(() => api('POST', `/api/robots/${target}/reset`));
    return;
  }
  if (kind === 'fwd') {
    out.innerHTML = kv([
      ['Encoder trái (tốc độ đo)', `${f(maxL)} mm/s ${mark(maxL > 5)}`],
      ['Encoder phải (tốc độ đo)', `${f(maxR)} mm/s ${mark(maxR > 5)}`],
      ['Quãng s thay đổi', `${f(ds)} mm`]])
      + (maxL > 5 && maxR > 5 ? '<p class="ok-text small">Encoder đúng chiều. Nhớ kiểm tra bằng mắt: 2 bánh đều quay TIẾN.</p>'
        : `<p class="err small">Encoder ${maxL <= 5 ? 'trái' : ''} ${maxR <= 5 ? 'phải' : ''} không đo được chiều tiến: nếu bánh đó quay tiến → bật <code>enc.invert</code>; nếu quay lùi → bật <code>motor.invert</code>; nếu không quay/không đếm → kiểm tra dây motor/encoder, kéo lên 3.3 V.</p>`);
  } else {
    out.innerHTML = kv([['Góc thay đổi (gyro)', `${f(dy, 1)}° ${mark(dy > 3)}`]])
      + (dy > 3 ? '<p class="ok-text small">Gyro đúng chiều (quay trái → góc tăng).</p>'
        : dy < -3 ? '<p class="err small">Góc GIẢM khi quay trái → bật <code>imu.invert_z</code> (MPU gắn úp) rồi Save. Nếu xe thực tế quay PHẢI → kiểm tra lại invert motor.</p>'
          : '<p class="err small">Góc gần như không đổi: xe chưa quay (bánh trượt/kê gầm) hoặc IMU lỗi (cờ HW_IMU).</p>');
  }
}

// 0b: hiệu chuẩn encoder quay tay — mốc tick → người dùng quay N vòng tới → tính → lưu
let encSide = 'l';
let encTimer = null;
let encStartTicks = null;

function encTicks() {
  const r = (store.frame?.robots || []).find((x) => x.id === target);
  return r?.enc ? r.enc[encSide] : null;
}

async function encStart() {
  if (!target) { toast('Chưa có robot', 'err'); return; }
  const r = await guard(() => api('POST', `/api/robots/${target}/cal_enc/start`, { side: encSide }));
  if (!r) return;
  encStartTicks = r.start_ticks;
  $('encOk').disabled = false;
  $('calOut-enc').innerHTML = '';
  clearInterval(encTimer);
  encTimer = setInterval(() => {
    const t = encTicks();
    $('encLive').innerHTML = t === null ? '<span class="muted">chưa có tick từ robot</span>'
      : `Bánh ${encSide === 'l' ? 'TRÁI' : 'PHẢI'}: <b>${t - encStartTicks >= 0 ? '+' : ''}${t - encStartTicks}</b> tick — quay tay đúng ${$('encRevs').value} vòng về phía trước rồi bấm OK`;
  }, 150);
}

async function encOk() {
  clearInterval(encTimer);
  const r = await guard(() => api('POST', `/api/robots/${target}/cal_enc/finish`, { side: encSide, revs: Number($('encRevs').value) || 2 }));
  if (!r) return;
  const i = r.info;
  const sideName = encSide === 'l' ? 'trái' : 'phải';
  $('calOut-enc').innerHTML = kv([
    ['Tick đếm được', `${i.delta_ticks} trong ${i.revs} vòng`],
    ['Tick / vòng bánh', `<b>${f(i.ticks_per_rev, 1)}</b>`],
    ['Tỉ số hộp số (PPR ' + i.ppr + ')', `${f(i.gear_ratio, 2)} (gần chuẩn GA25: 1:${i.nearest_standard}) · đang dùng ${f(i.gear_ratio_old, 2)}`],
    ['Tỉ số sẽ lưu', `<b>${f(i.gear_ratio_used, 2)}</b>${i.gear_ratio_used !== i.gear_ratio ? ' (trung bình 2 bánh)' : ''}`],
    ['mm / tick', f(i.mm_per_tick_new, 4)],
    ['Chiều encoder ' + sideName, i.direction_ok ? '<span class="ok-text">✓ đúng (quay tới → tick tăng)</span>'
      : `<b class="err">ngược</b> → sẽ ${i.invert_new ? 'BẬT' : 'TẮT'} <code>enc.invert_${encSide}</code>`],
  ]) + '<button class="btn primary" id="encSave">Lưu vào robot</button>';
  $('encSave').onclick = async () => {
    const s = await guard(() => api('POST', `/api/robots/${target}/cal_enc/save`, { side: encSide }), `Đã lưu encoder ${sideName} vào flash`);
    if (s) $('calOut-enc').insertAdjacentHTML('beforeend', ' <span class="ok-text">Đã lưu</span>');
  };
  $('encOk').disabled = true;
}

export function initCalib() {
  $('encStart').onclick = encStart;
  $('encOk').onclick = encOk;
  $('encSide').querySelectorAll('button').forEach((b) => {
    b.onclick = () => {
      encSide = b.dataset.side;
      $('encSide').querySelectorAll('button').forEach((x) => x.classList.toggle('active', x === b));
      clearInterval(encTimer);
      $('encLive').innerHTML = '';
      $('encOk').disabled = true;
    };
  });
  document.querySelectorAll('[data-dir]').forEach((b) => { b.onclick = () => dirTest(b.dataset.dir); });
  document.querySelectorAll('[data-cal]').forEach((b) => { b.onclick = () => run(b.dataset.cal); });
  on('robots', () => renderTargets());
  on('tab', (t) => { if (t === 'calib') renderTargets(); });
  renderTargets();
}
