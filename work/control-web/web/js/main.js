// Điểm vào: tabs, WebSocket, khởi tạo các trang.
import { api, connectWS, guard, toast } from './api.js';
import { initCalib } from './calib.js';
import { initConfig } from './config.js';
import { initDevices, refreshRobots } from './devices.js';
import { initManual, stopManual } from './manual.js';
import { initMap, validate } from './mapedit.js';
import { emit, loadProject, on, saveProject, store } from './store.js';
import { initView, onFrame, startRun } from './view.js';

const TAB_KEY = 'tworobot.tab';

function showTab(name) {
  document.querySelectorAll('.tab').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.page').forEach((p) => p.classList.toggle('active', p.id === `tab-${name}`));
  try { localStorage.setItem(TAB_KEY, name); } catch { /* bỏ qua */ }
  emit('tab', name);
}

async function boot() {
  loadProject();
  store.schema = await api('GET', '/api/schema');
  const st = await api('GET', '/api/state');
  store.shared = st.shared;
  store.robots = st.robots;

  initDevices();
  initConfig();
  initMap();
  initView();
  initManual();
  initCalib();

  // Chế độ chạy 2 xe / 1 xe: các ô chọn ở tab Bản đồ và tab Chạy đồng bộ với nhau
  const modeSel = document.querySelectorAll('[data-mode-select]');
  const applyModeClass = () => {
    document.body.classList.toggle('mode-follow', store.mode === 'follow');
    document.body.classList.toggle('mode-single', store.mode !== 'pair');  // 1 xe / bám tấm: hiện ô chọn xe
    document.body.classList.toggle('mode-solo', store.mode === 'solo');
  };
  applyModeClass();

  // Ô chọn xe (chế độ 1 xe / bám tấm): mọi xe đều chạy được, không theo vai trò
  const robotSel = document.querySelectorAll('[data-robot-select]');
  const renderRobotSel = () => {
    const rs = store.robots || [];
    if (!rs.some((r) => r.id === store.rid)) store.rid = rs[0]?.id ?? null;
    const opts = rs.map((r) => `<option value="${r.id}">${r.name}${r.online === false ? ' (offline)' : ''}</option>`).join('')
      || '<option value="">(chưa có xe)</option>';
    robotSel.forEach((el) => { el.innerHTML = opts; el.value = store.rid ?? ''; });
  };
  robotSel.forEach((el) => {
    el.onchange = () => {
      stopManual();
      store.rid = el.value || null;
      robotSel.forEach((o) => { o.value = el.value; });
      emit('selection', store.rid);
      saveProject();
      validate();
    };
  });
  on('robots', renderRobotSel);
  renderRobotSel();

  modeSel.forEach((el) => {
    el.value = store.mode;
    el.onchange = () => {
      stopManual();
      store.mode = el.value;
      modeSel.forEach((o) => { o.value = store.mode; });
      applyModeClass();
      emit('mode', store.mode);
      saveProject();
      validate();
      toast(store.mode === 'solo' ? 'Chế độ 1 xe: chỉ xe leader chạy theo đường'
        : store.mode === 'follow' ? 'Bám tấm: xe follower giữ khoảng cách tới tấm phẳng cầm tay (không cần vẽ đường)'
          : 'Chế độ 2 xe: leader + follower');
    };
  });

  document.querySelectorAll('.tab').forEach((b) => { b.onclick = () => showTab(b.dataset.tab); });
  document.getElementById('estopTop').onclick = () => guard(() => api('POST', '/api/run/estop'), 'E-STOP');
  on('run-request', async () => { showTab('run'); await startRun(); });

  let robotsKey = '';
  connectWS((msg) => {
    if (msg.type === 'frame') {
      onFrame(msg);
      const key = msg.robots.map((r) => `${r.id}:${r.role}`).join(',');
      if (key !== robotsKey) { robotsKey = key; refreshRobots(); }
    }
  }, (up) => {
    document.getElementById('wsDot').classList.toggle('on', up);
    if (!up) toast('Mất kết nối server, đang thử lại…', 'err');
  });

  const TABS = ['devices', 'config', 'map', 'calib', 'run'];
  let start = st.robots.length ? 'run' : 'devices';
  try { start = localStorage.getItem(TAB_KEY) || start; } catch { /* bỏ qua */ }
  const hash = location.hash.slice(1);
  if (TABS.includes(hash)) start = hash;
  window.addEventListener('hashchange', () => { const h = location.hash.slice(1); if (TABS.includes(h)) showTab(h); });
  showTab(start);
  refreshRobots();
}

boot().catch((e) => {
  console.error(e);
  document.body.insertAdjacentHTML('afterbegin', `<p class="err" style="padding:16px">Không khởi động được UI: ${e.message}</p>`);
});
