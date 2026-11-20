// Điểm vào: tabs, WebSocket, khởi tạo các trang.
import { api, connectWS, guard, toast } from './api.js';
import { initConfig } from './config.js';
import { initDevices, refreshRobots } from './devices.js';
import { initMap } from './mapedit.js';
import { emit, loadProject, on, store } from './store.js';
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

  const TABS = ['devices', 'config', 'map', 'run'];
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
