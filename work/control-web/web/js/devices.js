// Tab Thiết bị: danh sách robot (sim/thật), gán vai trò, identify, bật/tắt mô phỏng.
import { api, guard } from './api.js';
import { emit, store } from './store.js';

const $ = (id) => document.getElementById(id);

function flagChips(flags = []) {
  return flags.map((f) => {
    const cls = ['LEADER_LOST', 'LINK_LOST', 'BAT_LOW', 'CFG_DEFAULT', 'IMU_BIAS_HIGH'].includes(f) ? 'bad'
      : ['SAT', 'SLIP', 'CFG_DIRTY', 'IMU_UNCAL', 'IMU_CALIB'].includes(f) ? 'warn' : '';
    return `<span class="chip ${cls}">${f}</span>`;
  }).join('');
}

export async function refreshRobots() {
  const list = await guard(() => api('GET', '/api/robots'));
  if (!list) return;
  store.robots = list;
  render();
  emit('robots', list);
}

function render() {
  const tb = $('devTable').querySelector('tbody');
  $('devCount').textContent = `${store.robots.length} robot`;
  if (!store.robots.length) {
    tb.innerHTML = '<tr><td colspan="8" class="muted">Chưa có robot. Bật mô phỏng hoặc cắm robot thật vào cùng LAN.</td></tr>';
    return;
  }
  tb.innerHTML = store.robots.map((r) => `
    <tr>
      <td><b>${r.name}</b><div class="muted small mono">${r.id}${r.fw ? ' · fw ' + r.fw : ''}</div></td>
      <td><span class="badge ${r.source}">${r.source}</span></td>
      <td class="mono">${r.ip ?? '—'}</td>
      <td>${r.online ? '🟢' : '⚪'} ${r.state}${r.fault ? ` <span class="chip bad">${r.fault}</span>` : ''}</td>
      <td>
        <select data-role="${r.id}">
          <option value="" ${!r.role ? 'selected' : ''}>—</option>
          <option value="leader" ${r.role === 'leader' ? 'selected' : ''}>Đầu (leader)</option>
          <option value="follower" ${r.role === 'follower' ? 'selected' : ''}>Đuôi (follower)</option>
        </select>
      </td>
      <td class="mono">${r.cfg_rev ?? '—'}</td>
      <td>${flagChips(r.flags)}</td>
      <td><button class="btn small" data-ident="${r.id}">Identify</button>
          <button class="btn small" data-reset="${r.id}">Reset</button></td>
    </tr>`).join('');
  tb.querySelectorAll('select[data-role]').forEach((sel) => {
    sel.onchange = async () => {
      await guard(() => api('PUT', `/api/robots/${sel.dataset.role}/role`, { role: sel.value || null }), 'Đã đổi vai trò');
      refreshRobots();
    };
  });
  tb.querySelectorAll('[data-ident]').forEach((b) => {
    b.onclick = () => guard(() => api('POST', `/api/robots/${b.dataset.ident}/identify`), 'Robot đang nháy (xem tab Chạy)');
  });
  tb.querySelectorAll('[data-reset]').forEach((b) => {
    b.onclick = () => guard(() => api('POST', `/api/robots/${b.dataset.reset}/reset`), 'Đã reset');
  });
}

export function initDevices() {
  $('simStart').onclick = async () => {
    await guard(() => api('POST', '/api/sim/start', { seed: Number($('simSeed').value) || 1, grid: store.grid }),
      'Đã bật 2 robot ảo');
    refreshRobots();
  };
  $('simStop').onclick = async () => {
    await guard(() => api('POST', '/api/sim/stop'), 'Đã tắt mô phỏng');
    refreshRobots();
  };
  setInterval(() => { if (document.querySelector('#tab-devices.active')) refreshRobots(); }, 1500);
}
