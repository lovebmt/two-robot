// Tab Thiết bị: danh sách robot (sim/thật), gán vai trò, identify, bật/tắt mô phỏng.
import { api, guard, toast } from './api.js';
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
      <td class="mono">${r.ip ?? '—'}${r.link?.rtt_ms != null ? `<div class="muted small">${r.link.rtt_ms.toFixed(0)} ms${r.link.errors ? ' · lỗi ' + r.link.errors : ''}</div>` : ''}</td>
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
          <button class="btn small" data-reset="${r.id}">Reset</button>
          ${r.source === 'REAL' ? `<button class="btn small" data-del="${r.id}">Ngắt</button>` : ''}</td>
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
  tb.querySelectorAll('[data-del]').forEach((b) => {
    b.onclick = async () => { await guard(() => api('DELETE', `/api/robots/${b.dataset.del}`), 'Đã ngắt'); refreshRobots(); };
  });
}

async function doScan() {
  const raw = $('scanTargets').value.trim() || $('scanTargets').placeholder;
  const targets = raw.split(',').map((x) => x.trim()).filter(Boolean);
  $('scanBtn').disabled = true;
  $('scanBtn').textContent = 'Đang quét…';
  try {
    const r = await guard(() => api('POST', '/api/discover', { targets }));
    if (r) {
      const names = r.found.map((f) => `${f.name} (${f.base})`).join(', ');
      if (r.found.length) toast(`Thấy ${r.found.length} robot: ${names}`, 'ok'); else toast('Không thấy robot nào', 'err');
    }
  } finally {
    $('scanBtn').disabled = false;
    $('scanBtn').textContent = 'Quét';
    refreshRobots();
  }
}

export function initDevices() {
  $('scanBtn').onclick = doScan;
  $('scanTargets').addEventListener('keydown', (e) => { if (e.key === 'Enter') doScan(); });
  api('GET', '/api/discover/default').then((d) => { $('scanTargets').placeholder = d.subnet; }).catch(() => {});
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
