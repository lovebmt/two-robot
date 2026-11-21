// Trạng thái dùng chung giữa các tab + pub/sub đơn giản.

const LS_KEY = 'tworobot.project.v1';

export const store = {
  schema: null,
  frame: null,       // TelemetryFrame mới nhất
  robots: [],        // tóm tắt từ /api/robots hoặc frame
  grid: { cell_mm: 100, cols: 30, rows: 18 },
  points: [],        // đỉnh lưới [[i, j], ...]
  mode: 'pair',      // 'pair' = 2 xe · 'solo' = 1 xe (chỉ leader chạy theo đường)
  plan: null,        // kết quả /api/plan
  shared: null,
};

const subs = new Map();

export function on(evt, fn) {
  if (!subs.has(evt)) subs.set(evt, new Set());
  subs.get(evt).add(fn);
}

export function emit(evt, data) {
  for (const fn of subs.get(evt) || []) {
    try { fn(data); } catch (e) { console.error(evt, e); }
  }
}

export function saveProject() {
  try {
    localStorage.setItem(LS_KEY, JSON.stringify({ grid: store.grid, points: store.points, mode: store.mode }));
  } catch { /* storage bị chặn: bỏ qua */ }
}

export function loadProject() {
  try {
    const p = JSON.parse(localStorage.getItem(LS_KEY) || 'null');
    if (p && p.grid && Array.isArray(p.points)) {
      store.grid = p.grid;
      store.points = p.points;
      if (['solo', 'pair', 'follow'].includes(p.mode)) store.mode = p.mode;
    }
  } catch { /* bỏ qua */ }
}

export function downloadJSON(name, obj) {
  const blob = new Blob([JSON.stringify(obj, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
