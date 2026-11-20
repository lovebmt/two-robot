// Đồ thị cuộn theo thời gian (canvas), không dùng thư viện.
import { cssVar, fitCanvas } from './draw.js';

export class Chart {
  constructor(canvas, { title, series, window = 30, unit = '', fixed = null }) {
    this.canvas = canvas;
    this.title = title;
    this.series = series; // [{key, label, color: '--chart-1', dash?}]
    this.window = window;
    this.unit = unit;
    this.fixed = fixed;   // [min, max] cố định hoặc null = tự co giãn
    this.data = new Map(series.map((s) => [s.key, []]));
    this.lastT = null;
  }

  clear() {
    for (const arr of this.data.values()) arr.length = 0;
    this.lastT = null;
  }

  push(t, values) {
    if (this.lastT !== null && t < this.lastT - 0.5) this.clear(); // thời gian lùi (reset sim/replay)
    this.lastT = t;
    for (const s of this.series) {
      const v = values[s.key];
      const arr = this.data.get(s.key);
      arr.push([t, v === undefined || v === null || Number.isNaN(v) ? null : v]);
      while (arr.length && arr[0][0] < t - this.window) arr.shift();
    }
  }

  draw() {
    const { ctx, w, h } = fitCanvas(this.canvas);
    ctx.clearRect(0, 0, w, h);
    const padL = 40, padR = 8, padT = 34, padB = 10;
    const t1 = this.lastT ?? 0;
    const t0 = t1 - this.window;
    let lo = Infinity, hi = -Infinity;
    for (const arr of this.data.values()) {
      for (const [, v] of arr) if (v !== null) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
    }
    if (this.fixed) [lo, hi] = this.fixed;
    if (!Number.isFinite(lo)) { lo = 0; hi = 1; }
    if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
    const m = (hi - lo) * 0.08;
    lo -= m; hi += m;
    const X = (t) => padL + ((t - t0) / this.window) * (w - padL - padR);
    const Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * (h - padT - padB);

    ctx.font = '10px ' + cssVar('--mono');
    ctx.fillStyle = cssVar('--muted');
    ctx.strokeStyle = cssVar('--grid-major');
    ctx.lineWidth = 1;
    for (let k = 0; k <= 3; k++) {
      const v = lo + ((hi - lo) * k) / 3;
      const y = Y(v);
      ctx.globalAlpha = 0.5;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.fillText(Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1), 2, y + 3);
    }
    ctx.fillStyle = cssVar('--text');
    ctx.font = '600 11px ' + cssVar('--font');
    ctx.fillText(`${this.title}${this.unit ? ' (' + this.unit + ')' : ''}`, padL, 13);

    // Chú thích ở dòng thứ 2, dưới tiêu đề
    let lx = padL;
    ctx.font = '10px ' + cssVar('--font');
    for (const s of this.series) {
      const tw = ctx.measureText(s.label).width;
      ctx.fillStyle = cssVar(s.color);
      ctx.fillRect(lx, 21, 10, 3);
      ctx.fillStyle = cssVar('--muted');
      ctx.fillText(s.label, lx + 13, 26);
      lx += tw + 24;
    }

    for (const s of this.series) {
      const arr = this.data.get(s.key);
      ctx.strokeStyle = cssVar(s.color);
      ctx.lineWidth = 1.6;
      ctx.setLineDash(s.dash ? [5, 4] : []);
      ctx.beginPath();
      let pen = false;
      for (const [t, v] of arr) {
        if (v === null) { pen = false; continue; }
        const x = X(t), y = Y(Math.max(lo, Math.min(hi, v)));
        if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }
}
