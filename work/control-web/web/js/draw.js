// Hàm vẽ dùng chung: lưới, đường, xe (thân hộp, bánh, mắt ToF), tia ToF.
// Toạ độ thế giới: mm, y hướng lên. Canvas: y hướng xuống.

export function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

// view: {cx, cy, zoom} tuỳ chọn (tâm theo mm, phóng to) cho chế độ bám camera.
export function makeTransform(w, h, grid, pad = 24, view = null) {
  const W = grid.cols * grid.cell_mm;
  const H = grid.rows * grid.cell_mm;
  let s = Math.min((w - 2 * pad) / W, (h - 2 * pad) / H);
  let ox = (w - W * s) / 2;
  let oy = (h - H * s) / 2;
  if (view) {
    s *= view.zoom;
    ox = w / 2 - view.cx * s;
    oy = h / 2 - (H - view.cy) * s;
  }
  return {
    s, W, H,
    toC: (x, y) => [ox + x * s, oy + (H - y) * s],
    toW: (cx, cy) => [(cx - ox) / s, H - (cy - oy) / s],
  };
}

export function drawGrid(ctx, T, grid) {
  const { cols, rows, cell_mm: c } = grid;
  ctx.lineWidth = 1;
  for (let i = 0; i <= cols; i++) {
    ctx.strokeStyle = i % 5 === 0 ? cssVar('--grid-major') : cssVar('--grid');
    const [x0, y0] = T.toC(i * c, 0);
    const [, y1] = T.toC(i * c, rows * c);
    ctx.beginPath(); ctx.moveTo(x0 + 0.5, y0); ctx.lineTo(x0 + 0.5, y1); ctx.stroke();
  }
  for (let j = 0; j <= rows; j++) {
    ctx.strokeStyle = j % 5 === 0 ? cssVar('--grid-major') : cssVar('--grid');
    const [x0, y0] = T.toC(0, j * c);
    const [x1] = T.toC(cols * c, j * c);
    ctx.beginPath(); ctx.moveTo(x0, y0 + 0.5); ctx.lineTo(x1, y0 + 0.5); ctx.stroke();
  }
  ctx.fillStyle = cssVar('--muted');
  ctx.font = '10px ' + cssVar('--mono');
  for (let i = 0; i <= cols; i += 5) {
    const [x, y] = T.toC(i * c, 0);
    ctx.fillText(`${(i * c / 1000).toFixed(1)}m`, x + 2, y - 3);
  }
}

export function drawPath(ctx, T, path, opts = {}) {
  if (!path || !path.segments || !path.segments.length) return;
  ctx.save();
  ctx.strokeStyle = opts.color || cssVar('--path');
  ctx.lineWidth = opts.width || 3;
  ctx.lineJoin = 'round';
  ctx.globalAlpha = opts.alpha ?? 1;
  ctx.beginPath();
  path.segments.forEach((sg, k) => {
    const [ax, ay] = T.toC(sg.a[0], sg.a[1]);
    const [bx, by] = T.toC(sg.b[0], sg.b[1]);
    if (k === 0) ctx.moveTo(ax, ay);
    ctx.lineTo(bx, by);
  });
  ctx.stroke();
  // Góc cua: số thứ tự + vòng quét
  path.segments.forEach((sg, k) => {
    if (!sg.turn_after) return;
    const [bx, by] = T.toC(sg.b[0], sg.b[1]);
    if (opts.sweep) {
      ctx.strokeStyle = cssVar('--muted');
      ctx.setLineDash([4, 4]);
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(bx, by, opts.sweep * T.s, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.fillStyle = opts.color || cssVar('--path');
    ctx.beginPath(); ctx.arc(bx, by, 9, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 10px ' + cssVar('--font');
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(String(k + 1), bx, by + 0.5);
    ctx.textAlign = 'start'; ctx.textBaseline = 'alphabetic';
  });
  ctx.restore();
}

// pose: {x, y, yaw}; geom: {length,width,front,rear,tof_x,tof_b,track,wheel_d}
export function drawRobot(ctx, T, pose, geom, opts = {}) {
  if (!pose || !geom) return;
  ctx.save();
  const [cx, cy] = T.toC(pose.x, pose.y);
  ctx.translate(cx, cy);
  ctx.rotate(-pose.yaw);
  ctx.scale(T.s, T.s);
  const hw = geom.width / 2;
  ctx.globalAlpha = opts.alpha ?? 1;
  ctx.lineWidth = (opts.lineWidth || 1.5) / T.s;
  if (opts.dashed) ctx.setLineDash([6 / T.s, 4 / T.s]);
  // thân (trục x tới trước = phải; y lên → canvas y xuống nên lật)
  ctx.beginPath();
  ctx.rect(-geom.rear, -hw, geom.front + geom.rear, geom.width);
  if (opts.fill) { ctx.fillStyle = opts.fill; ctx.fill(); }
  ctx.strokeStyle = opts.stroke || cssVar('--text');
  ctx.stroke();
  ctx.setLineDash([]);
  if (!opts.outlineOnly) {
    // bánh xe
    const wl = geom.wheel_d || 65;
    ctx.fillStyle = cssVar('--truth');
    const tw = 8;
    const ht = (geom.track || 130) / 2;
    ctx.fillRect(-wl / 2, -ht - tw / 2, wl, tw);
    ctx.fillRect(-wl / 2, ht - tw / 2, wl, tw);
    // mắt ToF
    ctx.fillStyle = opts.stroke || cssVar('--text');
    const b = (geom.tof_b || 60) / 2;
    for (const y of [-b, b]) {
      ctx.beginPath(); ctx.arc(geom.tof_x, y, 5, 0, Math.PI * 2); ctx.fill();
    }
    // mũi tên hướng
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2.5 / T.s;
    ctx.beginPath(); ctx.moveTo(-geom.rear * 0.4, 0); ctx.lineTo(geom.front * 0.6, 0); ctx.stroke();
  }
  ctx.restore();
  if (opts.label) {
    // Nhãn đặt trên (labelPos='above') hoặc dưới xe để 2 xe gần nhau không đè nhãn.
    ctx.save();
    ctx.fillStyle = opts.stroke || cssVar('--text');
    ctx.font = '600 11px ' + cssVar('--font');
    ctx.textAlign = 'center';
    const off = Math.max(geom.length, geom.width) / 2 * T.s + 8;
    ctx.fillText(opts.label, cx, opts.labelPos === 'below' ? cy + off + 8 : cy - off);
    ctx.restore();
  }
}

// Tia ToF: vẽ từ mắt cảm biến theo hướng xe, dài = số đo (status 0) hoặc vạch đỏ ngắn.
export function drawRays(ctx, T, pose, geom, tof) {
  if (!pose || !geom || !tof) return;
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  for (const side of ['l', 'r']) {
    const r = tof[side];
    if (!r) continue;
    const by = side === 'l' ? geom.tof_b / 2 : -geom.tof_b / 2;
    const ox = pose.x + geom.tof_x * c - by * s;
    const oy = pose.y + geom.tof_x * s + by * c;
    const ok = r.st === 0 && r.mm < 2000;
    const len = ok ? r.mm : 60;
    const [x0, y0] = T.toC(ox, oy);
    const [x1, y1] = T.toC(ox + len * c, oy + len * s);
    ctx.save();
    ctx.strokeStyle = ok ? cssVar('--ray-ok') : cssVar('--ray-bad');
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    if (ok) { ctx.beginPath(); ctx.arc(x1, y1, 2.5, 0, Math.PI * 2); ctx.fillStyle = ctx.strokeStyle; ctx.fill(); }
    ctx.restore();
  }
}
