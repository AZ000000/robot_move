/* =============================================================
 * render.js — Canvas 绘制
 *
 *   世界坐标 → 画布坐标：
 *     cam 中心跟着底盘，画面永远显示底盘附近 ±3m 范围
 *     1m = pixelsPerMeter 像素（动态调整）
 * ============================================================= */

const Render = (() => {

  let canvas, ctx, width, height;
  const WORLD_MARGIN = 0.8;   /* 视野外边距 m */

  function init() {
    canvas = document.getElementById('sim-canvas');
    ctx = canvas.getContext('2d');
    resize();
    window.addEventListener('resize', resize);
  }

  function resize() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    width  = rect.width;
    height = rect.height;
    canvas.width  = Math.floor(width  * dpr);
    canvas.height = Math.floor(height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  /* 世界 m → 画布 px */
  function worldToCanvas(wx, wy, cx, cy, ppm) {
    const px = width/2  + (wx - cx) * ppm;
    const py = height/2 + (wy - cy) * ppm;
    return [px, py];
  }

  function draw(state) {
    const s = state;
    const cx = s.phys.x, cy = s.phys.y;
    const targetSpanX = Math.max(0.6, Math.abs(cx) + WORLD_MARGIN);
    const targetSpanY = Math.max(0.6, Math.abs(cy) + WORLD_MARGIN);
    const ppm = Math.min(width / (targetSpanX * 2), height / (targetSpanY * 2));

    ctx.clearRect(0, 0, width, height);

    /* ---- 背景 ---- */
    const grad = ctx.createLinearGradient(0, 0, 0, height);
    grad.addColorStop(0, '#0b1220');
    grad.addColorStop(1, '#090f1c');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, width, height);

    /* ---- 网格 ---- */
    ctx.strokeStyle = 'rgba(56,189,248,0.15)';
    ctx.lineWidth = 1;
    const gridStep = (ppm > 80) ? 0.5 : 1.0;
    const startX = Math.floor(cx / gridStep) * gridStep - 5;
    const endX   = startX + 10;
    const startY = Math.floor(cy / gridStep) * gridStep - 5;
    const endY   = startY + 10;
    for (let gx = startX; gx <= endX; gx += gridStep) {
      const [px1, py1] = worldToCanvas(gx, cy - 4, cx, cy, ppm);
      const [px2, py2] = worldToCanvas(gx, cy + 4, cx, cy, ppm);
      ctx.beginPath(); ctx.moveTo(px1, py1); ctx.lineTo(px2, py2); ctx.stroke();
    }
    for (let gy = startY; gy <= endY; gy += gridStep) {
      const [px1, py1] = worldToCanvas(cx - 4, gy, cx, cy, ppm);
      const [px2, py2] = worldToCanvas(cx + 4, gy, cx, cy, ppm);
      ctx.beginPath(); ctx.moveTo(px1, py1); ctx.lineTo(px2, py2); ctx.stroke();
    }

    /* 0,0 十字标 */
    const [ozx, ozy] = worldToCanvas(0, 0, cx, cy, ppm);
    ctx.strokeStyle = 'rgba(245,158,11,0.5)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(ozx - 8, ozy); ctx.lineTo(ozx + 8, ozy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(ozx, ozy - 8); ctx.lineTo(ozx, ozy + 8); ctx.stroke();
    ctx.fillStyle = 'rgba(245,158,11,0.8)';
    ctx.font = '11px "JetBrainsMono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText('origin', ozx, ozy - 12);

    /* ---- 轨迹残影 ---- */
    const hist = s.phys.poseHistory;
    if (hist.length > 1) {
      ctx.beginPath();
      for (let i = 0; i < hist.length; i++) {
        const [px, py] = worldToCanvas(hist[i].x, hist[i].y, cx, cy, ppm);
        if (i === 0) ctx.moveTo(px, py);
        else         ctx.lineTo(px, py);
      }
      ctx.strokeStyle = 'rgba(56,189,248,0.55)';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    /* ---- 底盘主体 ---- */
    drawChassis(s, cx, cy, ppm);
  }

  /* 画底盘：主体 + 两条履带 + 朝向箭头 + 轮子 */
  function drawChassis(s, cx, cy, ppm) {
    const wb  = PHYS.CFG.WHEELBASE_M;
    const track_w = PHYS.CFG.TRACK_WIDTH_M;
    const track_l = PHYS.CFG.TRACK_LENGTH_M;
    const cw  = Math.max(0.04, wb - 2 * track_w - 0.04);   /* 车体宽度 ≈ 中心距 - 两侧履带宽 - 间隙 */
    const cl  = track_l + 0.03;                              /* 车体长度略长于履带 */
    const theta = s.phys.theta;

    // 四条关键点：左前后、右前后
    // 相对底盘中心的坐标（局部坐标系，朝 +x 为前）
    const local = [
      [-cl/2, -wb/2 - track_w/2],  /* 左履带外前 */
      [ cl/2, -wb/2 - track_w/2],  /* 左履带外后 */
      [-cl/2,  wb/2 + track_w/2],
      [ cl/2,  wb/2 + track_w/2],
      [ cl/2, -wb/2],              /* 右车体前右 */
      [ cl/2,  wb/2],
      [-cl/2,  wb/2],
      [-cl/2, -wb/2],
    ];

    function rot(x, y) {
      return [
        x * Math.cos(theta) - y * Math.sin(theta),
        x * Math.sin(theta) + y * Math.cos(theta)
      ];
    }

    function wc([x, y]) { return worldToCanvas(x + cx, y + cy, cx, cy, ppm); }

    /* 左履带 */
    ctx.save();
    ctx.fillStyle = '#1a2440';
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const tl_p0 = wc(rot(-track_l/2, -wb/2 - track_w/2));
    const tl_p1 = wc(rot( track_l/2, -wb/2 - track_w/2));
    const tl_p2 = wc(rot( track_l/2, -wb/2 + track_w/2));
    const tl_p3 = wc(rot(-track_l/2, -wb/2 + track_w/2));
    ctx.moveTo(tl_p0[0], tl_p0[1]);
    ctx.lineTo(tl_p1[0], tl_p1[1]);
    ctx.lineTo(tl_p2[0], tl_p2[1]);
    ctx.lineTo(tl_p3[0], tl_p3[1]);
    ctx.closePath(); ctx.fill(); ctx.stroke();

    /* 右履带 */
    const tr_p0 = wc(rot(-track_l/2, wb/2 + track_w/2));
    const tr_p1 = wc(rot( track_l/2, wb/2 + track_w/2));
    const tr_p2 = wc(rot( track_l/2, wb/2 - track_w/2));
    const tr_p3 = wc(rot(-track_l/2, wb/2 - track_w/2));
    ctx.beginPath();
    ctx.moveTo(tr_p0[0], tr_p0[1]);
    ctx.lineTo(tr_p1[0], tr_p1[1]);
    ctx.lineTo(tr_p2[0], tr_p2[1]);
    ctx.lineTo(tr_p3[0], tr_p3[1]);
    ctx.closePath(); ctx.fill(); ctx.stroke();

    /* 履带条纹（显示左右实际速度方向）*/
    drawTrackStripes(s.phys.vL, tl_p0, tl_p1, tl_p3, 'left');
    drawTrackStripes(s.phys.vR, tr_p0, tr_p1, tr_p3, 'right');
    ctx.restore();

    /* 车体主体 */
    ctx.save();
    ctx.fillStyle = '#111a2e';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    const bl0 = wc(rot(-cw/2, -cl/2));
    const bl1 = wc(rot( cw/2, -cl/2));
    const bl2 = wc(rot( cw/2,  cl/2));
    const bl3 = wc(rot(-cw/2,  cl/2));
    ctx.beginPath();
    ctx.moveTo(bl0[0], bl0[1]);
    ctx.lineTo(bl1[0], bl1[1]);
    ctx.lineTo(bl2[0], bl2[1]);
    ctx.lineTo(bl3[0], bl3[1]);
    ctx.closePath(); ctx.fill(); ctx.stroke();

    /* 朝向箭头 */
    const [ax, ay] = wc(rot(cl/2 + 0.06, 0));
    const [bx, by] = wc(rot(cl/2 - 0.01, 0));
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(ax, ay); ctx.stroke();
    // 箭头头
    const hx = ax - bx, hy = ay - by;
    const hl = Math.sqrt(hx*hx + hy*hy);
    const ux = hx/hl, uy = hy/hl;
    const a1x = ax - ux * 10 + uy * 6;
    const a1y = ay - uy * 10 - ux * 6;
    const a2x = ax - ux * 10 - uy * 6;
    const a2y = ay - uy * 10 + ux * 6;
    ctx.fillStyle = '#22c55e';
    ctx.beginPath();
    ctx.moveTo(ax, ay); ctx.lineTo(a1x, a1y); ctx.lineTo(a2x, a2y); ctx.closePath(); ctx.fill();

    /* 中心圆 */
    const [ccx, ccy] = wc(rot(0, 0));
    ctx.fillStyle = '#f59e0b';
    ctx.beginPath(); ctx.arc(ccx, ccy, 4, 0, Math.PI*2); ctx.fill();

    ctx.restore();

    /* ---- 速度矢量 ---- */
    drawVelocityArrow(s, cx, cy, ppm, theta);
  }

  function drawTrackStripes(v, pA, pB, pC, side) {
    // 在履带里画几条横条纹，方向由 v 决定（动画感）
    const stripe_color = v > 0 ? '#22c55e' : v < 0 ? '#ef4444' : '#64748b';
    ctx.strokeStyle = stripe_color;
    ctx.lineWidth = 2;
    const k = 0.2;
    const dx1 = (pB[0]-pA[0]) * k;
    const dy1 = (pB[1]-pA[1]) * k;
    const dx2 = (pC[0]-pA[0]) * k;
    const dy2 = (pC[1]-pA[1]) * k;
    for (let i = 1; i < 5; i++) {
      const a = [pA[0] + dx1*i, pA[1] + dy1*i];
      const b = [a[0] + dx2,   a[1] + dy2];
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    }
  }

  function drawVelocityArrow(s, cx, cy, ppm, theta) {
    const v = s.phys.vCenter;
    if (Math.abs(v) < 0.01) return;
    const len = Math.min(80, Math.abs(v) * 80);
    const ux = Math.cos(theta), uy = Math.sin(theta);
    const [sx, sy] = worldToCanvas(cx, cy, cx, cy, ppm);
    const ex = sx + ux * len * Math.sign(v);
    const ey = sy + uy * len * Math.sign(v);
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
    // 箭头头
    const ang = Math.atan2(ey-sy, ex-sx);
    ctx.fillStyle = '#fbbf24';
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - 10*Math.cos(ang - 0.4), ey - 10*Math.sin(ang - 0.4));
    ctx.lineTo(ex - 10*Math.cos(ang + 0.4), ey - 10*Math.sin(ang + 0.4));
    ctx.closePath(); ctx.fill();
  }

  return { init, draw };
})();
