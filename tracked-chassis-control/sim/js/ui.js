/* =============================================================
 * ui.js — 右侧控制面板 + 状态栏
 * ============================================================= */

const UI = (() => {

  let inputEl, btnEl, logEl, statusEl, presetsEl;

  function init() {
    inputEl   = document.getElementById('cmd-input');
    btnEl     = document.getElementById('cmd-send');
    logEl     = document.getElementById('log');
    statusEl  = document.getElementById('status-body');
    presetsEl = document.querySelectorAll('.preset-btn');

    inputEl.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') send();
    });
    btnEl.addEventListener('click', send);

    presetsEl.forEach(b => b.addEventListener('click', () => {
      Sim.send(b.dataset.cmd);
      appendLog('preset → ' + b.dataset.cmd);
    }));

    document.getElementById('btn-reset').addEventListener('click', () => {
      const r = Sim.send('CMD:RESET');
      appendLog(r.msg);
    });
    document.getElementById('btn-stop').addEventListener('click', () => {
      const r = Sim.send('CMD:STOP');
      appendLog(r.msg);
    });
    document.getElementById('btn-start').addEventListener('click', () => {
      Sim.start();
      appendLog('▶ 仿真已启动');
    });
  }

  function send() {
    const text = inputEl.value.trim();
    if (!text) return;
    const r = Sim.send(text);
    if (r.ok) appendLog('✓ ' + r.msg);
    else      appendLog('✗ ' + r.msg);
    inputEl.select();
  }

  function appendLog(msg) {
    const t = new Date();
    const ts = `${String(t.getHours()).padStart(2,'0')}:${String(t.getMinutes()).padStart(2,'0')}:${String(t.getSeconds()).padStart(2,'0')}`;
    const div = document.createElement('div');
    div.innerHTML = `<span style="color:#7b8aa6">[${ts}]</span> ${msg}`;
    logEl.prepend(div);
    while (logEl.children.length > 20) logEl.removeChild(logEl.lastChild);
  }

  /* 每帧刷一次状态表 */
  function updateStatus(s) {
    const p = s.phys;
    const modeColor = { DIFF:'#38bdf8', BRAKE:'#f59e0b', PIVOT:'#ef4444' }[s.mode] || '#38bdf8';
    const modeLabel = { DIFF:'平滑差速', BRAKE:'单边制动', PIVOT:'原地反转' }[s.mode] || s.mode;

    statusEl.innerHTML = `
      <tr><td>位姿 x / y / θ</td><td><b>${p.x.toFixed(2)}</b> m / <b>${p.y.toFixed(2)}</b> m / <b>${(p.theta*180/Math.PI).toFixed(1)}°</b></td></tr>
      <tr><td>中心线速度 v</td><td><b style="color:#22c55e">${p.vCenter.toFixed(3)}</b> m/s</td></tr>
      <tr><td>角速度 ω</td><td><b style="color:#f59e0b">${p.omega.toFixed(3)}</b> rad/s</td></tr>
      <tr><td>左履带 实际 vL / 目标</td><td><b>${p.vL.toFixed(3)}</b> / <span style="color:#fbbf24">${s.smoothL.toFixed(3)}</span> m/s</td></tr>
      <tr><td>右履带 实际 vR / 目标</td><td><b>${p.vR.toFixed(3)}</b> / <span style="color:#fbbf24">${s.smoothR.toFixed(3)}</span> m/s</td></tr>
      <tr><td>PWM 输出 左 / 右</td><td><b>${Math.round(s.motorL_out)}</b> / <b>${Math.round(s.motorR_out)}</b>  (±${PID.CFG.PWM_RES})</td></tr>
      <tr><td>转向模式</td><td><b style="color:${modeColor}">${modeLabel}</b> · <code>${s.mode}</code></td></tr>
      <tr><td>目标输入</td><td><code>v=${s.targetV.toFixed(2)}, ω=${s.targetOmega.toFixed(2)}</code></td></tr>
      <tr><td>控制周期</td><td>100 Hz · tick #${s.tickCount}</td></tr>
    `;
  }

  return { init, updateStatus, appendLog };
})();
