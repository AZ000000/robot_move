/* =============================================================
 * simulator.js — 把所有模块串起来的主循环
 *
 *   控制环 (100Hz / 10ms)：
 *     1. 有新命令 → kinematics.full() → 算 vL_target, vR_target
 *     2. speed_planner.step()          → 平滑 vL_smooth, vR_smooth
 *     3. pid.calc() × 2               → 输出 motorL, motorR (PWM 计数值)
 *
 *   物理环 (同频率)：
 *     4. PHYS.step(motorL, motorR)    → 更新 x, y, theta, vL_actual, vR_actual
 *
 *   渲染环 (requestAnimationFrame)：
 *     5. 画世界 (轨迹 / 网格 / 底盘 / 履带)
 * ============================================================= */

(function() {

  const CONTROL_HZ = 100;
  const CONTROL_DT = 1 / CONTROL_HZ;

  /* ---------- 状态 ---------- */
  const state = {
    targetV:     0,
    targetOmega: 0,
    mode:        'DIFF',

    smoothL: 0, smoothR: 0,
    cmdNew:  false,

    pidL: PID.create(),
    pidR: PID.create(),

    phys: PHYS.create(0, 0, -Math.PI/2),  /* canvas y 向下，theta=-π/2 → sin=-1 → y 减小 → 画面"上方" */

    motorL_out: 0,   /* 最近一次 PID 输出 */
    motorR_out: 0,
    tickCount:   0
  };

  /* ---------- 控制环（每 10ms）---------- */
  function controlTick() {
    state.tickCount++;

    if (state.cmdNew) {
      const r = KIN.full(state.targetV, state.targetOmega);
      state.smoothL_target = r.vL;
      state.smoothR_target = r.vR;
      state.mode = r.mode;
      state.cmdNew = false;
      PID.reset(state.pidL);
      PID.reset(state.pidR);
    }

    if (state.smoothL_target === undefined) state.smoothL_target = 0;
    if (state.smoothR_target === undefined) state.smoothR_target = 0;

    // 速度平滑
    const pl = PLANNER.step(
      state.smoothL, state.smoothL_target,
      state.smoothR, state.smoothR_target
    );
    state.smoothL = pl.vL;
    state.smoothR = pl.vR;

    // PID — 注意被控对象的"实际速度"从 PHYS 读
    state.motorL_out = PID.calc(state.pidL, state.smoothL, state.phys.vL, CONTROL_DT);
    state.motorR_out = PID.calc(state.pidR, state.smoothR, state.phys.vR, CONTROL_DT);

    // 物理更新
    PHYS.step(state.phys, state.motorL_out, state.motorR_out, CONTROL_DT);
  }

  /* ---------- 启动控制环 ---------- */
  let controlRunning = false;
  let controlAccum = 0;
  let lastWallTime = 0;

  function startControlLoop() {
    if (controlRunning) return;
    controlRunning = true;
    lastWallTime = performance.now();
    requestAnimationFrame(controlAndRender);
  }

  function stopControlLoop() {
    controlRunning = false;
  }

  function controlAndRender(now) {
    if (!controlRunning) return;

    let delta = (now - lastWallTime) / 1000;
    lastWallTime = now;
    if (delta > 0.1) delta = 0.1;

    // 固定步长控制（最多补 5 次，防卡顿爆栈）
    controlAccum += delta;
    let guard = 5;
    while (controlAccum >= CONTROL_DT && guard-- > 0) {
      controlTick();
      controlAccum -= CONTROL_DT;
    }
    if (guard <= 0) controlAccum = 0;

    Render.draw(state);
    UI.updateStatus(state);

    requestAnimationFrame(controlAndRender);
  }

  /* ---------- 命令输入接口 ---------- */
  function sendCommand(text) {
    const result = PROTO.parse(text);
    if (!result) return { ok: false, msg: '无法解析命令' };

    switch (result.action) {
      case 'MOV':
        state.targetV     = result.v;
        state.targetOmega = result.omega;
        state.cmdNew      = true;
        return { ok: true, msg: `MOV v=${result.v.toFixed(2)}m/s ω=${result.omega.toFixed(2)}rad/s` };

      case 'STOP':
        state.targetV     = 0;
        state.targetOmega = 0;
        state.cmdNew      = true;
        return { ok: true, msg: 'STOP — 软停车' };

      case 'RESET':
        PHYS.reset(state.phys);
        state.targetV = state.targetOmega = 0;
        state.mode = 'DIFF';
        state.smoothL = state.smoothR = 0;
        state.cmdNew = false;
        PID.reset(state.pidL); PID.reset(state.pidR);
        return { ok: true, msg: 'RESET — 位置、速度、轨迹全部清零' };
    }
    return { ok: false, msg: '未知命令' };
  }

  /* ---------- 暴露到 window ---------- */
  window.Sim = {
    start:   startControlLoop,
    stop:    stopControlLoop,
    state:   state,
    send:    sendCommand
  };

})();
