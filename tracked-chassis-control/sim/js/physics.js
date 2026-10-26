/* =============================================================
 * physics.js — 被控对象模型
 *
 *   这是仿真器的"底盘"本身。它不知道 PID、不知道运动学，
 *   只知道 vL 和 vR → 世界坐标 (x, y, theta) 怎么变。
 *
 *   运动方程：
 *     v_center = (vL + vR) / 2
 *     omega    = (vR - vL) / WHEELBASE
 *     theta   += omega * dt
 *     x       += v_center * cos(theta) * dt
 *     y       += v_center * sin(theta) * dt
 *
 *   额外引入简化摩擦 + 打滑模型（让仿真更像真机）
 * ============================================================= */

const PHYS = (() => {

  const CFG = {
    WHEELBASE_M: 0.10,       /* 与 firmware/core/chassis_config.h 保持一致 */
    TRACK_LENGTH_M: 0.08,    /* 单条履带长度 8 cm，用户可调 */
    TRACK_WIDTH_M: 0.03,     /* 履带宽 3 cm */
    GROUND_FRICTION: 0.02,     /* 每次 tick 速度衰减 (0~1) */
    MAX_V_MPS: 1.2,
    MAX_OMEGA_RADPS: 4.0
  };

  function create(x = 0, y = 0, theta = 0) {
    return {
      x, y, theta,
      vL: 0,          /* 两条履带的实际线速度 (m/s) */
      vR: 0,
      vCenter: 0,
      omega: 0,
      poseHistory: [] /* 轨迹残影 */
    };
  }

  /*
   * 给定每条履带的"驱动力"（就是电机给多少），
   * 更新实际速度 + 世界坐标位置
   *
   * input: motorL, motorR 取值范围大致 ±PWM_RES，
   *        这里用一个线性映射 motor → 目标轮速
   */
  function step(state, motorL, motorR, dt) {

    // 1. 电机输出 → 轮目标速度
    const motorToSpeed = 1.2;  // PID ±1.0 归一化驱动力 → ±1.2 m/s 满载
    const vL_target = motorL * motorToSpeed;
    const vR_target = motorR * motorToSpeed;

    // 2. 一阶惯性：电机有滞后，不能瞬间达到目标
    const tau = 0.08;  // 80ms 时间常数，模拟电机+减速器的机械滞后
    const alpha = 1 - Math.exp(-dt / tau);
    state.vL += (vL_target - state.vL) * alpha;
    state.vR += (vR_target - state.vR) * alpha;

    // 3. 摩擦衰减（非常小）
    const fricL = state.vL * (1 - CFG.GROUND_FRICTION * dt * 4);
    const fricR = state.vR * (1 - CFG.GROUND_FRICTION * dt * 4);
    state.vL = fricL;
    state.vR = fricR;

    // 4. 运动学 → 世界坐标
    state.vCenter = (state.vL + state.vR) / 2;
    state.omega   = (state.vR - state.vL) / CFG.WHEELBASE_M;

    state.theta += state.omega * dt;
    state.x     += state.vCenter * Math.cos(state.theta) * dt;
    state.y     += state.vCenter * Math.sin(state.theta) * dt;

    // 5. 残影记录
    state.poseHistory.push({ x: state.x, y: state.y });
    if (state.poseHistory.length > 400) state.poseHistory.shift();

    return state;
  }

  function reset(state, x = 0, y = 0, theta = 0) {
    state.x = x; state.y = y; state.theta = theta;
    state.vL = state.vR = 0;
    state.vCenter = state.omega = 0;
    state.poseHistory = [];
    return state;
  }

  return { CFG, create, step, reset };
})();
