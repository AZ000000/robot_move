/* =============================================================
 * kinematics.js — 差速底盘逆运动学 + 转向模式选择
 *  直接由 firmware/core/kinematics.c 翻译而来，逻辑完全一致
 * ============================================================= */

const KIN = (() => {

  const CFG = {
    WHEELBASE_M:         0.32,
    V_MAX_MPS:           0.8,
    OMEGA_MAX_RADPS:     2.0,
    STEER_DIFF_MAX_OMEGA: 0.5,
    STEER_BRAKE_MIN_OMEGA:0.5,
    STEER_BRAKE_MAX_OMEGA:1.0,
    PIVOT_VELOCITY_THRESH:0.05,
  };

  // 逆运动学
  function solve(v, omega) {
    return {
      vL: v - omega * CFG.WHEELBASE_M / 2,
      vR: v + omega * CFG.WHEELBASE_M / 2
    };
  }

  function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }

  // 完整运动学 + 转向模式选择
  // 三种模式：DIFF / BRAKE / PIVOT
  function full(v, omega) {
    let { vL, vR } = solve(v, omega);
    let mode = 'DIFF';

    const aO = Math.abs(omega);
    const aV = Math.abs(v);

    if (aV < CFG.PIVOT_VELOCITY_THRESH && aO > CFG.STEER_BRAKE_MIN_OMEGA) {
      // 原地反转
      vL = -omega * CFG.WHEELBASE_M / 2;
      vR =  omega * CFG.WHEELBASE_M / 2;
      mode = 'PIVOT';
    } else if (aO > CFG.STEER_BRAKE_MIN_OMEGA && aV > 0.05) {
      // 单边制动
      if (omega > 0) { vL = 0; vR = 2 * v; }
      else           { vL = 2 * v; vR = 0; }
      mode = 'BRAKE';
    }
    // 否则 DIFF

    vL = clamp(vL, -CFG.V_MAX_MPS, CFG.V_MAX_MPS);
    vR = clamp(vR, -CFG.V_MAX_MPS, CFG.V_MAX_MPS);

    return { vL, vR, mode };
  }

  return { CFG, solve, full };
})();
