/* =============================================================
 * speed_planner.js — 加减速平滑器
 *  由 firmware/core/speed_planner.c 翻译
 * ============================================================= */

const PLANNER = (() => {

  const CFG = {
    MAX_ACCEL_MPS2: 0.5,
    MAX_DECEL_MPS2: 1.0,
    CONTROL_DT:      0.01,
    V_MAX_MPS:       0.8
  };

  function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }

  function planAxis(current, target) {
    const max_step = Math.abs(target) < Math.abs(current)
      ? CFG.MAX_DECEL_MPS2 * CFG.CONTROL_DT
      : CFG.MAX_ACCEL_MPS2 * CFG.CONTROL_DT;

    const delta = target - current;
    const step = clamp(delta, -max_step, max_step);
    return clamp(current + step, -CFG.V_MAX_MPS, CFG.V_MAX_MPS);
  }

  function step(vL_cur, vL_tgt, vR_cur, vR_tgt) {
    return {
      vL: planAxis(vL_cur, vL_tgt),
      vR: planAxis(vR_cur, vR_tgt)
    };
  }

  return { CFG, step };
})();
