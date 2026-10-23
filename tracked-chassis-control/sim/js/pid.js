/* =============================================================
 * pid.js — 增量式 PID
 *  由 firmware/core/pid.c 翻译
 * ============================================================= */

const PID = (() => {

  const CFG = {
    PID_KP: 0.8,
    PID_KI: 0.02,
    PID_KD: 0.15,
    PWM_RES: 65535,
    I_MAX:   5000
  };

  function create(kp, ki, kd) {
    return {
      kp: kp ?? CFG.PID_KP,
      ki: ki ?? CFG.PID_KI,
      kd: kd ?? CFG.PID_KD,
      integral: 0,
      prevErr: 0
    };
  }

  function calc(p, target, actual, dt) {
    const err = target - actual;

    const outP = p.kp * err;

    let nextI = p.integral + p.ki * err * dt;
    nextI = Math.max(-CFG.I_MAX, Math.min(CFG.I_MAX, nextI));
    p.integral = nextI;
    const outI = p.integral;

    const deriv = (err - p.prevErr) / dt;
    p.prevErr = err;
    const outD = p.kd * deriv;

    let out = outP + outI + outD;
    out = Math.max(-CFG.PWM_RES, Math.min(CFG.PWM_RES, out));
    return out;
  }

  function reset(p) { p.integral = 0; p.prevErr = 0; }

  return { CFG, create, calc, reset };
})();
