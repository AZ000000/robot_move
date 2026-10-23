/* =============================================================
 * cmd_parser.js — ASCII 协议解析（JS 版）
 *  直接对应 firmware/proto/cmd_parser.c
 * ============================================================= */

const PROTO = (() => {

  const CFG = {
    V_MAX_MPS:       0.8,
    OMEGA_MAX_RADPS: 2.0
  };

  function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }

  /* 输入一行字符串，返回 { v, omega, action } 或 null */
  function parse(line) {
    line = line.trim();
    if (!line) return null;

    if (line.startsWith('CMD:MOV,')) {
      const body = line.substring(8);   /* 跳过 "CMD:MOV," */
      const match = body.match(/v=([-+]?\d*\.?\d+),\s*ω=([-+]?\d*\.?\d+)/);
      if (!match) return null;
      let v     = parseFloat(match[1]);
      let omega = parseFloat(match[2]);
      v     = clamp(v,     -CFG.V_MAX_MPS,       CFG.V_MAX_MPS);
      omega = clamp(omega, -CFG.OMEGA_MAX_RADPS, CFG.OMEGA_MAX_RADPS);
      return { action: 'MOV', v, omega };
    }

    if (line === 'CMD:STOP') {
      return { action: 'STOP' };
    }

    if (line === 'CMD:RESET') {
      return { action: 'RESET' };
    }

    return null;
  }

  return { CFG, parse };
})();
