/* =============================================================
 * kinematics.c — 差速底盘逆运动学 + 转向模式选择
 * 只管纯数学，不知道 PWM、不知道电机、不知道编码器
 * ============================================================= */
#include <math.h>
#include "kinematics.h"
#include "chassis_config.h"
#include "chassis_types.h"

/* --------- 逆运动学：v,ω → vL, vR --------- */
void kinematics_solve(float v, float omega,
                      float *vL_out, float *vR_out)
{
    *vL_out = v - omega * WHEELBASE_M * 0.5f;
    *vR_out = v + omega * WHEELBASE_M * 0.5f;
}

/* --------- 简单限幅 --------- */
static float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

/* --------- 完整运动学：含限幅 + 转向模式选择 ---------
 *  三种模式：
 *   - STEER_DIFF  : |ω| ≤ 0.5 rad/s  → 两侧正转但不同速
 *   - STEER_BRAKE : 0.5 < |ω| ≤ 1.0  → 内侧停，外侧 2×v
 *   - STEER_PIVOT : |v| < 0.05 且 |ω| > 1.0 → 原地反转
 */
steer_mode_t kinematics_full(float v, float omega,
                             float *vL_out, float *vR_out)
{
    steer_mode_t mode;

    /* 先按默认差速算 */
    kinematics_solve(v, omega, vL_out, vR_out);

    float abs_omega = fabsf(omega);
    float abs_v     = fabsf(v);

    /* -------- 条件 1：原地反转 -------- */
    if (abs_v < PIVOT_VELOCITY_THRESH && abs_omega > STEER_BRAKE_MIN_OMEGA) {
        *vL_out = -omega * WHEELBASE_M * 0.5f;
        *vR_out =  omega * WHEELBASE_M * 0.5f;
        mode = STEER_PIVOT;
    }
    /* -------- 条件 2：单边制动 -------- */
    else if (abs_omega > STEER_BRAKE_MIN_OMEGA && abs_v > 0.05f) {
        if (omega > 0) {     /* 向左转 → 左履带停 */
            *vL_out = 0.0f;
            *vR_out = 2.0f * v;
        } else {             /* 向右转 */
            *vL_out = 2.0f * v;
            *vR_out = 0.0f;
        }
        mode = STEER_BRAKE;
    }
    /* -------- 条件 3：平滑差速 -------- */
    else {
        mode = STEER_DIFF;
    }

    /* 最终限幅到 ±V_MAX_MPS */
    *vL_out = clampf(*vL_out, -V_MAX_MPS, V_MAX_MPS);
    *vR_out = clampf(*vR_out, -V_MAX_MPS, V_MAX_MPS);

    return mode;
}

/* --------- 从期望 v 反推两侧目标角速度 (rad/s)，供编码器 --------- */
float kinematics_speed_to_radps(float v_mps)
{
    /* ω = v / r */
    return v_mps / DRIVE_WHEEL_R_M;
}
