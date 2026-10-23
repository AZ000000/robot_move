/* =============================================================
 * speed_planner.c — 加减速平滑器（S 曲线简化版）
 *
 *  不让目标速度瞬间跳变，而是每个控制周期最多改 Δv
 *  Δv = MAX_ACCEL_MPS2 × CONTROL_DT
 *
 *  放在 PID 之前，把 "理想速度" 变成 "平滑可跟踪速度"
 * ============================================================= */
#include "speed_planner.h"
#include "chassis_config.h"

static float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static float plan_axis(float current, float target)
{
    float max_step;
    if (fabsf(target) < fabsf(current)) {
        max_step = MAX_DECEL_MPS2 * CONTROL_DT;    /* 减速快一点 */
    } else {
        max_step = MAX_ACCEL_MPS2 * CONTROL_DT;
    }

    float delta = target - current;
    if (delta >  max_step) delta =  max_step;
    if (delta < -max_step) delta = -max_step;

    return clampf(current + delta, -V_MAX_MPS, V_MAX_MPS);
}

/* --------- 对两侧同时做速度平滑 --------- */
void speed_planner_step(float *vL_inout, float vL_target,
                        float *vR_inout, float vR_target)
{
    *vL_inout = plan_axis(*vL_inout, vL_target);
    *vR_inout = plan_axis(*vR_inout, vR_target);
}

/* --------- 完全刹停（急停调用，不做平滑）--------- */
void speed_planner_halt(float *vL_inout, float *vR_inout)
{
    *vL_inout = 0.0f;
    *vR_inout = 0.0f;
}
