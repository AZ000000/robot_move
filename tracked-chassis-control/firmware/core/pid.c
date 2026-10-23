/* =============================================================
 * pid.c — 通用增量式 PID
 *
 *   output = Kp * e + Ki * ∫e + Kd * de/dt
 *
 *   使用方法：每个电机通道一个 pid_ctx_t 实例，每 10ms 调一次
 * ============================================================= */
#include "pid.h"
#include "chassis_config.h"

void pid_init(pid_ctx_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integral = 0.0f;
    p->prev_err = 0.0f;
    p->out_min  = -PWM_RES;
    p->out_max  =  PWM_RES;
}

void pid_reset(pid_ctx_t *p)
{
    p->integral = 0.0f;
    p->prev_err = 0.0f;
}

/* --------- 计算一次 PID ---------
 *  target  : 目标速度 (m/s)
 *  actual  : 实际速度 (m/s)
 *  dt_sec  : 控制周期 (秒)，一般传 CONTROL_DT
 *  返回值  : PWM 输出 (计数值，±PWM_RES)
 */
float pid_calc(pid_ctx_t *p, float target, float actual, float dt_sec)
{
    float err = target - actual;

    /* P */
    float out_p = p->kp * err;

    /* I — 带抗饱和：只有输出没超限时才累加 */
    float next_i = p->integral + p->ki * err * dt_sec;
    float preview = out_p + next_i * 0.0f /* 占位，不完整算 */ + 0.0f;
    (void)preview;
    if (next_i >  PID_I_MAX) next_i =  PID_I_MAX;
    if (next_i < -PID_I_MAX) next_i = -PID_I_MAX;
    p->integral = next_i;
    float out_i = p->integral;

    /* D */
    float deriv = (err - p->prev_err) / dt_sec;
    p->prev_err = err;
    float out_d = p->kd * deriv;

    float out = out_p + out_i + out_d;

    /* 输出限幅 */
    if (out >  p->out_max) out =  p->out_max;
    if (out <  p->out_min) out =  p->out_min;

    return out;
}
