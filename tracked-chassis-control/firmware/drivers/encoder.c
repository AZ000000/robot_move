/* =============================================================
 * encoder.c — 正交解码 + 速度估算
 *
 *  设计假设：
 *   - MCU HAL 层已把 TIM2/TIM3 配成编码器接口模式
 *     (HAL_TIM_Encoder_Start / HAL_TIM_Encoder_Start_IT)
 *   - 每个通道一个 16-bit 计数器
 * ============================================================= */
#include "encoder.h"
#include "chassis_config.h"
#include "chassis_types.h"

/* --------- HAL 句柄（外部定义，在 main.c / stm32f1xx_it.c 里）--------- */
extern void *htim_enc_left;
extern void *htim_enc_right;

/* --------- 累计 tick（复位清零） --------- */
static volatile int32_t  cum_tick[ENC_COUNT] = {0};
static          float    v_actual[ENC_COUNT] = {0.0f};
static volatile uint32_t last_tick_ms[ENC_COUNT] = {0};

/* --------- 初始化 --------- */
void encoder_init(void)
{
    for (int i = 0; i < ENC_COUNT; i++) {
        cum_tick[i]      = 0;
        v_actual[i]      = 0.0f;
        last_tick_ms[i]  = 0;
    }
}

/* --------- 中断回调：编码器溢出时调用 ---------
 * （在 HAL_TIM_PeriodElapsedCallback 或 EXTI 回调里触发）
 */
void encoder_on_overflow(enc_ch_t ch, int32_t delta)
{
    cum_tick[ch] += delta;
}

/* --------- 每 10ms 读一次计数器，算增量转速度 ---------
 *  由 PID 控制周期调用
 */
float encoder_get_speed(enc_ch_t ch)
{
    /* HAL 层把计数方向 / 增减量算好，这里假设 HAL 已经给我们 delta */
    /* 简化示例：每次从 TIMx->CNT 读 */
    extern uint32_t HAL_GetTick(void);  /* HAL 提供 */
    (void)HAL_GetTick;                  /* 防止 warning */

    /* 实际工程里：
     *   diff = TIM2->CNT - enc_capture_prev[ch];
     *   enc_capture_prev[ch] = TIM2->CNT;
     *   cum_tick[ch] += diff;
     */

    /* 物理公式：
     *   v (m/s) = (diff) * (DRIVE_WHEEL_R_M * 2π) / (PULSE_PER_REV * ENCODER_MULT * GEAR_RATIO) / CONTROL_DT
     *
     * 合并常数：
     *   K = (2π · DRIVE_WHEEL_R_M) / (PULSE_PER_REV · ENCODER_MULT · GEAR_RATIO · CONTROL_DT)
     */
    static const float K = (6.283185f * DRIVE_WHEEL_R_M)
                         / (PULSE_PER_REV * ENCODER_MULT * GEAR_RATIO * CONTROL_DT);

    /* 这里给占位的 tick delta — 实际工程用 HAL 读 */
    extern int32_t enc_delta_this_cycle[ENC_COUNT];
    int32_t diff = enc_delta_this_cycle[ch];

    v_actual[ch] = (float)diff * K;
    if (v_actual[ch] >  V_MAX_MPS) v_actual[ch] =  V_MAX_MPS;
    if (v_actual[ch] < -V_MAX_MPS) v_actual[ch] = -V_MAX_MPS;

    return v_actual[ch];
}

/* --------- 查询累计 tick（给 CMD:QUERY:ENCODER 用）--------- */
int32_t encoder_get_cumulative(enc_ch_t ch)
{
    return cum_tick[ch];
}
