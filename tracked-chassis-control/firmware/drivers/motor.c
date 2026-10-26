/* =============================================================
 * motor.c — PWM 输出 + 方向 + 制动
 *
 *  底层用 STM32 HAL_TIM_PWM 接口。
 *  本层只管 "一个电机 = 一个 PWM + 一个 DIR" 的抽象。
 * ============================================================= */
#include "motor.h"
#include "chassis_config.h"
#include "chassis_types.h"

/* --------- HAL 句柄（由 main.c 定义）--------- */
extern void *htim_pwm;              /* 共用 PWM 定时器 */

/* --------- 私有 --------- */
static uint16_t last_pwm[MOTOR_COUNT] = {0};
static uint8_t  motor_enabled         = 1;   /* 总开关 */

/* --------- 初始化 --------- */
void motor_init(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        last_pwm[i] = 0;
    }
    motor_enabled = 1;
}

/* --------- 使能 / 禁用总输出（紧急停时拉低 EN 脚）--------- */
void motor_set_enabled(uint8_t on)
{
    motor_enabled = on;
    if (!on) {
        /* TB6612FNG 的 PWMA/PWMB 清零，或 TB6612FNG 或 L298N 的 ENA/ENB 拉低 */
        for (int i = 0; i < MOTOR_COUNT; i++) motor_apply_raw(i, 0);
    }
}

/* --------- 直接输出（PWM 原始值 0~PWM_RES）---------
 *  符号由上层决定方向：
 *    PWM > 0 → 正转
 *    PWM < 0 → 反转
 *    PWM == 0 → 自由停车或制动（由 brake 参数决定）
 */
void motor_apply(int32_t pwm_raw, motor_ch_t ch, motor_stop_mode_t stop)
{
    if (ch >= MOTOR_COUNT) return;
    if (!motor_enabled) { last_pwm[ch] = 0; return; }

    int32_t abs_pwm = (pwm_raw < 0) ? -pwm_raw : pwm_raw;
    if (abs_pwm > (int32_t)PWM_RES) abs_pwm = (int32_t)PWM_RES;

    /* 方向 */
    if (pwm_raw > 0)      motor_set_dir(ch, 1);
    else if (pwm_raw < 0) motor_set_dir(ch, 0);

    /* 静止时的行为 */
    if (abs_pwm == 0) {
        if (stop == STOP_BRAKE) {
            motor_brake(ch);
            last_pwm[ch] = 0;
            return;
        }
        /* STOP_COAST：DIR 设成无效值让 H 桥高阻 */
        motor_set_dir(ch, 0xFF);
        last_pwm[ch] = 0;
        return;
    }

    motor_apply_raw(ch, (uint16_t)abs_pwm);
    last_pwm[ch] = (uint16_t)abs_pwm;
}

/* --------- PWM 变化率限制（防电流尖峰）--------- */
void motor_apply_slew(int32_t pwm_raw, motor_ch_t ch, motor_stop_mode_t stop)
{
    if (ch >= MOTOR_COUNT) return;

    int32_t abs_pwm = (pwm_raw < 0) ? -pwm_raw : pwm_raw;
    int32_t diff    = abs_pwm - last_pwm[ch];
    if (diff >  PWM_SLEW_RATE) diff =  PWM_SLEW_RATE;
    if (diff < -PWM_SLEW_RATE) diff = -PWM_SLEW_RATE;

    motor_apply((int32_t)(last_pwm[ch] + diff) * (pwm_raw >= 0 ? 1 : -1),
                ch, stop);
}

/* --------- 主动制动：DIR 拉成 HIGH-HIGH（TB6612FNG）或 LOW-LOW --------- */
void motor_brake(motor_ch_t ch)
{
    motor_set_dir(ch, 0xFE);   /* 特殊值，底层 HAL 识别为短路制动 */
    motor_apply_raw(ch, 0);
}

/* --------- HAL 对接桩函数（在 stm32f1xx_hal_conf 或 main.c 里实现）--------- */
__attribute__((weak))
void motor_set_dir(motor_ch_t ch, uint8_t val)   { (void)ch; (void)val; }
__attribute__((weak))
void motor_apply_raw(motor_ch_t ch, uint16_t pwm){ (void)ch; (void)pwm; }
