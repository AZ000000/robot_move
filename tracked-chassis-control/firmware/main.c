/* =============================================================
 * main.c — STM32 HAL 固件入口
 *
 *   这里是整个控制循环的调度者。具体业务逻辑全部下沉到
 *   core / drivers / proto 三层里，main.c 只做：
 *      1. 初始化顺序
 *      2. 主循环里处理新命令（1ms 级）
 *      3. HAL 回调里挂驱动（中断级）
 *
 *   ┌──────────────────────────────────────────────────────┐
 *   │  main loop (1ms)                                     │
 *   │   └── 检查 cmd_new                                   │
 *   │        ├── kinematics_full → 算 vL vR                │
 *   │        ├── cmd_timeout_check                        │
 *   │        └── (PID 不在主循环，在 TIM2 中断里跑)         │
 *   ├──────────────────────────────────────────────────────┤
 *   │  TIM2 中断 (100Hz / 10ms)                            │
 *   │   ├── encoder_get_speed × 2                          │
 *   │   ├── speed_planner_step                             │
 *   │   ├── pid_calc × 2                                   │
 *   │   └── motor_apply × 2                                │
 *   ├──────────────────────────────────────────────────────┤
 *   │  UART RX 中断 (逐字节)                               │
 *   │   ├── ringbuf_push                                   │
 *   │   └── cmd_parse_ascii / cmd_parse_bin（检测到完整帧） │
 *   └──────────────────────────────────────────────────────┘
 *
 * 作者：嵌入式控制白皮书 v1.0 附带代码
 * 硬件：STM32F103C8T6 + MTB6612 + 带编码器减速电机 × 2
 * ============================================================= */

#include "main.h"

/* ---------- HAL 头（STM32CubeMX 生成）---------- */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_tim.h"

/* ---------- 业务头 ---------- */
#include "chassis_config.h"
#include "chassis_types.h"
#include "kinematics.h"
#include "speed_planner.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "ringbuf.h"
#include "cmd_parser.h"

/* ---------- HAL 句柄（外部定义或 CubeMX 生成）---------- */
TIM_HandleTypeDef htim_pwm_left;
TIM_HandleTypeDef htim_pwm_right;
TIM_HandleTypeDef htim_ctrl;       /* 100Hz 中断定时器 */
UART_HandleTypeDef huart1;

/* ---------- 内部状态 ---------- */
static pid_ctx_t  pid_L;
static pid_ctx_t  pid_R;
static ringbuf_t  rx_rb;
static uint8_t    rx_byte;          /* UART 单字节接收缓冲 */

/* =============================================================
 *  HAL 桩函数：让 drivers/motor.c 里的 weak 覆盖生效
 *  把 PWM 计数值真正写进 TIMx->CCRx
 * ============================================================= */
void motor_set_dir(motor_ch_t ch, uint8_t val)
{
    switch (ch) {
        case MOTOR_LEFT:
            if (val == 1) {   /* 正转 */
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            } else if (val == 0) {   /* 反转 */
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
            } else if (val == 0xFE) {   /* 短路制动 */
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
            } else {   /* 高阻 */
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            }
            break;
        case MOTOR_RIGHT:
            /* PB3/PB4 对称，这里用 PB5/PB6 也行 */
            break;
        default: break;
    }
}

void motor_apply_raw(motor_ch_t ch, uint16_t pwm)
{
    switch (ch) {
        case MOTOR_LEFT:
            __HAL_TIM_SET_COMPARE(&htim_pwm_left, TIM_CHANNEL_3, pwm);
            break;
        case MOTOR_RIGHT:
            __HAL_TIM_SET_COMPARE(&htim_pwm_right, TIM_CHANNEL_4, pwm);
            break;
        default: break;
    }
}

uint32_t cmd_now_ms(void)      { return HAL_GetTick(); }
void     motor_brake_all(void) { motor_brake(MOTOR_LEFT); motor_brake(MOTOR_RIGHT); }

/* =============================================================
 *  100Hz 控制周期 — TIM2 中断
 *  这里是实时性要求最高的一段，不能有阻塞
 * ============================================================= */
static void ctrl_tick_100hz(void)
{
    /* 1. 读编码器速度 */
    float vL_actual = encoder_get_speed(ENC_LEFT);
    float vR_actual = encoder_get_speed(ENC_RIGHT);

    /* 2. 速度平滑（防止猛冲）*/
    speed_planner_step(&g_chassis.smooth_vL, g_chassis.target_vL,
                       &g_chassis.smooth_vR, g_chassis.target_vR);

    /* 3. PID */
    float pwm_L = pid_calc(&pid_L, g_chassis.smooth_vL, vL_actual, CONTROL_DT);
    float pwm_R = pid_calc(&pid_R, g_chassis.smooth_vR, vR_actual, CONTROL_DT);

    /* 4. PWM 输出（带变化率限制）*/
    motor_stop_mode_t stop = (g_chassis.state == STATE_RUN) ? STOP_COAST : STOP_BRAKE;
    motor_apply_slew((int32_t)pwm_L, MOTOR_LEFT,  stop);
    motor_apply_slew((int32_t)pwm_R, MOTOR_RIGHT, stop);
}

/* =============================================================
 *  HAL 回调：TIM2 中断（每 10ms）
 * ============================================================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {     /* 100Hz 控制定时器 */
        ctrl_tick_100hz();
    }
}

/* =============================================================
 *  HAL 回调：UART RX 中断
 *  每收到 1 字节触发一次，丢进环形缓冲区
 * ============================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ringbuf_push(&rx_rb, rx_byte);

        /* 尝试 ASCII 帧 */
        int flen = ringbuf_has_ascii_frame(&rx_rb);
        if (flen > 0) {
            uint8_t frame[64];
            ringbuf_flush_to(&rx_rb, frame, flen);
            frame[flen] = 0;
            cmd_parse_ascii((const char *)frame, flen);
        }
        /* 尝试二进制帧 */
        flen = ringbuf_has_bin_frame(&rx_rb);
        if (flen > 0) {
            uint8_t frame[16];
            ringbuf_flush_to(&rx_rb, frame, flen);
            cmd_parse_bin(frame, flen);
        }

        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

/* =============================================================
 *  主函数
 * ============================================================= */
int main(void)
{
    /* ---- CubeMX 生成 ---- */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();            /* 100Hz 控制定时器 */
    MX_TIM3_Init();            /* PWM 输出 */
    MX_TIM4_Init();            /* 编码器正交解码 */

    /* ---- 业务初始化 ---- */
    encoder_init();
    motor_init();
    ringbuf_init(&rx_rb);

    pid_init(&pid_L, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
    pid_init(&pid_R, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);

    HAL_TIM_PWM_Start(&htim_pwm_left,  TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim_pwm_right, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim_ctrl);                 /* 开 100Hz 中断 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);         /* 开 UART 中断 */

    motor_set_enabled(1);
    g_chassis.state = STATE_IDLE;
    g_chassis.cmd_last_ts_ms = HAL_GetTick();

    /* ---- 主循环（1ms 节拍足够）---- */
    while (1) {

        /* 有新命令 → 做一次运动学 + 转向模式选择 */
        if (g_chassis.cmd_new) {
            g_chassis.steer_mode = kinematics_full(
                g_chassis.target_v,
                g_chassis.target_omega,
                &g_chassis.target_vL,
                &g_chassis.target_vR
            );
            g_chassis.cmd_new = 0;
        }

        /* 命令超时检测（防上位机挂掉后疯跑）*/
        cmd_timeout_check(HAL_GetTick());

        HAL_Delay(1);
    }
}

/* =============================================================
 *  CubeMX 占位函数（真正实现在 .ioc 重新生成后会覆盖，
 *  但这里留最小骨架保证能编译过）
 * ============================================================= */
void SystemClock_Config(void) {}
void MX_GPIO_Init(void)      {}
void MX_USART1_UART_Init(void){}
void MX_TIM2_Init(void)      {}
void MX_TIM3_Init(void)      {}
void MX_TIM4_Init(void)      {}
