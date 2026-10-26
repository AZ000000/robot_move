/* =============================================================
 * chassis_config.h — 所有可调硬件参数集中在这里
 * 改板子、换电机、换编码器都只改这一个文件
 * ============================================================= */
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

/* --------- 几何参数（★ 改板子先改这组）--------- */
#define WHEELBASE_M       0.10f     /* 两侧履带中心距 L (m)：10 cm，决定转弯半径 */
#define DRIVE_WHEEL_R_M   0.015f    /* 驱动链轮节圆半径 (m)：量法 = 齿根圆半径 + (齿顶-齿根)/2，或 齿数×节距/2π */
#define TRACK_LENGTH_M    0.08f     /* 单条履带长度 (m)：8 cm，仿真器渲染用 */
#define TRACK_WIDTH_M     0.03f     /* 单条履带宽度 (m)：默认 3 cm，渲染用 */

/* --------- 速度 / 负载 --------- */
#define V_MAX_MPS         0.8f      /* 满载实测最大直行速度 (m/s) */
#define OMEGA_MAX_RADPS   2.0f      /* 允许最大角速度 (rad/s) */

/* --------- 电机 + 编码器（★ 按电机规格书填）--------- */
#define PULSE_PER_REV     390       /* 电机每转编码器 tick 数（单相，= 霍尔 PPR） */
#define GEAR_RATIO        45        /* 电机 → 驱动轮总减速比（减速箱 i） */
#define ENCODER_MULT      4         /* 正交解码后倍率（4× PPR，STM32 默认） */

/* --------- 定时器 --------- */
#define PWM_RES           65535     /* PWM 计数值上限（16-bit 定时器） */
#define CONTROL_HZ        100       /* PID 控制周期 (Hz) */
#define CONTROL_DT        0.01f     /* = 1 / CONTROL_HZ */

/* --------- PWM 输出范围 --------- */
#define PWM_MIN_STATIONARY  800     /* 空载让电机开始转的最小 PWM（标定值） */
#define PWM_SLEW_RATE       200     /* 每周期 PWM 变化上限，防突变 */

/* --------- 速度平滑 --------- */
#define MAX_ACCEL_MPS2    0.5f      /* 最大加速度 m/s² */
#define MAX_DECEL_MPS2    1.0f      /* 最大减速度（刹车可以更猛一点） */

/* --------- 转向模式阈值 --------- */
#define STEER_DIFF_MAX_OMEGA   0.5f   /* |ω| ≤ 此值 → 平滑差速 */
#define STEER_BRAKE_MIN_OMEGA  0.5f   /* 单边制动下限 */
#define STEER_BRAKE_MAX_OMEGA  1.0f   /* 单边制动上限（超过 → 原地旋转） */
#define PIVOT_VELOCITY_THRESH  0.05f  /* |v| < 此值 → 允许原地反转 */

/* --------- PID 默认参数 --------- */
#define PID_KP_DEFAULT   0.8f
#define PID_KI_DEFAULT   0.02f
#define PID_KD_DEFAULT   0.15f
#define PID_I_MAX        5000.0f      /* 积分项限幅，防饱和 */

/* --------- 保护阈值 --------- */
#define MOTOR_RATED_CURR_A   5.0f     /* 电机额定电流 A */
#define OVERCURR_L1_RATIO    1.5f     /* 1.5× 额定 → PWM 降 70% */
#define OVERCURR_L2_RATIO    2.0f     /* 2.0× 额定 → 保护性停车 */
#define TENSION_FAULT_N      500.0f   /* 张力传感器低于此值 → 脱轨风险 (N) */
#define ENCODER_TIMEOUT_MS   50       /* 50ms 无 tick → 堵转 */
#define CMD_TIMEOUT_MS       500      /* 上位机 500ms 无命令 → 软停车 */

/* --------- 串口 --------- */
#define UART_BAUD           115200
#define RINGBUF_SIZE        128      /* 命令环形缓冲区大小 */

/* =============================================================
 *  TB6612FNG 管脚分配
 *
 *    TB6612FNG 真值表（每个独立通道，PWMA/AIN1/AIN2 或 PWMB/BIN1/BIN2）：
 *
 *    PWMA(A/B)  |  AIN1  |  AIN2  |  OUT1/OUT2
 *    -----------|--------|--------|------------
 *      HIGH     |  HIGH  |  LOW   |  正转（CCW）
 *      HIGH     |  LOW   |  HIGH  |  反转（CW）
 *      LOW      |   ×    |   ×    |  高阻（自由停车）
 *      HIGH     |  HIGH  |  HIGH  |  短路制动
 *      HIGH     |  LOW   |  LOW   |  短路制动
 *
 *    STBY = PB5  →  高 = 正常工作，低 = 所有输出高阻（紧急停）
 *    PWMA / PWMB 由 PWM 定时器（PB0 / PB1）驱动，频率建议 20~50 kHz
 *    电机方向由 IN1/IN2 组合决定，不是"单 DIR 脚"
 *
 *    如果接的是 L298N（单 ENA + IN1 + IN2），TB6612 宏同样可用，
 *    只是 EN = ENA，IN1/IN2 直接对应即可。
 * ============================================================= */
#define TB6612_STBY_PORT   GPIOB
#define TB6612_STBY_PIN    GPIO_PIN_5   /* 总使能，接 STBY */

/* 左电机通道 A */
#define TB6612_PWMA_PORT   GPIOB
#define TB6612_PWMA_PIN    GPIO_PIN_0   /* PWM（TIM3 CH3）*/
#define TB6612_AIN1_PORT   GPIOB
#define TB6612_AIN1_PIN    GPIO_PIN_3
#define TB6612_AIN2_PORT   GPIOB
#define TB6612_AIN2_PIN    GPIO_PIN_4

/* 右电机通道 B */
#define TB6612_PWMB_PORT   GPIOB
#define TB6612_PWMB_PIN    GPIO_PIN_1   /* PWM（TIM3 CH4）*/
#define TB6612_BIN1_PORT   GPIOB
#define TB6612_BIN1_PIN    GPIO_PIN_6
#define TB6612_BIN2_PORT   GPIOB
#define TB6612_BIN2_PIN    GPIO_PIN_7

#endif /* CHASSIS_CONFIG_H */
