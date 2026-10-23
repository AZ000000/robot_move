/* =============================================================
 * chassis_config.h — 所有可调硬件参数集中在这里
 * 改板子、换电机、换编码器都只改这一个文件
 * ============================================================= */
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

/* --------- 几何参数 --------- */
#define WHEELBASE_M       0.32f     /* 两侧履带中心距 L (m) */
#define DRIVE_WHEEL_R_M   0.05f     /* 驱动轮 / 驱动链轮半径 (m) */

/* --------- 速度 / 负载 --------- */
#define V_MAX_MPS         0.8f      /* 满载实测最大直行速度 (m/s) */
#define OMEGA_MAX_RADPS   2.0f      /* 允许最大角速度 (rad/s) */

/* --------- 电机 + 编码器 --------- */
#define PULSE_PER_REV     390       /* 电机每转编码器 tick 数（单相） */
#define GEAR_RATIO        98        /* 电机 → 驱动轮总减速比 */
#define ENCODER_MULT      4         /* 正交解码后倍率（4x PPR） */

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

#endif /* CHASSIS_CONFIG_H */
