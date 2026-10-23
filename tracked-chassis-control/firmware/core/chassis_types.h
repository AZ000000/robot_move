/* =============================================================
 * chassis_types.h — 全局类型定义
 * ============================================================= */
#ifndef CHASSIS_TYPES_H
#define CHASSIS_TYPES_H

#include <stdint.h>
#include "chassis_config.h"

/* 电机通道 */
typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2
} motor_ch_t;

/* 编码器通道 */
typedef enum {
    ENC_LEFT  = 0,
    ENC_RIGHT = 1,
    ENC_COUNT = 2
} enc_ch_t;

/* 固件状态机 */
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_RUN,
    STATE_FAULT
} chassis_state_t;

/* 转向模式 */
typedef enum {
    STEER_NONE   = 0,
    STEER_DIFF   = 1,   /* 平滑差速 */
    STEER_BRAKE  = 2,   /* 单边制动 */
    STEER_PIVOT  = 3    /* 原地反转 */
} steer_mode_t;

/* 命令类型 */
typedef enum {
    CMD_MOV   = 0x01,
    CMD_STOP  = 0x02,
    CMD_BRAKE = 0x03,
    CMD_QUERY = 0x04,
    CMD_PID   = 0x05
} cmd_type_t;

/* 查询子类型 */
typedef enum {
    QUERY_ENCODER = 0x01,
    QUERY_STATUS  = 0x02,
    QUERY_VOLTAGE = 0x03
} query_type_t;

/* CRC */
typedef uint16_t crc16_t;

/* 全局运行时状态 */
typedef struct {
    volatile float  target_v;        /* 线速度目标 (m/s) */
    volatile float  target_omega;    /* 角速度目标 (rad/s) */
    volatile float  target_vL;
    volatile float  target_vR;
    volatile float  smooth_vL;       /* 经速度规划后的输入 */
    volatile float  smooth_vR;
    volatile uint8_t cmd_new;
    volatile chassis_state_t state;
    volatile steer_mode_t steer_mode;
    volatile uint32_t cmd_last_ts_ms;  /* 最近一帧命令的 tick，做超时保护 */
} chassis_ctx_t;

extern chassis_ctx_t g_chassis;

#endif /* CHASSIS_TYPES_H */
