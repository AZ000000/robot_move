/* =============================================================
 * cmd_parser.c — ASCII + 二进制命令解析
 *
 *   生产者: ringbuf 检测到一帧 → cmd_parse_one()
 *   消费者: 往 g_chassis 写入目标值，置 cmd_new 标志
 *
 *   两种协议:
 *     ASCII:  "CMD:MOV,v=0.30,ω=0.50\r\n"
 *     BIN:    13 字节 SOF+CMD+v+w+CRC+EOF
 * ============================================================= */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "cmd_parser.h"
#include "chassis_config.h"
#include "chassis_types.h"
#include "crc16.h"

/* --------- 全局上下文 --------- */
chassis_ctx_t g_chassis = {
    .target_v      = 0.0f,
    .target_omega  = 0.0f,
    .target_vL     = 0.0f,
    .target_vR     = 0.0f,
    .smooth_vL     = 0.0f,
    .smooth_vR     = 0.0f,
    .cmd_new       = 0,
    .state         = STATE_IDLE,
    .steer_mode    = STEER_NONE,
    .cmd_last_ts_ms= 0
};

/* --------- 私有辅助 --------- */
static void clamp_targets(void)
{
    if (g_chassis.target_v     >  V_MAX_MPS)       g_chassis.target_v     =  V_MAX_MPS;
    if (g_chassis.target_v     < -V_MAX_MPS)       g_chassis.target_v     = -V_MAX_MPS;
    if (g_chassis.target_omega >  OMEGA_MAX_RADPS) g_chassis.target_omega =  OMEGA_MAX_RADPS;
    if (g_chassis.target_omega < -OMEGA_MAX_RADPS) g_chassis.target_omega = -OMEGA_MAX_RADPS;
}

/* --------- ASCII 解析 --------- */
int cmd_parse_ascii(const char *frame, int len)
{
    if (len <= 0) return -1;

    /* CMD:MOV,v=%f,ω=%f */
    if (strncmp(frame, "CMD:MOV,", 8) == 0) {
        float v = 0.0f, omega = 0.0f;
        if (sscanf(frame + 8, "v=%f,ω=%f", &v, &omega) >= 1) {
            g_chassis.target_v     = v;
            g_chassis.target_omega = omega;
            clamp_targets();
            g_chassis.cmd_new = 1;
            g_chassis.state   = STATE_RUN;
            g_chassis.cmd_last_ts_ms = cmd_now_ms();
            return 0;
        }
    }
    else if (strncmp(frame, "CMD:STOP", 8) == 0) {
        g_chassis.target_v     = 0.0f;
        g_chassis.target_omega = 0.0f;
        g_chassis.cmd_new      = 1;
        g_chassis.state        = STATE_IDLE;
        return 0;
    }
    else if (strncmp(frame, "CMD:BRAKE", 9) == 0) {
        /* 单纯标记给 HAL 层去做物理制动 */
        g_chassis.state = STATE_IDLE;
        motor_brake_all();
        return 0;
    }
    else if (strncmp(frame, "CMD:QUERY:ENCODER", 17) == 0) {
        return 1;   /* 让上层处理应答帧 */
    }
    else if (strncmp(frame, "CMD:QUERY:STATUS", 16) == 0) {
        return 2;
    }

    return -1;
}

/* --------- 二进制帧解析 --------- */
int cmd_parse_bin(const uint8_t *frame, int len)
{
    if (len < 13) return -1;
    if (frame[0]  != 0xA5) return -1;
    if (frame[12] != 0x5A) return -1;

    /* CRC 校验：SOF + CMD + v + w = 10 字节 */
    uint16_t crc_calc = crc16_modbus(frame, 10);
    uint16_t crc_recv = (uint16_t)frame[10] | ((uint16_t)frame[11] << 8);
    if (crc_calc != crc_recv) return -2;   /* CRC 错误，静默丢弃 */

    uint8_t cmd = frame[1];
    float v, omega;
    memcpy(&v,     &frame[2], 4);
    memcpy(&omega, &frame[6], 4);

    switch (cmd) {
        case 0x01:   /* MOV */
            g_chassis.target_v     = v;
            g_chassis.target_omega = omega;
            clamp_targets();
            g_chassis.cmd_new = 1;
            g_chassis.state   = STATE_RUN;
            g_chassis.cmd_last_ts_ms = cmd_now_ms();
            return 0;

        case 0x02:   /* STOP */
            g_chassis.target_v = g_chassis.target_omega = 0.0f;
            g_chassis.cmd_new = 1;
            g_chassis.state   = STATE_IDLE;
            return 0;

        default:
            return -3;
    }
}

/* --------- 命令超时检测 ---------
 *  在主循环里调用，超过 CMD_TIMEOUT_MS 就把状态切回 IDLE
 */
void cmd_timeout_check(uint32_t now_ms)
{
    if (g_chassis.state == STATE_RUN) {
        if ((now_ms - g_chassis.cmd_last_ts_ms) > CMD_TIMEOUT_MS) {
            g_chassis.target_v = g_chassis.target_omega = 0.0f;
            g_chassis.cmd_new = 1;   /* 让运动学重新跑一次 → 输出 0 */
            g_chassis.state   = STATE_IDLE;
        }
    }
}
