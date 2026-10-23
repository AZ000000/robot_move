#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>
#include "chassis_types.h"

extern chassis_ctx_t g_chassis;

/* 在 stm32f1xx_it.c 或 main.c 里实现，返回当前 tick ms */
uint32_t cmd_now_ms(void);

/* 由 HAL 层实现，让两路电机都短接制动 */
void motor_brake_all(void);

int  cmd_parse_ascii(const char *frame, int len);
int  cmd_parse_bin  (const uint8_t *frame, int len);
void cmd_timeout_check(uint32_t now_ms);

#endif
