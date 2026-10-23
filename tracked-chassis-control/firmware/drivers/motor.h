#ifndef MOTOR_H
#define MOTOR_H

#include "chassis_types.h"

typedef enum {
    STOP_COAST = 0,   /* 自由停车，电机悬空 */
    STOP_BRAKE = 1    /* 短路制动 */
} motor_stop_mode_t;

void motor_init(void);
void motor_set_enabled(uint8_t on);
void motor_apply      (int32_t pwm_raw, motor_ch_t ch, motor_stop_mode_t stop);
void motor_apply_slew (int32_t pwm_raw, motor_ch_t ch, motor_stop_mode_t stop);
void motor_brake      (motor_ch_t ch);

/* 对接 HAL 的桩函数，main.c 里 override */
void motor_set_dir   (motor_ch_t ch, uint8_t val);
void motor_apply_raw (motor_ch_t ch, uint16_t pwm);

#endif
