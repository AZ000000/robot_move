#ifndef PID_H
#define PID_H

#include "chassis_config.h"

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_err;
    float out_min;
    float out_max;
} pid_ctx_t;

void  pid_init(pid_ctx_t *p, float kp, float ki, float kd);
void  pid_reset(pid_ctx_t *p);
float pid_calc (pid_ctx_t *p, float target, float actual, float dt_sec);

#endif
