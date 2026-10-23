#ifndef ENCODER_H
#define ENCODER_H

#include "chassis_types.h"

void  encoder_init(void);
void  encoder_on_overflow(enc_ch_t ch, int32_t delta);
float encoder_get_speed(enc_ch_t ch);
int32_t encoder_get_cumulative(enc_ch_t ch);

#endif
