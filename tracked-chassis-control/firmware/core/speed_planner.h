#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <math.h>

void speed_planner_step(float *vL_inout, float vL_target,
                        float *vR_inout, float vR_target);
void speed_planner_halt(float *vL_inout, float *vR_inout);

#endif
