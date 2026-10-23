/* =============================================================
 * kinematics.h
 * ============================================================= */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "chassis_types.h"

void        kinematics_solve(float v, float omega,
                             float *vL_out, float *vR_out);

steer_mode_t kinematics_full(float v, float omega,
                             float *vL_out, float *vR_out);

float       kinematics_speed_to_radps(float v_mps);

#endif
