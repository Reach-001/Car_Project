#ifndef MOTION_INTERNAL_H
#define MOTION_INTERNAL_H

#include <stdint.h>

void Ackermann_Compute(float speed, float angle, float *left_out, float *right_out);

void SpeedPi_Init(void);
int16_t SpeedPi_Compute(int motor_id, float target, float actual);

#endif /* MOTION_INTERNAL_H */
