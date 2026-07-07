#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdbool.h>
#include <stdint.h>

bool BspServo_Init(void);
bool BspServo_IsAvailable(void);
void BspServo_SetSteerPermille(int16_t steer_permille);

#endif
