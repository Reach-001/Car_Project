#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

typedef enum
{
    BSP_MOTOR_LEFT = 0,
    BSP_MOTOR_RIGHT
} BspMotorId;

void BspMotor_Init(void);
void BspMotor_SetDuty(BspMotorId motor, int16_t duty_permille);
void BspMotor_StopAll(void);

#endif
