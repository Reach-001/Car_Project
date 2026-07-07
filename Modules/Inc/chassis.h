#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

typedef struct
{
    int16_t speed_permille;
    int16_t steer_permille;
    int32_t left_encoder_delta;
    int32_t right_encoder_delta;
} ChassisState;

void Chassis_Init(void);
void Chassis_SetCommand(int16_t speed_permille, int16_t steer_permille);
void Chassis_Stop(void);
void Chassis_Task10ms(void);
ChassisState Chassis_GetState(void);

#endif
