#include "chassis.h"

#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_servo.h"

static ChassisState s_state;

static int16_t clamp_permille(int16_t value)
{
    if (value > 1000)
    {
        return 1000;
    }
    if (value < -1000)
    {
        return -1000;
    }
    return value;
}

void Chassis_Init(void)
{
    s_state.speed_permille = 0;
    s_state.steer_permille = 0;
    s_state.left_encoder_delta = 0;
    s_state.right_encoder_delta = 0;
}

void Chassis_SetCommand(int16_t speed_permille, int16_t steer_permille)
{
    s_state.speed_permille = clamp_permille(speed_permille);
    s_state.steer_permille = clamp_permille(steer_permille);
}

void Chassis_Stop(void)
{
    Chassis_SetCommand(0, 0);
    BspMotor_StopAll();
    BspServo_SetSteerPermille(0);
}

void Chassis_Task10ms(void)
{
    BspEncoderSample encoder = BspEncoder_Read();
    s_state.left_encoder_delta = encoder.left_delta;
    s_state.right_encoder_delta = encoder.right_delta;

    BspMotor_SetDuty(BSP_MOTOR_LEFT, s_state.speed_permille);
    BspMotor_SetDuty(BSP_MOTOR_RIGHT, s_state.speed_permille);
    BspServo_SetSteerPermille(s_state.steer_permille);
}

ChassisState Chassis_GetState(void)
{
    return s_state;
}
