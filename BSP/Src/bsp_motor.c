#include "bsp_motor.h"

#include "tim.h"

#define MOTOR_DUTY_MAX 1000
#define MOTOR_TIM_PERIOD 499U

static uint32_t duty_to_compare(int16_t duty_permille)
{
    int32_t duty = duty_permille;
    if (duty < 0)
    {
        duty = -duty;
    }
    if (duty > MOTOR_DUTY_MAX)
    {
        duty = MOTOR_DUTY_MAX;
    }

    return (uint32_t)((duty * (int32_t)MOTOR_TIM_PERIOD) / MOTOR_DUTY_MAX);
}

void BspMotor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    BspMotor_StopAll();
}

void BspMotor_SetDuty(BspMotorId motor, int16_t duty_permille)
{
    uint32_t forward = 0U;
    uint32_t reverse = 0U;
    uint32_t compare = duty_to_compare(duty_permille);

    if (duty_permille >= 0)
    {
        forward = compare;
    }
    else
    {
        reverse = compare;
    }

    if (motor == BSP_MOTOR_LEFT)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, forward);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, reverse);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, forward);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, reverse);
    }
}

void BspMotor_StopAll(void)
{
    BspMotor_SetDuty(BSP_MOTOR_LEFT, 0);
    BspMotor_SetDuty(BSP_MOTOR_RIGHT, 0);
}
