#include "bsp_servo.h"

#include "tim.h"

#define SERVO_CENTER_US 1500
#define SERVO_RANGE_US 500
#define SERVO_MIN_US (SERVO_CENTER_US - SERVO_RANGE_US)
#define SERVO_MAX_US (SERVO_CENTER_US + SERVO_RANGE_US)

static int16_t s_steer_permille;
static bool s_available;

static uint32_t steer_to_compare(int16_t steer_permille)
{
    int32_t pulse_us;

    if (steer_permille > 1000)
    {
        steer_permille = 1000;
    }
    else if (steer_permille < -1000)
    {
        steer_permille = -1000;
    }

    pulse_us = SERVO_CENTER_US + ((int32_t)steer_permille * SERVO_RANGE_US) / 1000;
    if (pulse_us < SERVO_MIN_US)
    {
        pulse_us = SERVO_MIN_US;
    }
    else if (pulse_us > SERVO_MAX_US)
    {
        pulse_us = SERVO_MAX_US;
    }

    return (uint32_t)pulse_us;
}

bool BspServo_Init(void)
{
    s_steer_permille = 0;
    s_available = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) == HAL_OK;
    BspServo_SetSteerPermille(0);
    return s_available;
}

bool BspServo_IsAvailable(void)
{
    return s_available;
}

void BspServo_SetSteerPermille(int16_t steer_permille)
{
    if (steer_permille > 1000)
    {
        steer_permille = 1000;
    }
    else if (steer_permille < -1000)
    {
        steer_permille = -1000;
    }

    s_steer_permille = steer_permille;

    if (s_available)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, steer_to_compare(s_steer_permille));
    }
}
