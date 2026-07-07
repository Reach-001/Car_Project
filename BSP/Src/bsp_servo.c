#include "bsp_servo.h"

static int16_t s_steer_permille;

bool BspServo_Init(void)
{
    s_steer_permille = 0;
    return BspServo_IsAvailable();
}

bool BspServo_IsAvailable(void)
{
    return false;
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
    (void)s_steer_permille;
}
