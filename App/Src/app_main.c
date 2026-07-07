#include "app_main.h"

#include "app_tasks.h"
#include "bsp_encoder.h"
#include "bsp_io.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "chassis.h"
#include "scheduler.h"

#include "stm32g4xx_hal.h"

void App_Init(void)
{
    BspIo_Init();
    BspMotor_Init();
    BspEncoder_Init();
    (void)BspServo_Init();
    Chassis_Init();

    Scheduler_Init();
    AppTasks_Register();
}

void App_Run(void)
{
    Scheduler_Run(HAL_GetTick());
}
