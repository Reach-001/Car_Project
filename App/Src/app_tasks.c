#include "app_tasks.h"

#include "bsp_io.h"
#include "chassis.h"
#include "scheduler.h"

static void Task_Chassis10ms(void *context)
{
    (void)context;
    Chassis_Task10ms();
}

static void Task_Input20ms(void *context)
{
    (void)context;
    BspKeyState keys = BspIo_ReadKeys();

    if (keys.key1)
    {
        Chassis_Stop();
    }
}

static void Task_Heartbeat500ms(void *context)
{
    static bool led_on;
    (void)context;

    led_on = !led_on;
    BspIo_SetStateLed(led_on);
}

void AppTasks_Register(void)
{
    (void)Scheduler_AddTask("chassis", Task_Chassis10ms, 0, 10U, 0U);
    (void)Scheduler_AddTask("input", Task_Input20ms, 0, 20U, 2U);
    (void)Scheduler_AddTask("heartbeat", Task_Heartbeat500ms, 0, 500U, 0U);
}
