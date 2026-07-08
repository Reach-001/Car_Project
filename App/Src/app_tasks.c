#include "app_tasks.h"

#include "app_mode.h"
#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "iwdg.h"
#include "bt_link.h"
#include "chassis.h"
#include "k230_link.h"
#include "scheduler.h"
#include "tracker.h"
#include "ultrasonic.h"

static void Task_Chassis10ms(void *context)
{
    (void)context;
    Chassis_Task10ms();
}

static void Task_Peripheral10ms(void *context)
{
    (void)context;
    BspKey_Task10ms();
    BspBuzzer_Task10ms();
    Tracker_Task10ms();
    Ultrasonic_Task10ms();
}

static void Task_Input20ms(void *context)
{
    (void)context;

    if (BspKey_TakeClickedEvent(BSP_KEY_1))
    {
        Chassis_Stop();
    }
}

static void Task_Communication10ms(void *context)
{
    BtCommand bt_command;
    K230Result k230_result;
    (void)context;

    K230Link_Task();
    BtLink_Task();

    if (BtLink_TakeCommand(&bt_command))
    {
        AppMode_HandleBtCommand(&bt_command);
    }

    if (K230Link_GetLatestResult(&k230_result))
    {
        AppMode_HandleK230Result(&k230_result);
    }
}

static void Task_Mode20ms(void *context)
{
    (void)context;
    AppMode_Task20ms();
}

static void Task_Heartbeat500ms(void *context)
{
    static bool led_on;
    (void)context;

    led_on = !led_on;
    BspLed_SetStateLed(led_on);
    HAL_IWDG_Refresh(&hiwdg);
}

void AppTasks_Register(void)
{
    (void)Scheduler_AddTask("peripheral", Task_Peripheral10ms, 0, 10U, 0U);
    (void)Scheduler_AddTask("chassis", Task_Chassis10ms, 0, 10U, 0U);
    (void)Scheduler_AddTask("comm", Task_Communication10ms, 0, 10U, 1U);
    (void)Scheduler_AddTask("mode", Task_Mode20ms, 0, 20U, 3U);
    (void)Scheduler_AddTask("input", Task_Input20ms, 0, 20U, 2U);
    (void)Scheduler_AddTask("heartbeat", Task_Heartbeat500ms, 0, 500U, 0U);
}
