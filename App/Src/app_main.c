#include "app_main.h"

#include "app_tasks.h"
#include "app_mode.h"
#include "bsp_buzzer.h"
#include "bsp_encoder.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"
#include "bt_link.h"
#include "chassis.h"
#include "k230_link.h"
#include "scheduler.h"
#include "tracker.h"
#include "ultrasonic.h"

#include "stm32g4xx_hal.h"

void App_Init(void)
{
    BspLed_Init();
    BspBuzzer_Init();
    BspKey_Init();
    BspMotor_Init();
    BspEncoder_Init();
    (void)BspServo_Init();
    Chassis_Init();
    Tracker_Init();
    Ultrasonic_Init();
    K230Link_Init();
    BtLink_Init();
    AppMode_Init();

    BspUart_Init(BSP_UART_K230);
    BspUart_Init(BSP_UART_BT);

    Scheduler_Init();
    AppTasks_Register();
}

void App_Run(void)
{
    Scheduler_Run(HAL_GetTick());
}
