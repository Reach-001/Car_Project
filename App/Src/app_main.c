/* ────────────────────────────────────────────────────────────
 * App 入口（V2 Module 层架构）
 *
 * 初始化顺序（严格按依赖关系）：
 *   1. BSP 硬件层
 *   2. SystemStatePool
 *   3. Module 域（从外设 → 控制）
 *   4. 调度器 + 任务注册
 *
 * 主循环：更新 tick → 调度所有周期任务 → IWDG 喂狗
 * ──────────────────────────────────────────────────────────── */

#include "app_main.h"

#if TEST_SELECT != TEST_NONE
#include "app_test.h"
#else
#include "app_tasks.h"
#endif

#include "system_state_pool.h"
#include "bsp_buzzer.h"
#include "bsp_encoder.h"
#include "bsp_gpio_sensor.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"
#include "sensor_domain.h"
#include "estimation.h"
#include "motion.h"
#include "decision.h"
#include "comm.h"
#include "hmi.h"
#include "debug_trace.h"
#include "scheduler.h"
#include "iwdg.h"

#include "stm32g4xx_hal.h"

/* 全局状态池（唯一跨域数据交换中心） */
SystemStatePool g_state;

void App_Init(void)
{
#if TEST_SELECT != TEST_NONE
    AppTest_Init();
    return;
#endif

    /* ── 1. BSP 硬件层 ── */
    BspLed_Init();
    BspBuzzer_Init();
    BspKey_Init();
    BspMotor_Init();
    BspEncoder_Init();
    (void)BspServo_Init();
    BspGpioSensor_Init();

    BspUart_Init(BSP_UART_K230);
    BspUart_Init(BSP_UART_BT);

    /* ── 2. SystemStatePool ── */
    SystemStatePool_Init(&g_state);

    /* ── 3. Module 域（由外而内） ── */
    Sensor_Init();
    Estimation_Init();
    Motion_Init();
    Comm_Init();
    Decision_Init();
    Hmi_Init();
    DebugTrace_Init();

    /* ── 4. 调度器 ── */
    Scheduler_Init();
    AppTasks_Register();
}

void App_Run(void)
{
#if TEST_SELECT != TEST_NONE
    HAL_IWDG_Refresh(&hiwdg);
    AppTest_Run();
    return;
#endif

    g_state.tick_ms = HAL_GetTick();
    Scheduler_Run(g_state.tick_ms);
}
