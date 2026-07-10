/* ────────────────────────────────────────────────────────────
 * V2 任务注册（时间槽隔离）
 *
 * 10ms 硬实时槽：Estimation → Motion
 *   （先估计速度，再执行控制闭环）
 * 20ms 逻辑槽：HMI_Key → Sensor → Comm → Decision
 *   （按键先采集事件，传感器和通信更新数据，
 *     Decision 最后计算目标）
 * Debug 曲线：100ms 输出，默认关闭，避免干扰普通蓝牙控制。
 * 500ms 低优先槽：HMI
 *   （LED/蜂鸣器更新，不影响控制实时性）
 * ──────────────────────────────────────────────────────────── */

#include "app_tasks.h"

#include "system_state_pool.h"
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

/* 引用 App 层的全局状态池 */
extern SystemStatePool g_state;

/* ── 10ms 槽 ── */

static void Task_Estimation10ms(void *ctx)
{
    Estimation_Task10ms((SystemStatePool *)ctx);
}

static void Task_Motion10ms(void *ctx)
{
    Motion_Task10ms((SystemStatePool *)ctx);
}

/* ── 20ms 槽 ── */

static void Task_HmiKey10ms(void *ctx)
{
    Hmi_KeyTask10ms((SystemStatePool *)ctx);
}

static void Task_Sensor20ms(void *ctx)
{
    Sensor_Task20ms((SystemStatePool *)ctx);
}

static void Task_Comm20ms(void *ctx)
{
    Comm_Task20ms((SystemStatePool *)ctx);
}

static void Task_Decision20ms(void *ctx)
{
    SystemStatePool *pool = (SystemStatePool *)ctx;
    Decision_Task20ms(pool);
    /* 本周期消费者处理完成后，统一清理一次性事件 */
    SystemStatePool_ClearCycleEvents(pool);
}

static void Task_DebugCurve100ms(void *ctx)
{
    DebugTrace_Task100ms((SystemStatePool *)ctx);
}

/* ── 500ms 槽 ── */

static void Task_Hmi500ms(void *ctx)
{
    SystemStatePool *pool = (SystemStatePool *)ctx;
    Hmi_Task500ms(pool);
    HAL_IWDG_Refresh(&hiwdg);          /* 500ms 喂一次看门狗 */
}

/* ── 注册 ── */

void AppTasks_Register(void)
{
    /* 10ms：估计在前，控制在后 */
    Scheduler_AddTask("estimation", Task_Estimation10ms, &g_state, 10U, 0U);
    Scheduler_AddTask("motion",     Task_Motion10ms,     &g_state, 10U, 1U);

    /* 20ms：按键（10ms 去抖需要高频）→ 传感器 → 通信 → 决策最后 */
    Scheduler_AddTask("hmi_key",    Task_HmiKey10ms,    &g_state, 10U, 0U);
    Scheduler_AddTask("sensor",     Task_Sensor20ms,    &g_state, 20U, 2U);
    Scheduler_AddTask("comm",       Task_Comm20ms,      &g_state, 20U, 4U);
    Scheduler_AddTask("decision",   Task_Decision20ms,  &g_state, 20U, 6U);

    /* 100ms：蓝牙输出小端 float 曲线帧 */
    Scheduler_AddTask("debug_curve", Task_DebugCurve100ms, &g_state, 100U, 7U);

    /* 500ms：LED+蜂鸣器+喂狗 */
    Scheduler_AddTask("hmi",        Task_Hmi500ms,      &g_state, 500U, 0U);
}
