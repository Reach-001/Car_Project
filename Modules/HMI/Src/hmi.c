#include "hmi.h"
#include "hmi_internal.h"

#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_led.h"

/* ────────────────────────────────────────────────────────────
 * HMI 域聚合入口
 *
 * 子模块：key_service（按键→event）、buzzer_service（故障→提示音）、
 *        led_service（模式→LED）
 *
 * 注意：按键去抖需要 10ms 频率，但 HMI 域以 500ms 运行。
 *       所以按键事件由 KeyService_Task10ms() 单独高频调用处理，
 *       HMI_Task500ms 只负责 LED 和蜂鸣器的慢速更新。
 * ──────────────────────────────────────────────────────────── */

static HmiState s_state;

/* ── 初始化 ── */

void Hmi_Init(void)
{
    s_state.led1_on     = false;
    s_state.led2_on     = false;
    s_state.led3_on     = false;
    s_state.state_led_on = false;
    s_state.buzzer_active = false;
}

/* ── 键事件处理（需每 10ms 调用一次，由 App 任务调度） ── */

void Hmi_KeyTask10ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    BspKey_Task10ms();
    KeyService_Process10ms(pool);
}

/* ── 500ms 周期 ── */

void Hmi_Task500ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    BuzzerService_Update(pool);
    LedService_Update(pool);
}

/* ── 状态查询 ── */

HmiState Hmi_GetState(void)
{
    return s_state;
}
