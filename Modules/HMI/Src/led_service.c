/* ────────────────────────────────────────────────────────────
 * LED 状态服务（HMI 域内部使用）
 *
 * 根据 pool->mode 更新 4 个 LED 的显示策略：
 *
 *   STOP:       STATE_LED 每 500ms 翻转（快闪），其余灭
 *   MANUAL:     LED1 + STATE_LED 常亮，其余灭
 *   LINE_FOLLOW: LED1 + LED2 常亮，其余灭
 *   AVOIDANCE:  LED3 每 500ms 翻转（快闪），其余灭
 *   INSPECTION: LED3 常亮，其余灭
 *   ERROR:      LED1~3 + STATE_LED 全部每 500ms 翻转（全闪）
 *
 * 闪烁周期：500ms（每 500ms Task 调用一次，相位由 s_tick 控制）。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_led.h"
#include "hmi_internal.h"
#include "system_state_pool.h"

static uint32_t s_tick;       /* 500ms 累计计数器，控制闪烁相位 */

void LedService_Update(SystemStatePool *pool)
{
    if (pool == 0) return;

    ++s_tick;
    bool half = (s_tick & 1U) != 0U;     /* 500ms 相位：奇次=亮/偶次=灭 */

    switch (pool->mode)
    {
    case SYS_MODE_STOP:
        BspLed_Set(BSP_LED_1,     false);
        BspLed_Set(BSP_LED_2,     false);
        BspLed_Set(BSP_LED_3,     false);
        BspLed_Set(BSP_LED_STATE, half);
        break;

    case SYS_MODE_MANUAL:
        BspLed_Set(BSP_LED_1,     true);
        BspLed_Set(BSP_LED_2,     false);
        BspLed_Set(BSP_LED_3,     false);
        BspLed_Set(BSP_LED_STATE, true);
        break;

    case SYS_MODE_LINE_FOLLOW:
        BspLed_Set(BSP_LED_1,     true);
        BspLed_Set(BSP_LED_2,     true);
        BspLed_Set(BSP_LED_3,     false);
        BspLed_Set(BSP_LED_STATE, false);
        break;

    case SYS_MODE_AVOIDANCE:
        BspLed_Set(BSP_LED_1,     false);
        BspLed_Set(BSP_LED_2,     false);
        BspLed_Set(BSP_LED_3,     half);
        BspLed_Set(BSP_LED_STATE, false);
        break;

    case SYS_MODE_INSPECTION:
        BspLed_Set(BSP_LED_1,     false);
        BspLed_Set(BSP_LED_2,     false);
        BspLed_Set(BSP_LED_3,     true);
        BspLed_Set(BSP_LED_STATE, false);
        break;

    case SYS_MODE_ERROR:
        BspLed_Set(BSP_LED_1,     half);
        BspLed_Set(BSP_LED_2,     half);
        BspLed_Set(BSP_LED_3,     half);
        BspLed_Set(BSP_LED_STATE, !half);
        break;
    }
}
