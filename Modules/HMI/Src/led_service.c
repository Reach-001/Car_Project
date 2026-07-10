/* ────────────────────────────────────────────────────────────
 * LED 状态服务（HMI 域内部使用）
 *
 * 根据 pool->mode 更新 LED 显示策略：
 *   STOP       → STATE_LED 快闪
 *   MANUAL     → LED1 + STATE_LED 常亮
 *   LINE_FOLLOW → LED1 + LED2 常亮
 *   AVOIDANCE  → LED3 快闪
 *   ERROR      → LED1~3 全闪
 * ──────────────────────────────────────────────────────────── */

#include "bsp_led.h"
#include "hmi_internal.h"
#include "system_state_pool.h"

static uint32_t s_tick;       /* 500ms 累计（每 500ms +1） */

void LedService_Update(SystemStatePool *pool)
{
    if (pool == 0) return;

    ++s_tick;
    bool half = (s_tick & 1U) != 0U;     /* 500ms 闪烁相位 */

    switch (pool->mode)
    {
    case SYS_MODE_STOP:
        BspLed_Set(BSP_LED_1, false);
        BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_3, false);
        BspLed_Set(BSP_LED_STATE, half);
        break;

    case SYS_MODE_MANUAL:
        BspLed_Set(BSP_LED_1, true);
        BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_3, false);
        BspLed_Set(BSP_LED_STATE, true);
        break;

    case SYS_MODE_LINE_FOLLOW:
        BspLed_Set(BSP_LED_1, true);
        BspLed_Set(BSP_LED_2, true);
        BspLed_Set(BSP_LED_3, false);
        BspLed_Set(BSP_LED_STATE, false);
        break;

    case SYS_MODE_AVOIDANCE:
        BspLed_Set(BSP_LED_1, false);
        BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_3, half);
        BspLed_Set(BSP_LED_STATE, false);
        break;

    case SYS_MODE_INSPECTION:
        BspLed_Set(BSP_LED_1, false);
        BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_3, true);
        BspLed_Set(BSP_LED_STATE, false);
        break;

    case SYS_MODE_ERROR:
        BspLed_Set(BSP_LED_1, half);
        BspLed_Set(BSP_LED_2, half);
        BspLed_Set(BSP_LED_3, half);
        BspLed_Set(BSP_LED_STATE, !half);
        break;
    }
}
