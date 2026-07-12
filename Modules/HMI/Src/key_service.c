/* ────────────────────────────────────────────────────────────
 * 按键事件服务（HMI 域内部使用）
 *
 * 读取 BspKey 的单击事件，写入 pool->event。
 * 不直接控制底盘，只写事件标志。
 *
 * 按键映射：
 *   KEY1 → pool->event.key_stop_clicked   (紧急停车)
 *   KEY2 → pool->event.key_mode_clicked   (手动/巡线切换)
 *   KEY3 → pool->event.key_task_clicked   (普通循线 / K230循线任务切换)
 *   KEY4 → 短按切换 Debug，长按强制打开 Debug
 * ──────────────────────────────────────────────────────────── */

#include "bsp_key.h"
#include "hmi_internal.h"
#include "system_state_pool.h"

#define USER_KEY_LONG_PRESS_MS 800U

void KeyService_Process10ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    if (BspKey_TakeClickedEvent(BSP_KEY_1))
    {
        pool->event.key_stop_clicked = true;
    }

    if (BspKey_TakeClickedEvent(BSP_KEY_2))
    {
        pool->event.key_mode_clicked = true;
    }

    if (BspKey_TakeClickedEvent(BSP_KEY_3))
    {
        pool->event.key_task_clicked = true;
    }

    if (BspKey_TakeReleasedEvent(BSP_KEY_4))
    {
        BspKeyInfo info = BspKey_GetInfo(BSP_KEY_4);
        if (info.pressed_time_ms >= USER_KEY_LONG_PRESS_MS)
        {
            pool->event.key_user_long_pressed = true;
            pool->debug.enabled = true;
        }
        else
        {
            pool->event.key_user_clicked = true;
            pool->debug.enabled = !pool->debug.enabled;
        }
    }
}
