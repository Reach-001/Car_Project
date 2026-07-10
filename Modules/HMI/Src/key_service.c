/* ────────────────────────────────────────────────────────────
 * 按键事件服务（HMI 域内部使用）
 *
 * 读取 BspKey 的单击事件，写入 pool->event。
 * 不直接控制底盘，只写事件标志。
 *
 * 按键映射：
 *   KEY1 → pool->event.key_stop_clicked   (紧急停车)
 *   KEY2 → pool->event.key_mode_clicked   (模式切换)
 *   KEY3 → pool->event.key_task_clicked   (任务启停)
 *   KEY4 → pool->event.key_user_clicked   (用户自定义)
 * ──────────────────────────────────────────────────────────── */

#include "bsp_key.h"
#include "hmi_internal.h"
#include "system_state_pool.h"

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

    if (BspKey_TakeClickedEvent(BSP_KEY_4))
    {
        pool->event.key_user_clicked = true;
    }
}
