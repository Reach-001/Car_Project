/* ────────────────────────────────────────────────────────────
 * 蜂鸣器状态服务（HMI 域内部使用）
 *
 * 根据 pool->fault 播放对应提示音（仅一次，不放完不重复）。
 *
 * 映射：
 *   obstacle_too_close → OBSTACLE 急促提示音（连续）
 *   emergency_stop     → ERROR 提示音
 *   mode 刚切换 → START 提示音（由 Decision 的 mode 变化触发）等后续扩展
 * ──────────────────────────────────────────────────────────── */

#include "bsp_buzzer.h"
#include "hmi_internal.h"
#include "system_state_pool.h"

void BuzzerService_Update(SystemStatePool *pool)
{
    if (pool == 0) return;

    /* 故障模式的提示音 */
    if (pool->fault.obstacle_too_close)
    {
        if (!BspBuzzer_IsActive())
        {
            BspBuzzer_Play(BUZZER_PATTERN_OBSTACLE);
        }
    }
    else if (pool->fault.emergency_stop)
    {
        if (!BspBuzzer_IsActive())
        {
            BspBuzzer_Play(BUZZER_PATTERN_ERROR);
        }
    }

    /* 必须调 Task10ms 驱动提示音状态机 */
    BspBuzzer_Task10ms();
}
