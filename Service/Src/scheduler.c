/* ────────────────────────────────────────────────────────────
 * 协作式周期任务调度器实现
 *
 * 核心逻辑：每 tick 遍历任务表 → 检查 (now - last_run) ≥ period → 执行
 * O(N) 复杂度，任务少时足够。非优先级调度，按注册顺序执行。
 *
 * start_delay 技巧：初始 last_run_ms 往回拨，让第一次执行推迟 start_delay ms。
 *   正常：last_run_ms = 0 → 下 tick 即执行
 *   延迟：last_run_ms = 0U - delay → 下 tick 时 now-(0U-delay)=now+delay≥period 才执行
 *   利用了 uint32_t 无符号回绕。
 *
 * 参数说明：
 *   SCHEDULER_MAX_TASKS = 最大可注册任务数。
 *                         当前 16 个够用（实际用了 8 个）。
 *                         改大 → 多占 40 字节/Task 的 RAM。
 *                        SchedulerTask 结构体大小 ≈ 24 字节/Task。
 * ──────────────────────────────────────────────────────────── */

#include "scheduler.h"

/* ════════════════════════════════════════════════════════════
 * 调度器参数
 * ════════════════════════════════════════════════════════════ */

#define SCHEDULER_MAX_TASKS 16U             /* 最大任务数，当前 8 个已用 */

static SchedulerTask s_tasks[SCHEDULER_MAX_TASKS];          /* 任务表，约 384 字节 RAM */
static uint32_t      s_task_count;                          /* 已注册数量              */

/* ── 初始化 ── */

void Scheduler_Init(void) { s_task_count = 0U; }

/* ── 添加任务 ── */

bool Scheduler_AddTask(const char *name,
                       SchedulerTaskFn function, void *context,
                       uint32_t period_ms, uint32_t start_delay_ms)
{
    if ((function == 0) || (period_ms == 0U) || (s_task_count >= SCHEDULER_MAX_TASKS))
        return false;

    SchedulerTask *task = &s_tasks[s_task_count++];
    task->name     = name;
    task->function = function;
    task->context  = context;
    task->period_ms = period_ms;
    task->last_run_ms = (start_delay_ms == 0U) ? 0U : (0U - start_delay_ms);
    task->enabled = true;
    return true;
}

/* ── 周期执行 ── */

void Scheduler_Run(uint32_t now_ms)
{
    for (uint32_t i = 0U; i < s_task_count; ++i)
    {
        SchedulerTask *task = &s_tasks[i];
        if (!task->enabled) continue;
        if ((uint32_t)(now_ms - task->last_run_ms) >= task->period_ms)
        {
            task->last_run_ms = now_ms;
            task->function(task->context);
        }
    }
}
