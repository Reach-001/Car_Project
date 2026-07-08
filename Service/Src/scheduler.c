#include "scheduler.h"

/* ────────────────────────────────────────────────────────────
 * 协作式周期任务调度器实现
 *
 * 核心逻辑：遍历任务表 → 检查 (now - last_run) ≥ period → 执行
 * O(N) 复杂度，任务少时足够。不是优先级调度，按注册顺序执行。
 *
 * start_delay 实现技巧：初始 last_run_ms 往回拨，让首次执行延后。
 *   正常：last_run_ms = 0 → 首次 tick 立即执行
 *   延迟：last_run_ms = 0 - start_delay → 首次 tick 时 now-(0-delay)=now+delay≥period 才执行
 *   实际上用 0U - start_delay_ms 得到大无符号数，依赖 uint32_t 回绕来延时。
 * ──────────────────────────────────────────────────────────── */

#define SCHEDULER_MAX_TASKS 16U         /* 最大注册任务数 */

static SchedulerTask s_tasks[SCHEDULER_MAX_TASKS];   /* 任务表 */
static uint32_t      s_task_count;                   /* 已注册数量 */

/* ── 初始化 ── */

void Scheduler_Init(void)
{
    s_task_count = 0U;
}

/* ── 添加任务 ── */

bool Scheduler_AddTask(const char *name,
                       SchedulerTaskFn function,
                       void *context,
                       uint32_t period_ms,
                       uint32_t start_delay_ms)
{
    /* 参数检查 */
    if ((function == 0) || (period_ms == 0U) || (s_task_count >= SCHEDULER_MAX_TASKS))
    {
        return false;
    }

    /* 填充任务表项 */
    SchedulerTask *task = &s_tasks[s_task_count++];
    task->name     = name;
    task->function = function;
    task->context  = context;
    task->period_ms = period_ms;

    /* start_delay=0 → 下次 tick 即可执行
     * start_delay>0 → 首次执行推迟 start_delay ms */
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
        if (!task->enabled) { continue; }

        /* 检查是否到期：(now - last_run) ≥ period
         * uint32_t 减法自带回绕处理 */
        if ((uint32_t)(now_ms - task->last_run_ms) >= task->period_ms)
        {
            task->last_run_ms = now_ms;
            task->function(task->context);
        }
    }
}
