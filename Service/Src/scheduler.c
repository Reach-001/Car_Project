#include "scheduler.h"

#define SCHEDULER_MAX_TASKS 16U

static SchedulerTask s_tasks[SCHEDULER_MAX_TASKS];
static uint32_t s_task_count;

void Scheduler_Init(void)
{
    s_task_count = 0U;
}

bool Scheduler_AddTask(const char *name,
                       SchedulerTaskFn function,
                       void *context,
                       uint32_t period_ms,
                       uint32_t start_delay_ms)
{
    if ((function == 0) || (period_ms == 0U) || (s_task_count >= SCHEDULER_MAX_TASKS))
    {
        return false;
    }

    SchedulerTask *task = &s_tasks[s_task_count++];
    task->name = name;
    task->function = function;
    task->context = context;
    task->period_ms = period_ms;
    task->last_run_ms = start_delay_ms == 0U ? 0U : (0U - start_delay_ms);
    task->enabled = true;
    return true;
}

void Scheduler_Run(uint32_t now_ms)
{
    for (uint32_t i = 0U; i < s_task_count; ++i)
    {
        SchedulerTask *task = &s_tasks[i];
        if (!task->enabled)
        {
            continue;
        }

        if ((uint32_t)(now_ms - task->last_run_ms) >= task->period_ms)
        {
            task->last_run_ms = now_ms;
            task->function(task->context);
        }
    }
}
