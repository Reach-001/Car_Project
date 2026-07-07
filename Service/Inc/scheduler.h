#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*SchedulerTaskFn)(void *context);

typedef struct
{
    const char *name;
    SchedulerTaskFn function;
    void *context;
    uint32_t period_ms;
    uint32_t last_run_ms;
    bool enabled;
} SchedulerTask;

void Scheduler_Init(void);
bool Scheduler_AddTask(const char *name,
                       SchedulerTaskFn function,
                       void *context,
                       uint32_t period_ms,
                       uint32_t start_delay_ms);
void Scheduler_Run(uint32_t now_ms);

#endif
