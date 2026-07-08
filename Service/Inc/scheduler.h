#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * 协作式周期任务调度器 —— 通用服务模块
 *
 * 非 RTOS，任务必须短小、非阻塞。禁止在任务中调 HAL_Delay()。
 * 每次 App_Run() 调用时，遍历任务表，执行所有到期的任务。
 *
 * 最大支持 16 个任务。start_delay_ms 用于错开启动，
 * 避免多个同周期任务在同一 tick 集中执行。
 *
 * 时间源由外部提供（HAL_GetTick()），调度器本身无平台依赖。
 * ──────────────────────────────────────────────────────────── */

/* 任务函数签名：入参 context 即注册时传入的上下文指针 */
typedef void (*SchedulerTaskFn)(void *context);

typedef struct
{
    const char       *name;          /* 任务名（调试用） */
    SchedulerTaskFn   function;      /* 任务函数指针                      */
    void             *context;       /* 传递给任务函数的上下文           */
    uint32_t          period_ms;     /* 执行周期（ms）                    */
    uint32_t          last_run_ms;   /* 上次执行时刻（内部维护）          */
    bool              enabled;       /* 是否启用                          */
} SchedulerTask;

/* ── 生命周期 ── */

/** 初始化调度器内部状态 */
void Scheduler_Init(void);

/** 注册一个周期任务
 *  @param name           任务名（字符串常量）
 *  @param function       任务函数指针
 *  @param context        传递给 task 的上下文
 *  @param period_ms      执行周期（ms）
 *  @param start_delay_ms 首次执行的延迟（用于错峰），0 = 立即执行
 *  @return 注册成功返回 true，任务表满返回 false */
bool Scheduler_AddTask(const char *name,
                       SchedulerTaskFn function,
                       void *context,
                       uint32_t period_ms,
                       uint32_t start_delay_ms);

/** 执行所有到期的任务
 *  @param now_ms 当前系统时间（通常是 HAL_GetTick() 返回值）
 *  每次主循环调用一次 */
void Scheduler_Run(uint32_t now_ms);

#endif /* SCHEDULER_H */
