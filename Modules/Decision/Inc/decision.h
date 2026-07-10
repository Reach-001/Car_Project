#ifndef DECISION_H
#define DECISION_H

#include <stdbool.h>
#include <stdint.h>

#include "system_state_pool.h"

/* ────────────────────────────────────────────────────────────
 * Decision 域 —— 模式仲裁 + 目标计算
 *
 * 职责：读取所有输入（event / fault / sensor / comm），
 *       仲裁当前模式，计算目标速度和转角。
 *       不操作任何硬件，只写 pool->mode + pool->target。
 *
 * 仲裁优先级（从高到低）：
 *   emergency_stop → SYS_MODE_STOP
 *   heartbeat_lost → SYS_MODE_STOP
 *   obstacle_too_close → SYS_MODE_AVOIDANCE
 *   bt_command_ready → 蓝牙命令
 *   key_stop_clicked → SYS_MODE_STOP
 *   当前模式的持续控制
 *
 * 依赖：无（只读 pool 字段）
 * 禁止：调用任何 BSP 函数、Motion、Comm、Sensor
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    SystemMode mode;
    SystemMode prev_mode;
    uint32_t   mode_enter_ms;
    float      target_speed_mps;
    float      target_steer_rad;
} DecisionState;

/* ── 生命周期 ── */

void Decision_Init(void);

/** 20ms 周期调用：读输入 → 仲裁模式 → 计算目标 → 写入 pool */
void Decision_Task20ms(SystemStatePool *pool);

DecisionState Decision_GetState(void);

#endif /* DECISION_H */
