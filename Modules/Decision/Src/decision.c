#include "decision.h"

#include "stm32g4xx_hal.h"     /* HAL_GetTick */

/* ────────────────────────────────────────────────────────────
 * Decision 域实现
 *
 * 模式状态机：收到事件后切换模式，每个模式有自身的控制逻辑。
 *
 * 当前各模式只实现最基本行为：
 *   STOP     → target = 0
 *   MANUAL   → target 来自蓝牙命令
 *   LINE_FOLLOW → 从循迹偏差计算（P 控制）
 *   AVOIDANCE → 停车等待（后续扩展）
 *   INSPECTION → 预留
 *
 * 所有模式卡在安全限幅内（Decision 层第一道，Motion 层第二道）。
 * ──────────────────────────────────────────────────────────── */

static DecisionState s_state;

/* ── 安全限幅 ── */
#define SPEED_MAX_MPS   1.0f
#define STEER_MAX_RAD   0.5f

/* 循迹 PID 参数 */
#define LINE_KP    0.0005f     /* 循迹偏差→转角比例 */

static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ── 初始化 ── */

void Decision_Init(void)
{
    s_state.mode            = SYS_MODE_STOP;
    s_state.prev_mode       = SYS_MODE_STOP;
    s_state.mode_enter_ms   = HAL_GetTick();
    s_state.target_speed_mps  = 0.0f;
    s_state.target_steer_rad  = 0.0f;
}

/* ── 20ms 周期 ── */

void Decision_Task20ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    /* ── 步骤 1：故障仲裁（最高优先级） ── */
    if (pool->fault.emergency_stop)
    {
        pool->mode = SYS_MODE_ERROR;
        pool->target.speed_mps       = 0.0f;
        pool->target.steer_angle_rad = 0.0f;
        pool->target.valid           = true;
        return;
    }

    if (pool->fault.heartbeat_lost)
    {
        pool->mode = SYS_MODE_STOP;
        pool->target.speed_mps       = 0.0f;
        pool->target.steer_angle_rad = 0.0f;
        pool->target.valid           = true;
        return;
    }

    /* ── 步骤 2：按键事件（次高优先级） ── */
    if (pool->event.key_stop_clicked)
    {
        pool->mode                    = SYS_MODE_STOP;
        s_state.mode                  = SYS_MODE_STOP;
        s_state.mode_enter_ms         = HAL_GetTick();
        pool->target.speed_mps        = 0.0f;
        pool->target.steer_angle_rad  = 0.0f;
        pool->target.valid            = true;
        return;
    }

    if (pool->event.key_mode_clicked)
    {
        /* 模式轮转：STOP → MANUAL → LINE_FOLLOW → STOP */
        SystemMode next[] = {SYS_MODE_STOP, SYS_MODE_MANUAL, SYS_MODE_LINE_FOLLOW};
        int cur = s_state.mode;
        int ni  = (cur >= 2) ? 0 : cur + 1;
        s_state.mode         = next[ni];
        s_state.mode_enter_ms = HAL_GetTick();
        pool->mode           = s_state.mode;
    }

    /* ── 步骤 3：蓝牙命令 ── */
    if (pool->event.bt_command_ready)
    {
        BtCommand *cmd = &pool->comm.bt_command;

        if (cmd->type == BT_COMMAND_STOP)
        {
            s_state.mode = SYS_MODE_STOP;
            s_state.mode_enter_ms = HAL_GetTick();
        }
        else if (cmd->type == BT_COMMAND_MANUAL_MOVE)
        {
            s_state.mode = SYS_MODE_MANUAL;
            s_state.mode_enter_ms = HAL_GetTick();
        }
    }

    /* ── 步骤 4：同步 mode 到 pool ── */
    pool->mode = s_state.mode;

    /* ── 步骤 5：各模式的持续控制逻辑 ── */

    float speed = 0.0f;
    float steer = 0.0f;

    switch (s_state.mode)
    {
    case SYS_MODE_STOP:
    case SYS_MODE_ERROR:
        speed = 0.0f;
        steer = 0.0f;
        break;

    case SYS_MODE_MANUAL:
        if (pool->comm.bt_command.valid &&
            pool->comm.bt_command.type == BT_COMMAND_MANUAL_MOVE)
        {
            /* arg0 = speed_permille, arg1 = steer_permille（蓝牙遥控器） */
            speed = clamp((float)pool->comm.bt_command.arg0 * 0.001f,
                          -SPEED_MAX_MPS, SPEED_MAX_MPS);
            steer = clamp((float)pool->comm.bt_command.arg1 * 0.0005f,
                          -STEER_MAX_RAD, STEER_MAX_RAD);
        }
        break;

    case SYS_MODE_LINE_FOLLOW:
        if (pool->sensor.track_valid)
        {
            speed = 0.3f;                               /* 固定低速巡线 */
            steer = clamp((float)pool->sensor.track_error * LINE_KP,
                          -STEER_MAX_RAD, STEER_MAX_RAD);
        }
        break;

    case SYS_MODE_INSPECTION:
    case SYS_MODE_AVOIDANCE:
        speed = 0.0f;                                   /* 预留 */
        steer = 0.0f;
        break;
    }

    /* ── 写入 target ── */
    pool->target.speed_mps       = speed;
    pool->target.steer_angle_rad = steer;
    pool->target.valid           = true;

    /* 保持本地快照 */
    s_state.target_speed_mps = speed;
    s_state.target_steer_rad = steer;
}

/* ── 状态查询 ── */

DecisionState Decision_GetState(void)
{
    return s_state;
}
