/* ────────────────────────────────────────────────────────────
 * Decision 域实现
 *
 * 模式状态机：读 event/fault/sensor/comm → 仲裁模式 → 算 target
 * 不操作硬件，只写 pool->mode + pool->target
 *
 * 模式切换优先级（从高到低）：
 *   emergency_stop     → SYS_MODE_ERROR
 *   heartbeat_lost     → SYS_MODE_STOP
 *   key_stop_clicked   → SYS_MODE_STOP
 *   key_mode_clicked   → 手动 / 巡线模式切换
 *   key_task_clicked   → 启动巡线自动任务
 *   bt_command(STOP)   → SYS_MODE_STOP
 *   bt_command(AUTO,n) → SYS_MODE_LINE_FOLLOW / 预留自动任务
 *   bt_command(speed,angle) → SYS_MODE_MANUAL（目标锁存）
 *
 * 循迹控制：
 *   speed = VEHICLE_LINE_FOLLOW_SPEED_MPS
 *   steer = track_error × VEHICLE_LINE_FOLLOW_KP
 * ──────────────────────────────────────────────────────────── */

#include "decision.h"

#include "vehicle_config.h"
#include "stm32g4xx_hal.h"     /* HAL_GetTick */

static DecisionState s_state;

static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float steer_left_limit_rad(void)
{
    return VEHICLE_STEER_LEFT_CMD_LIMIT_RAD;
}

static float steer_right_limit_rad(void)
{
    return VEHICLE_STEER_RIGHT_CMD_LIMIT_RAD;
}

void Decision_Init(void)
{
    s_state.mode              = SYS_MODE_MANUAL;
    s_state.prev_mode         = SYS_MODE_MANUAL;
    s_state.mode_enter_ms     = HAL_GetTick();
    s_state.target_speed_mps  = 0.0f;
    s_state.target_steer_rad  = 0.0f;
}

void Decision_Task20ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    /* ── 故障仲裁（最高优先） ── */
    if (pool->fault.emergency_stop || pool->fault.heartbeat_lost)
    {
        pool->mode = pool->fault.emergency_stop ? SYS_MODE_ERROR : SYS_MODE_STOP;
        pool->auto_task = AUTO_TASK_NONE;
        s_state.target_speed_mps = 0.0f;
        s_state.target_steer_rad = 0.0f;
        pool->target = (SystemTarget){0};
        return;
    }

    /* ── 按键事件 ── */
    if (pool->event.key_stop_clicked)
    {
        s_state.mode = SYS_MODE_STOP;
        s_state.mode_enter_ms = HAL_GetTick();
        pool->auto_task = AUTO_TASK_NONE;
        s_state.target_speed_mps = 0.0f;
        s_state.target_steer_rad = 0.0f;
    }

    if (pool->event.key_mode_clicked)
    {
        if (s_state.mode == SYS_MODE_LINE_FOLLOW)
        {
            s_state.mode = SYS_MODE_MANUAL;
            pool->auto_task = AUTO_TASK_NONE;
            s_state.target_speed_mps = 0.0f;
            s_state.target_steer_rad = 0.0f;
        }
        else
        {
            s_state.mode = SYS_MODE_LINE_FOLLOW;
            pool->auto_task = AUTO_TASK_LINE_FOLLOW;
        }
        s_state.mode_enter_ms = HAL_GetTick();
    }

    if (pool->event.key_task_clicked)
    {
        s_state.mode = SYS_MODE_LINE_FOLLOW;
        pool->auto_task = AUTO_TASK_LINE_FOLLOW;
        s_state.mode_enter_ms = HAL_GetTick();
    }

    /* ── 蓝牙命令 ── */
    if (pool->event.bt_command_ready)
    {
        BtCommand *cmd = &pool->comm.bt_command;
        if (cmd->type == BT_COMMAND_STOP)
        {
            s_state.mode = SYS_MODE_STOP;
            s_state.mode_enter_ms = HAL_GetTick();
            pool->auto_task = AUTO_TASK_NONE;
            s_state.target_speed_mps = 0.0f;
            s_state.target_steer_rad = 0.0f;
        }
        else if (cmd->type == BT_COMMAND_START_TASK)
        {
            s_state.mode = SYS_MODE_LINE_FOLLOW;
            s_state.mode_enter_ms = HAL_GetTick();

            if (cmd->arg0 == 1)
            {
                pool->auto_task = AUTO_TASK_LINE_FOLLOW;
            }
            else if (cmd->arg0 == 2)
            {
                pool->auto_task = AUTO_TASK_LINE_FOLLOW_OBSTACLE;
            }
            else if (cmd->arg0 == 3)
            {
                pool->auto_task = AUTO_TASK_INSPECTION;
            }
            else
            {
                pool->auto_task = AUTO_TASK_NONE;
            }
        }
        else if (cmd->type == BT_COMMAND_MANUAL_MOVE)
        {
            s_state.mode = SYS_MODE_MANUAL;
            s_state.mode_enter_ms = HAL_GetTick();
            pool->auto_task = AUTO_TASK_NONE;
            s_state.target_speed_mps = clamp(
                ((float)cmd->arg0 * VEHICLE_MAX_SPEED_MPS) / 100.0f,
                -VEHICLE_MAX_SPEED_MPS, VEHICLE_MAX_SPEED_MPS);
            /* 角度线性映射：蓝牙输入值 / INPUT_MAX × 机械极限 = 实际转向角 */
            float ratio = (float)cmd->arg1 / VEHICLE_MANUAL_INPUT_MAX_DEG;
            if (ratio >= 0.0f) {
                s_state.target_steer_rad = ratio * steer_right_limit_rad();
            } else {
                s_state.target_steer_rad = ratio * steer_left_limit_rad();
            }
            s_state.target_steer_rad = clamp(s_state.target_steer_rad,
                                             -steer_left_limit_rad(), steer_right_limit_rad());
        }
    }

    /* ── 同步 mode ── */
    pool->mode = s_state.mode;

    /* ── 各模式控制逻辑 ── */
    float speed = 0.0f, steer = 0.0f;

    switch (s_state.mode)
    {
    case SYS_MODE_STOP:
    case SYS_MODE_ERROR:
        speed = 0.0f; steer = 0.0f;
        break;

    case SYS_MODE_MANUAL:
        speed = s_state.target_speed_mps;
        steer = s_state.target_steer_rad;
        break;

    case SYS_MODE_LINE_FOLLOW:
        if (pool->sensor.track_valid)
        {
            speed = VEHICLE_LINE_FOLLOW_SPEED_MPS;
            steer = clamp(-(float)pool->sensor.track_error * VEHICLE_LINE_FOLLOW_KP,
                          -steer_left_limit_rad(), steer_right_limit_rad());
        }
        if ((pool->auto_task == AUTO_TASK_LINE_FOLLOW_OBSTACLE) &&
            pool->sensor.obstacle_near)
        {
            speed = 0.0f;
            steer = 0.0f;
        }
        break;

    default:
        break;
    }

    pool->target.speed_mps       = speed;
    pool->target.steer_angle_rad = steer;
    pool->target.valid           = true;
}

DecisionState Decision_GetState(void) { return s_state; }
