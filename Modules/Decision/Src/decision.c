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
static int16_t       s_line_last_error;
static uint32_t      s_line_last_valid_ms;
static bool          s_line_has_valid;

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

static float absf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

static void reset_line_follow_state(void)
{
    s_line_last_error = 0;
    s_line_last_valid_ms = HAL_GetTick();
    s_line_has_valid = false;
}

static float line_follow_clamp_steer(float steer)
{
    return clamp(steer, -steer_left_limit_rad(), steer_right_limit_rad());
}

static float line_follow_speed_from_error(int16_t error)
{
    float abs_error = absf_local((float)error);
    float normalized = abs_error / 2000.0f;
    float speed_range = VEHICLE_LINE_FOLLOW_SPEED_MPS - VEHICLE_LINE_FOLLOW_MIN_SPEED_MPS;
    float speed;

    if (normalized > 1.0f) normalized = 1.0f;
    if (speed_range < 0.0f) speed_range = 0.0f;

    speed = VEHICLE_LINE_FOLLOW_SPEED_MPS -
            (speed_range * VEHICLE_LINE_FOLLOW_SLOWDOWN_GAIN * normalized);

    return clamp(speed,
                 VEHICLE_LINE_FOLLOW_MIN_SPEED_MPS,
                 VEHICLE_LINE_FOLLOW_SPEED_MPS);
}

static float line_follow_steer_from_error(int16_t error, int16_t error_delta)
{
    float steer = VEHICLE_LINE_FOLLOW_STEER_SIGN *
                  ((VEHICLE_LINE_FOLLOW_KP * (float)error) +
                   (VEHICLE_LINE_FOLLOW_KD * (float)error_delta));
    return line_follow_clamp_steer(steer);
}

static void compute_line_follow_target(const SystemStatePool *pool,
                                       float *speed_out,
                                       float *steer_out)
{
    uint32_t now = HAL_GetTick();

    if ((pool == 0) || (speed_out == 0) || (steer_out == 0))
    {
        return;
    }

    if (pool->sensor.track_valid)
    {
        int16_t error = pool->sensor.track_error;
        int16_t error_delta = s_line_has_valid ? (int16_t)(error - s_line_last_error) : 0;

        s_line_last_error = error;
        s_line_last_valid_ms = now;
        s_line_has_valid = true;

        *speed_out = line_follow_speed_from_error(error);
        *steer_out = line_follow_steer_from_error(error, error_delta);
        return;
    }

    if (!s_line_has_valid)
    {
        *speed_out = 0.0f;
        *steer_out = 0.0f;
        return;
    }

    uint32_t lost_ms = now - s_line_last_valid_ms;
    if (lost_ms <= VEHICLE_LINE_FOLLOW_LOST_HOLD_MS)
    {
        *speed_out = line_follow_speed_from_error(s_line_last_error);
        *steer_out = line_follow_steer_from_error(s_line_last_error, 0);
    }
    else if (lost_ms <= VEHICLE_LINE_FOLLOW_LOST_STOP_MS)
    {
        float search_dir = (s_line_last_error >= 0) ? 1.0f : -1.0f;
        *speed_out = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
        *steer_out = line_follow_clamp_steer(VEHICLE_LINE_FOLLOW_STEER_SIGN *
                                             search_dir *
                                             VEHICLE_LINE_FOLLOW_SEARCH_STEER_DEG *
                                             VEHICLE_DEG_TO_RAD);
    }
    else
    {
        *speed_out = 0.0f;
        *steer_out = 0.0f;
    }
}

void Decision_Init(void)
{
    s_state.mode              = SYS_MODE_MANUAL;
    s_state.prev_mode         = SYS_MODE_MANUAL;
    s_state.mode_enter_ms     = HAL_GetTick();
    s_state.target_speed_mps  = 0.0f;
    s_state.target_steer_rad  = 0.0f;
    reset_line_follow_state();
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
        reset_line_follow_state();
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
        reset_line_follow_state();
    }

    if (pool->event.key_mode_clicked)
    {
        if (s_state.mode == SYS_MODE_LINE_FOLLOW)
        {
            s_state.mode = SYS_MODE_MANUAL;
            pool->auto_task = AUTO_TASK_NONE;
            s_state.target_speed_mps = 0.0f;
            s_state.target_steer_rad = 0.0f;
            reset_line_follow_state();
        }
        else
        {
            s_state.mode = SYS_MODE_LINE_FOLLOW;
            pool->auto_task = AUTO_TASK_LINE_FOLLOW;
            reset_line_follow_state();
        }
        s_state.mode_enter_ms = HAL_GetTick();
    }

    if (pool->event.key_task_clicked)
    {
        s_state.mode = SYS_MODE_LINE_FOLLOW;
        pool->auto_task = AUTO_TASK_LINE_FOLLOW;
        s_state.mode_enter_ms = HAL_GetTick();
        reset_line_follow_state();
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
            reset_line_follow_state();
        }
        else if (cmd->type == BT_COMMAND_START_TASK)
        {
            s_state.mode = SYS_MODE_LINE_FOLLOW;
            s_state.mode_enter_ms = HAL_GetTick();
            reset_line_follow_state();

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
            reset_line_follow_state();
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
        compute_line_follow_target(pool, &speed, &steer);
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
    s_state.target_speed_mps     = speed;
    s_state.target_steer_rad     = steer;
}

DecisionState Decision_GetState(void) { return s_state; }
