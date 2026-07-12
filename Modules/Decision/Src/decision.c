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
 *   key_task_clicked   → 普通巡线 / K230巡线切换
 *   bt_command(STOP)   → SYS_MODE_STOP
 *   bt_command(AUTO,n) → 自动任务入口（AUTO,1巡线，AUTO,2 K230巡线）
 *   bt_command(speed,angle) → SYS_MODE_MANUAL（目标锁存）
 *
 * 循迹控制：
 *   有线时：优先让中心探头压在黑线上，并随误差增大自动降速
 *   3 号探头未压线时：锁定方向低速前进，直到 3 号重新扫到黑线
 * ──────────────────────────────────────────────────────────── */

#include "decision.h"

#include "vehicle_config.h"
#include "stm32g4xx_hal.h"     /* HAL_GetTick */

#define LINE_TRACK_1_MASK   (1U << 0)
#define LINE_TRACK_2_MASK   (1U << 1)
#define LINE_TRACK_4_MASK   (1U << 3)
#define LINE_TRACK_5_MASK   (1U << 4)

typedef void (*DecisionModeTargetFn)(const SystemStatePool *pool,
                                     float *speed,
                                     float *steer);

typedef struct
{
    SystemMode           mode;
    DecisionModeTargetFn target_fn;
} DecisionModeHandler;

typedef enum
{
    K230_AUTO_PARK_NONE = 0,
    K230_AUTO_PARK_REVERSE = 1,
    K230_AUTO_PARK_PARALLEL = 2
} K230AutoParkMode;

static DecisionState s_state;
static int16_t       s_line_last_error;
static int16_t       s_line_last_search_error;
static uint32_t      s_line_random_state;
static uint32_t      s_k230_stop_until_ms;
static float         s_k230_speed_limit_mps;
static uint8_t       s_line_last_track_bits;
static int8_t        s_line_hard_turn_dir;
static int8_t        s_k230_turn_dir;
static K230AutoParkMode s_k230_park_mode;
static bool          s_line_has_valid;
static bool          s_k230_turn_seen_off_center;

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
    s_line_last_search_error = 0;
    s_line_random_state ^= HAL_GetTick() ^ 0xA5A55A5AU;
    s_line_last_track_bits = 0U;
    s_line_hard_turn_dir = 0;
    s_line_has_valid = false;
}

static void reset_k230_auto_state(void)
{
    s_k230_stop_until_ms = 0U;
    s_k230_speed_limit_mps = VEHICLE_LINE_FOLLOW_SPEED_MPS;
    s_k230_turn_dir = 0;
    s_k230_park_mode = K230_AUTO_PARK_NONE;
    s_k230_turn_seen_off_center = false;
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

static bool line_follow_center_on_line(uint8_t track_bits)
{
    return ((track_bits & VEHICLE_LINE_FOLLOW_CENTER_MASK) != 0U);
}

static float line_follow_search_steer_from_error(int16_t error)
{
    float search_dir = (error >= 0) ? 1.0f : -1.0f;
    return line_follow_clamp_steer(VEHICLE_LINE_FOLLOW_STEER_SIGN *
                                   search_dir *
                                   VEHICLE_LINE_FOLLOW_SEARCH_STEER_DEG *
                                   VEHICLE_DEG_TO_RAD);
}

static float line_follow_hard_steer_from_dir(int8_t turn_dir)
{
    float signed_dir = VEHICLE_LINE_FOLLOW_STEER_SIGN *
                       ((turn_dir >= 0) ? 1.0f : -1.0f);

    return (signed_dir >= 0.0f) ? steer_right_limit_rad()
                                : -steer_left_limit_rad();
}

static void line_follow_update_turn_sequence(uint8_t track_bits)
{
    if (((s_line_last_track_bits & LINE_TRACK_2_MASK) != 0U) &&
        ((track_bits & LINE_TRACK_1_MASK) != 0U))
    {
        s_line_hard_turn_dir = -1;
        s_line_last_search_error = (int16_t)-VEHICLE_LINE_FOLLOW_AMBIGUOUS_ERROR;
    }
    else if (((s_line_last_track_bits & LINE_TRACK_4_MASK) != 0U) &&
             ((track_bits & LINE_TRACK_5_MASK) != 0U))
    {
        s_line_hard_turn_dir = 1;
        s_line_last_search_error = (int16_t)VEHICLE_LINE_FOLLOW_AMBIGUOUS_ERROR;
    }

    s_line_last_track_bits = track_bits;
}

static int16_t line_follow_random_error(void)
{
    if (s_line_random_state == 0U)
    {
        s_line_random_state = HAL_GetTick() ^ 0x9E3779B9U;
    }

    s_line_random_state ^= (s_line_random_state << 13);
    s_line_random_state ^= (s_line_random_state >> 17);
    s_line_random_state ^= (s_line_random_state << 5);

    return ((s_line_random_state & 1U) != 0U)
               ? (int16_t)VEHICLE_LINE_FOLLOW_AMBIGUOUS_ERROR
               : (int16_t)-VEHICLE_LINE_FOLLOW_AMBIGUOUS_ERROR;
}

static int16_t line_follow_get_search_error(void)
{
    if (absf_local((float)s_line_last_error) >=
        (float)VEHICLE_LINE_FOLLOW_LOST_MIN_ERROR)
    {
        s_line_last_search_error = s_line_last_error;
        return s_line_last_error;
    }

    if (absf_local((float)s_line_last_search_error) >=
        (float)VEHICLE_LINE_FOLLOW_LOST_MIN_ERROR)
    {
        return s_line_last_search_error;
    }

    s_line_last_search_error = line_follow_random_error();
    return s_line_last_search_error;
}

static int16_t line_follow_resolve_error(uint8_t track_bits, int16_t error)
{
    bool center_on_line = line_follow_center_on_line(track_bits);

    if ((error == 0) && !center_on_line && (track_bits != 0U))
    {
        if (absf_local((float)s_line_last_search_error) >=
            (float)VEHICLE_LINE_FOLLOW_LOST_MIN_ERROR)
        {
            return s_line_last_search_error;
        }

        s_line_last_search_error = line_follow_random_error();
        return s_line_last_search_error;
    }

    return error;
}

static void apply_center_lock(uint8_t track_bits,
                              int16_t error,
                              float *speed,
                              float *steer)
{
    bool center_on_line;

    if ((speed == 0) || (steer == 0))
    {
        return;
    }

    center_on_line = line_follow_center_on_line(track_bits);
    if (center_on_line &&
        (absf_local((float)error) <= (float)VEHICLE_LINE_FOLLOW_CENTER_DEADBAND))
    {
        *steer *= VEHICLE_LINE_FOLLOW_CENTER_STEER_SCALE;
    }
}

static void compute_line_follow_target(const SystemStatePool *pool,
                                       float *speed_out,
                                       float *steer_out)
{
    if ((pool == 0) || (speed_out == 0) || (steer_out == 0))
    {
        return;
    }

    if (pool->sensor.track_valid)
    {
        bool center_on_line = line_follow_center_on_line(pool->sensor.track_bits);
        int16_t error = line_follow_resolve_error(pool->sensor.track_bits,
                                                  pool->sensor.track_error);
        bool had_valid = s_line_has_valid;
        int16_t prev_error = s_line_last_error;

        s_line_last_error = error;
        if (absf_local((float)error) >= (float)VEHICLE_LINE_FOLLOW_LOST_MIN_ERROR)
        {
            s_line_last_search_error = error;
        }
        s_line_has_valid = true;

        if (!center_on_line)
        {
            line_follow_update_turn_sequence(pool->sensor.track_bits);
            if (s_line_hard_turn_dir != 0)
            {
                *speed_out = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
                *steer_out = line_follow_hard_steer_from_dir(s_line_hard_turn_dir);
                return;
            }

            int16_t search_error = line_follow_get_search_error();
            *speed_out = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
            *steer_out = line_follow_search_steer_from_error(search_error);
            return;
        }

        s_line_hard_turn_dir = 0;
        s_line_last_track_bits = pool->sensor.track_bits;
        s_line_last_search_error = 0;

        int16_t error_delta = had_valid ? (int16_t)(error - prev_error) : 0;
        float speed = line_follow_speed_from_error(error);
        float steer = line_follow_steer_from_error(error, error_delta);
        apply_center_lock(pool->sensor.track_bits, error, &speed, &steer);

        *speed_out = speed;
        *steer_out = steer;
        return;
    }

    if (!s_line_has_valid)
    {
        int16_t search_error = line_follow_get_search_error();
        s_line_last_error = search_error;
        *speed_out = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
        *steer_out = line_follow_search_steer_from_error(search_error);
        return;
    }

    if (s_line_hard_turn_dir != 0)
    {
        *speed_out = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
        *steer_out = line_follow_hard_steer_from_dir(s_line_hard_turn_dir);
        return;
    }

    int16_t search_error = line_follow_get_search_error();
    s_line_last_error = search_error;
    *speed_out = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
    *steer_out = line_follow_search_steer_from_error(search_error);
}

static void decision_clear_target_latch(void)
{
    s_state.target_speed_mps = 0.0f;
    s_state.target_steer_rad = 0.0f;
}

static bool decision_auto_start_delay_active(void)
{
    if ((s_state.mode == SYS_MODE_MANUAL) ||
        (s_state.mode == SYS_MODE_STOP) ||
        (s_state.mode == SYS_MODE_ERROR))
    {
        return false;
    }

    return ((uint32_t)(HAL_GetTick() - s_state.mode_enter_ms) <
            VEHICLE_AUTO_START_DELAY_MS);
}

static void decision_enter_mode(SystemStatePool *pool,
                                SystemMode mode,
                                SystemAutoTask auto_task,
                                bool reset_line_state)
{
    s_state.prev_mode = s_state.mode;
    s_state.mode = mode;
    s_state.mode_enter_ms = HAL_GetTick();

    if (pool != 0)
    {
        pool->auto_task = auto_task;
    }

    if (reset_line_state)
    {
        reset_line_follow_state();
        reset_k230_auto_state();
    }
}

static SystemAutoTask decision_auto_task_from_arg(int16_t arg)
{
    if (arg == 1)
    {
        return AUTO_TASK_LINE_FOLLOW;
    }
    if (arg == 2)
    {
        return AUTO_TASK_LINE_FOLLOW_K230;
    }
    if (arg == 3)
    {
        return AUTO_TASK_INSPECTION;
    }

    return AUTO_TASK_NONE;
}

static SystemMode decision_mode_from_auto_task(SystemAutoTask auto_task)
{
    if ((auto_task == AUTO_TASK_LINE_FOLLOW) ||
        (auto_task == AUTO_TASK_LINE_FOLLOW_K230))
    {
        return SYS_MODE_LINE_FOLLOW;
    }
    if (auto_task == AUTO_TASK_INSPECTION)
    {
        return SYS_MODE_INSPECTION;
    }

    return SYS_MODE_STOP;
}

static void decision_target_zero(const SystemStatePool *pool,
                                 float *speed,
                                 float *steer)
{
    (void)pool;
    *speed = 0.0f;
    *steer = 0.0f;
}

static void decision_target_manual(const SystemStatePool *pool,
                                   float *speed,
                                   float *steer)
{
    (void)pool;
    *speed = s_state.target_speed_mps;
    *steer = s_state.target_steer_rad;
}

static float decision_k230_speed_from_value(int16_t value)
{
    float speed;

    if (value <= 0)
    {
        return 0.0f;
    }
    if (value >= 50)
    {
        return VEHICLE_LINE_FOLLOW_SPEED_MPS;
    }

    speed = VEHICLE_LINE_FOLLOW_SPEED_MPS * ((float)value / 50.0f);
    if ((speed > 0.0f) && (speed < VEHICLE_K230_MIN_SPEED_MPS))
    {
        speed = VEHICLE_K230_MIN_SPEED_MPS;
    }
    return speed;
}

static void decision_apply_speed_limit(float *speed)
{
    if (speed == 0)
    {
        return;
    }

    if ((*speed > s_k230_speed_limit_mps) && (s_k230_speed_limit_mps > 0.0f))
    {
        *speed = s_k230_speed_limit_mps;
    }
    else if ((*speed < -s_k230_speed_limit_mps) && (s_k230_speed_limit_mps > 0.0f))
    {
        *speed = -s_k230_speed_limit_mps;
    }
}

static void decision_consume_k230_command(const SystemStatePool *pool)
{
    const K230Result *res;

    if ((pool == 0) || !pool->event.k230_result_ready || !pool->comm.k230_result.valid)
    {
        return;
    }

    res = &pool->comm.k230_result;
    if (res->type == K230_RESULT_CMD_STOP)
    {
        int16_t seconds = (res->value0 < 0) ? 0 : res->value0;
        s_k230_stop_until_ms = res->timestamp_ms + ((uint32_t)seconds * 1000U);
        s_k230_turn_dir = 0;
        s_k230_park_mode = K230_AUTO_PARK_NONE;
        s_k230_turn_seen_off_center = false;
    }
    else if (res->type == K230_RESULT_CMD_SPEED)
    {
        s_k230_speed_limit_mps = decision_k230_speed_from_value(res->value0);
        s_k230_stop_until_ms = 0U;
        s_k230_turn_dir = 0;
        s_k230_park_mode = K230_AUTO_PARK_NONE;
        s_k230_turn_seen_off_center = false;
    }
    else if (res->type == K230_RESULT_CMD_TURN)
    {
        s_k230_turn_dir = (res->value0 < 0) ? -1 : 1;
        s_k230_stop_until_ms = 0U;
        s_k230_park_mode = K230_AUTO_PARK_NONE;
        s_k230_turn_seen_off_center = false;
    }
    else if (res->type == K230_RESULT_CMD_PARK)
    {
        if (res->value0 == 1)
        {
            s_k230_park_mode = K230_AUTO_PARK_REVERSE;
        }
        else if (res->value0 == 2)
        {
            s_k230_park_mode = K230_AUTO_PARK_PARALLEL;
        }
        else
        {
            s_k230_park_mode = K230_AUTO_PARK_NONE;
        }
        s_k230_stop_until_ms = 0U;
        s_k230_turn_dir = 0;
        s_k230_turn_seen_off_center = false;
    }
}

static bool decision_k230_should_stop(uint32_t now_ms)
{
    if (s_k230_stop_until_ms == 0U)
    {
        return false;
    }

    if ((int32_t)(now_ms - s_k230_stop_until_ms) >= 0)
    {
        s_k230_stop_until_ms = 0U;
        return false;
    }

    return true;
}

static bool decision_k230_turn_target(const SystemStatePool *pool,
                                      float *speed,
                                      float *steer)
{
    bool center_on_line;

    if ((pool == 0) || (s_k230_turn_dir == 0))
    {
        return false;
    }

    center_on_line = pool->sensor.track_valid &&
                     line_follow_center_on_line(pool->sensor.track_bits);
    if (!center_on_line)
    {
        s_k230_turn_seen_off_center = true;
    }
    else if (s_k230_turn_seen_off_center)
    {
        s_k230_turn_dir = 0;
        s_k230_turn_seen_off_center = false;
        return false;
    }

    *speed = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
    *steer = line_follow_hard_steer_from_dir(s_k230_turn_dir);
    return true;
}

static void decision_target_line_follow(const SystemStatePool *pool,
                                        float *speed,
                                        float *steer)
{
    if ((pool->auto_task != AUTO_TASK_LINE_FOLLOW) &&
        (pool->auto_task != AUTO_TASK_LINE_FOLLOW_K230))
    {
        *speed = 0.0f;
        *steer = 0.0f;
        return;
    }

    if (pool->auto_task == AUTO_TASK_LINE_FOLLOW_K230)
    {
        decision_consume_k230_command(pool);
        if (decision_k230_should_stop(HAL_GetTick()) ||
            (s_k230_park_mode != K230_AUTO_PARK_NONE))
        {
            *speed = 0.0f;
            *steer = 0.0f;
            return;
        }
        if (decision_k230_turn_target(pool, speed, steer))
        {
            return;
        }
    }

    compute_line_follow_target(pool, speed, steer);

    if (pool->auto_task == AUTO_TASK_LINE_FOLLOW_K230)
    {
        decision_apply_speed_limit(speed);
    }
}

static const DecisionModeHandler s_mode_handlers[] =
{
    { SYS_MODE_STOP,        decision_target_zero },
    { SYS_MODE_ERROR,       decision_target_zero },
    { SYS_MODE_MANUAL,      decision_target_manual },
    { SYS_MODE_LINE_FOLLOW, decision_target_line_follow },
    { SYS_MODE_INSPECTION,  decision_target_zero },
    { SYS_MODE_AVOIDANCE,   decision_target_zero },
};

static void decision_compute_target(const SystemStatePool *pool,
                                    float *speed,
                                    float *steer)
{
    uint32_t i;

    *speed = 0.0f;
    *steer = 0.0f;

    for (i = 0U; i < (uint32_t)(sizeof(s_mode_handlers) / sizeof(s_mode_handlers[0])); ++i)
    {
        if (s_mode_handlers[i].mode == s_state.mode)
        {
            s_mode_handlers[i].target_fn(pool, speed, steer);
            return;
        }
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
    reset_k230_auto_state();
}

void Decision_Task20ms(SystemStatePool *pool)
{
    bool input_event_handled = false;

    if (pool == 0) return;

    /* ── 故障仲裁（最高优先） ── */
    if (pool->fault.emergency_stop || pool->fault.heartbeat_lost)
    {
        SystemMode fault_mode = pool->fault.emergency_stop ? SYS_MODE_ERROR : SYS_MODE_STOP;
        decision_enter_mode(pool, fault_mode, AUTO_TASK_NONE, true);
        decision_clear_target_latch();
        pool->mode = s_state.mode;
        pool->target = (SystemTarget){0};
        return;
    }

    /* ── 按键事件 ── */
    if (pool->event.key_stop_clicked)
    {
        decision_enter_mode(pool, SYS_MODE_STOP, AUTO_TASK_NONE, true);
        decision_clear_target_latch();
        input_event_handled = true;
    }

    else if (pool->event.key_mode_clicked)
    {
        if (s_state.mode == SYS_MODE_LINE_FOLLOW)
        {
            decision_enter_mode(pool, SYS_MODE_MANUAL, AUTO_TASK_NONE, true);
            decision_clear_target_latch();
        }
        else
        {
            decision_enter_mode(pool, SYS_MODE_LINE_FOLLOW, AUTO_TASK_LINE_FOLLOW, true);
        }
        input_event_handled = true;
    }

    else if (pool->event.key_task_clicked)
    {
        if (s_state.mode != SYS_MODE_LINE_FOLLOW)
        {
            decision_enter_mode(pool, SYS_MODE_LINE_FOLLOW, AUTO_TASK_LINE_FOLLOW, true);
        }
        else if (pool->auto_task == AUTO_TASK_LINE_FOLLOW_K230)
        {
            decision_enter_mode(pool, SYS_MODE_LINE_FOLLOW, AUTO_TASK_LINE_FOLLOW, true);
        }
        else
        {
            decision_enter_mode(pool, SYS_MODE_LINE_FOLLOW, AUTO_TASK_LINE_FOLLOW_K230, true);
        }
        input_event_handled = true;
    }

    /* ── 蓝牙命令 ── */
    if (!input_event_handled && pool->event.bt_command_ready)
    {
        BtCommand *cmd = &pool->comm.bt_command;
        if (cmd->type == BT_COMMAND_STOP)
        {
            decision_enter_mode(pool, SYS_MODE_STOP, AUTO_TASK_NONE, true);
            decision_clear_target_latch();
        }
        else if (cmd->type == BT_COMMAND_START_TASK)
        {
            SystemAutoTask auto_task = decision_auto_task_from_arg(cmd->arg0);
            decision_enter_mode(pool,
                                decision_mode_from_auto_task(auto_task),
                                auto_task,
                                true);
        }
        else if (cmd->type == BT_COMMAND_MANUAL_MOVE)
        {
            decision_enter_mode(pool, SYS_MODE_MANUAL, AUTO_TASK_NONE, true);
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

    /* ── 状态机输出：新增运动模式时在 s_mode_handlers 中注册目标计算函数 ── */
    float speed = 0.0f, steer = 0.0f;
    decision_compute_target(pool, &speed, &steer);
    if (decision_auto_start_delay_active())
    {
        speed = 0.0f;
        steer = 0.0f;
    }

    pool->target.speed_mps       = speed;
    pool->target.steer_angle_rad = steer;
    pool->target.valid           = true;
    s_state.target_speed_mps     = speed;
    s_state.target_steer_rad     = steer;
}

DecisionState Decision_GetState(void) { return s_state; }
