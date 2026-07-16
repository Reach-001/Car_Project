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

#include "comm.h"
#include "vehicle_config.h"
#include "stm32g4xx_hal.h"     /* HAL_GetTick */

#define LINE_TRACK_1_MASK   (1U << 0)
#define LINE_TRACK_2_MASK   (1U << 1)
#define LINE_TRACK_4_MASK   (1U << 3)
#define LINE_TRACK_5_MASK   (1U << 4)
#define K230_REQUEST_TIMEOUT_MS 1000U

typedef void (*DecisionModeTargetFn)(const SystemStatePool *pool,
                                     float *speed,
                                     float *steer);

typedef struct
{
    SystemMode           mode;
    DecisionModeTargetFn target_fn;
} DecisionModeHandler;

typedef struct
{
    float speed_mps;
    float steer_rad;
    uint32_t duration_ms;
} DecisionParkStep;

typedef struct
{
    DecisionParkAction action;
    uint8_t step;
    uint32_t step_enter_ms;
    bool active;
    bool done;
} DecisionParkState;

typedef struct
{
    K230ResultType type;
    int16_t value0;
    int16_t value1;
    uint32_t timestamp_ms;
    bool valid;
} PendingK230Task;

static DecisionState s_state;
static int16_t       s_line_last_error;
static int16_t       s_line_last_search_error;
static uint32_t      s_line_random_state;
static uint32_t      s_k230_stop_until_ms;
static uint32_t      s_k230_task_cooldown_until_ms;
static uint32_t      s_k230_mark_stable_until_ms;
static float         s_k230_speed_limit_mps;
static float         s_k230_turn_steer_rad;
static uint8_t       s_line_last_track_bits;
static uint8_t       s_k230_task_mark_count;
static int8_t        s_line_hard_turn_dir;
static int8_t        s_k230_turn_dir;
static PendingK230Task s_k230_pending_task;
static DecisionParkState s_park_state;
static bool          s_line_has_valid;
static bool          s_k230_task_mark_blocked;
static bool          s_k230_mark_stabilizing;
static bool          s_k230_mark_stable_ready;
static bool          s_k230_turn_seen_off_center;
static bool          s_k230_request_waiting;
static uint32_t      s_k230_request_until_ms;

static const DecisionParkStep s_reverse_park_steps[] =
{
    /* 倒车入库：先从横杆点直行前探，再向右后方入库。 */
    {  0.0f,  0.0f,                                         300U },
    {  VEHICLE_PARK_SPEED_MPS,  0.0f,                        VEHICLE_REVERSE_PARK_FORWARD_MS },
    { -VEHICLE_PARK_SPEED_MPS,  VEHICLE_PARK_STEER_DEG * VEHICLE_DEG_TO_RAD, 3800U },
    { -VEHICLE_PARK_SPEED_MPS,  0.0f,                                         1000U },
    {  0.0f,  0.0f,                                         300U },
};

static const DecisionParkStep s_parallel_park_steps[] =
{
    /* 侧方停车：先从横杆点直行前探，再向左后方侧方入位。 */
    {  0.0f,  0.0f,                                         300U },
    {  VEHICLE_PARK_SPEED_MPS,  0.0f,                        VEHICLE_PARALLEL_PARK_FORWARD_MS },
    { -VEHICLE_PARK_SPEED_MPS, -VEHICLE_PARK_STEER_DEG * VEHICLE_DEG_TO_RAD, 2300U },
    { -VEHICLE_PARK_SPEED_MPS,  VEHICLE_PARK_STEER_DEG * VEHICLE_DEG_TO_RAD, 1500U },
    {  VEHICLE_PARK_SPEED_MPS, -VEHICLE_PARK_STEER_DEG * VEHICLE_DEG_TO_RAD,  1000U },
    {  VEHICLE_PARK_SPEED_MPS,  0.0f,                                         250U },
    {  0.0f,  0.0f,                                         300U },
};

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

static void k230_clear_pending_task(void)
{
    s_k230_pending_task.type = K230_RESULT_NONE;
    s_k230_pending_task.value0 = 0;
    s_k230_pending_task.value1 = 0;
    s_k230_pending_task.timestamp_ms = 0U;
    s_k230_pending_task.valid = false;
}

static void k230_store_pending_task(const K230Result *res)
{
    if (res == 0)
    {
        return;
    }

    s_k230_pending_task.type = res->type;
    s_k230_pending_task.value0 = res->value0;
    s_k230_pending_task.value1 = res->value1;
    s_k230_pending_task.timestamp_ms = res->timestamp_ms;
    s_k230_pending_task.valid = true;
}

static bool k230_pending_task_expired(uint32_t now_ms)
{
    if (!s_k230_pending_task.valid)
    {
        return false;
    }

    return ((uint32_t)(now_ms - s_k230_pending_task.timestamp_ms) >=
            VEHICLE_K230_PENDING_TIMEOUT_MS);
}

static bool k230_task_mark_detected(const SystemStatePool *pool, uint32_t now_ms)
{
    bool mark_now;

    if ((pool == 0) || !pool->sensor.track_valid)
    {
        s_k230_task_mark_count = 0U;
        return false;
    }

    if ((s_k230_task_cooldown_until_ms != 0U) &&
        ((int32_t)(now_ms - s_k230_task_cooldown_until_ms) < 0))
    {
        s_k230_task_mark_count = 0U;
        return false;
    }

    if ((s_k230_task_cooldown_until_ms != 0U) &&
        ((int32_t)(now_ms - s_k230_task_cooldown_until_ms) >= 0))
    {
        s_k230_task_cooldown_until_ms = 0U;
    }

    mark_now = ((pool->sensor.track_bits & VEHICLE_K230_TASK_MARK_MASK) ==
                VEHICLE_K230_TASK_MARK_MASK);
    if (!mark_now)
    {
        s_k230_task_mark_count = 0U;
        s_k230_task_mark_blocked = false;
        return false;
    }

    if (s_k230_task_mark_blocked)
    {
        s_k230_task_mark_count = 0U;
        return false;
    }

    if (s_k230_task_mark_count < VEHICLE_K230_TASK_MARK_CONFIRM_COUNT)
    {
        ++s_k230_task_mark_count;
    }

    return (s_k230_task_mark_count >= VEHICLE_K230_TASK_MARK_CONFIRM_COUNT);
}

static void k230_start_mark_stabilize(uint32_t now_ms)
{
    s_k230_mark_stabilizing = true;
    s_k230_mark_stable_ready = false;
    s_k230_mark_stable_until_ms = now_ms + VEHICLE_K230_MARK_STABLE_MS;
}

static bool k230_mark_stabilize_active(uint32_t now_ms)
{
    if (!s_k230_mark_stabilizing)
    {
        return false;
    }

    if ((int32_t)(now_ms - s_k230_mark_stable_until_ms) < 0)
    {
        return true;
    }

    s_k230_mark_stabilizing = false;
    s_k230_mark_stable_ready = true;
    s_k230_mark_stable_until_ms = 0U;
    return false;
}

static const DecisionParkStep *decision_park_steps(DecisionParkAction action,
                                                   uint32_t *count)
{
    if (count == 0)
    {
        return 0;
    }

    if (action == DECISION_PARK_REVERSE)
    {
        *count = (uint32_t)(sizeof(s_reverse_park_steps) / sizeof(s_reverse_park_steps[0]));
        return s_reverse_park_steps;
    }
    if (action == DECISION_PARK_PARALLEL)
    {
        *count = (uint32_t)(sizeof(s_parallel_park_steps) / sizeof(s_parallel_park_steps[0]));
        return s_parallel_park_steps;
    }

    *count = 0U;
    return 0;
}

void DecisionPark_Start(DecisionParkAction action, uint32_t now_ms)
{
    uint32_t count;

    if (decision_park_steps(action, &count) == 0)
    {
        DecisionPark_Stop();
        return;
    }

    s_park_state.action = action;
    s_park_state.step = 0U;
    s_park_state.step_enter_ms = now_ms;
    s_park_state.active = true;
    s_park_state.done = false;
}

void DecisionPark_Stop(void)
{
    s_park_state.action = DECISION_PARK_NONE;
    s_park_state.step = 0U;
    s_park_state.step_enter_ms = 0U;
    s_park_state.active = false;
    s_park_state.done = false;
}

bool DecisionPark_IsActive(void)
{
    return s_park_state.active;
}

bool DecisionPark_IsDone(void)
{
    return s_park_state.done;
}

bool DecisionPark_ComputeTarget(uint32_t now_ms, float *speed, float *steer)
{
    const DecisionParkStep *steps;
    uint32_t count;

    if (!s_park_state.active || (speed == 0) || (steer == 0))
    {
        if (s_park_state.done && (speed != 0) && (steer != 0))
        {
            *speed = 0.0f;
            *steer = 0.0f;
            return true;
        }
        return false;
    }

    steps = decision_park_steps(s_park_state.action, &count);
    if ((steps == 0) || (count == 0U))
    {
        DecisionPark_Stop();
        return false;
    }

    if (s_park_state.step >= count)
    {
        s_park_state.step = (uint8_t)(count - 1U);
    }

    if ((uint32_t)(now_ms - s_park_state.step_enter_ms) >=
        steps[s_park_state.step].duration_ms)
    {
        if ((uint32_t)s_park_state.step + 1U < count)
        {
            ++s_park_state.step;
            s_park_state.step_enter_ms = now_ms;
        }
        else
        {
            s_park_state.active = false;
            s_park_state.done = true;
            *speed = 0.0f;
            *steer = 0.0f;
            return true;
        }
    }

    *speed = clamp(steps[s_park_state.step].speed_mps,
                   -VEHICLE_MAX_SPEED_MPS,
                   VEHICLE_MAX_SPEED_MPS);
    *steer = clamp(steps[s_park_state.step].steer_rad,
                   -steer_left_limit_rad(),
                   steer_right_limit_rad());
    return true;
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
    s_k230_task_cooldown_until_ms = 0U;
    s_k230_mark_stable_until_ms = 0U;
    s_k230_speed_limit_mps = VEHICLE_LINE_FOLLOW_SPEED_MPS;
    s_k230_turn_steer_rad = 0.0f;
    s_k230_task_mark_count = 0U;
    s_k230_task_mark_blocked = false;
    s_k230_mark_stabilizing = false;
    s_k230_mark_stable_ready = false;
    s_k230_turn_dir = 0;
    s_k230_request_waiting = false;
    s_k230_request_until_ms = 0U;
    k230_clear_pending_task();
    DecisionPark_Stop();
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

static bool decision_k230_should_stop(uint32_t now_ms);

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

static float decision_k230_turn_steer_from_deg(int8_t turn_dir, int16_t angle_deg)
{
    float abs_deg = (angle_deg < 0) ? (float)-angle_deg : (float)angle_deg;
    float steer = abs_deg * VEHICLE_DEG_TO_RAD;

    if (turn_dir < 0)
    {
        return -clamp(steer, 0.0f, steer_left_limit_rad());
    }

    return clamp(steer, 0.0f, steer_right_limit_rad());
}

static void decision_consume_k230_command(const SystemStatePool *pool)
{
    const K230Result *res;
    uint32_t now_ms;

    if ((pool == 0) || !pool->event.k230_result_ready || !pool->comm.k230_result.valid)
    {
        return;
    }

    res = &pool->comm.k230_result;
    now_ms = HAL_GetTick();

    /* S:n 是时间锁存停车；停车窗口内忽略 V/L/R/P，避免 K230 连续输出 V:50 立即取消停车。 */
    if ((res->type != K230_RESULT_CMD_STOP) && decision_k230_should_stop(now_ms))
    {
        return;
    }

    if (res->type == K230_RESULT_CMD_STOP)
    {
        s_k230_request_waiting = false;
        k230_store_pending_task(res);
    }
    else if (res->type == K230_RESULT_CMD_SPEED)
    {
        s_k230_request_waiting = false;
        k230_store_pending_task(res);
    }
    else if (res->type == K230_RESULT_CMD_TURN)
    {
        s_k230_request_waiting = false;
        k230_store_pending_task(res);
    }
    else if (res->type == K230_RESULT_CMD_PARK)
    {
        s_k230_request_waiting = false;
        k230_store_pending_task(res);
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
        s_k230_turn_steer_rad = 0.0f;
        s_k230_turn_seen_off_center = false;
        return false;
    }

    *speed = VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS;
    *steer = s_k230_turn_steer_rad;
    return true;
}

static bool decision_k230_trigger_pending_task(const SystemStatePool *pool,
                                               uint32_t now_ms)
{
    K230ResultType type;
    int16_t value0;
    int16_t value1;
    bool mark_now;
    bool request_window_active;

    if (k230_mark_stabilize_active(now_ms))
    {
        return true;
    }

    if (!s_k230_pending_task.valid)
    {
        mark_now = k230_task_mark_detected(pool, now_ms);
        if (!mark_now)
        {
            if (s_k230_request_waiting &&
                ((int32_t)(now_ms - s_k230_request_until_ms) >= 0))
            {
                s_k230_request_waiting = false;
                s_k230_request_until_ms = 0U;
            }
            return false;
        }

        if (!s_k230_request_waiting ||
            ((int32_t)(now_ms - s_k230_request_until_ms) >= 0))
        {
            if (Comm_RequestK230Detect())
            {
                s_k230_request_waiting = true;
                s_k230_request_until_ms = now_ms + K230_REQUEST_TIMEOUT_MS;
            }
        }

        k230_start_mark_stabilize(now_ms);
        return true;
    }

    if (k230_pending_task_expired(now_ms))
    {
        k230_clear_pending_task();
        s_k230_request_waiting = false;
        s_k230_request_until_ms = 0U;
        s_k230_mark_stabilizing = false;
        s_k230_mark_stable_ready = false;
        s_k230_mark_stable_until_ms = 0U;
        return false;
    }

    mark_now = k230_task_mark_detected(pool, now_ms);
    request_window_active = s_k230_request_waiting &&
                            ((int32_t)(now_ms - s_k230_request_until_ms) < 0);
    if (mark_now && !s_k230_mark_stable_ready)
    {
        k230_start_mark_stabilize(now_ms);
        return true;
    }

    if (!mark_now && !request_window_active && !s_k230_mark_stable_ready)
    {
        return false;
    }

    s_k230_mark_stable_ready = false;

    type = s_k230_pending_task.type;
    value0 = s_k230_pending_task.value0;
    value1 = s_k230_pending_task.value1;
    k230_clear_pending_task();
    s_k230_request_waiting = false;
    s_k230_request_until_ms = 0U;
    s_k230_task_mark_count = 0U;
    s_k230_task_mark_blocked = true;
    s_k230_task_cooldown_until_ms = now_ms + VEHICLE_K230_TASK_COOLDOWN_MS;

    s_k230_stop_until_ms = 0U;
    s_k230_turn_dir = 0;
    s_k230_turn_steer_rad = 0.0f;
    s_k230_turn_seen_off_center = false;

    if (type == K230_RESULT_CMD_STOP)
    {
        int16_t seconds = (value0 < 0) ? 0 : value0;
        s_k230_stop_until_ms = now_ms + ((uint32_t)seconds * 1000U);
    }
    else if (type == K230_RESULT_CMD_SPEED)
    {
        s_k230_speed_limit_mps = decision_k230_speed_from_value(value0);
    }
    else if (type == K230_RESULT_CMD_TURN)
    {
        s_k230_turn_dir = (value0 < 0) ? -1 : 1;
        s_k230_turn_steer_rad = decision_k230_turn_steer_from_deg(s_k230_turn_dir,
                                                                  value1);
    }
    else if (type == K230_RESULT_CMD_PARK)
    {
        if (value0 == 1)
        {
            DecisionPark_Start(DECISION_PARK_REVERSE, now_ms);
        }
        else if (value0 == 2)
        {
            DecisionPark_Start(DECISION_PARK_PARALLEL, now_ms);
        }
    }

    return false;
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
        if (!DecisionPark_IsActive() &&
            decision_k230_trigger_pending_task(pool, HAL_GetTick()))
        {
            *speed = 0.0f;
            *steer = 0.0f;
            return;
        }
        if (!DecisionPark_IsActive() &&
            decision_k230_should_stop(HAL_GetTick()))
        {
            *speed = 0.0f;
            *steer = 0.0f;
            return;
        }
        if (DecisionPark_ComputeTarget(HAL_GetTick(), speed, steer))
        {
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
