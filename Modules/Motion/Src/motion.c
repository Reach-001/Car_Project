#include "motion.h"
#include "motion_internal.h"

#include "bsp_motor.h"
#include "bsp_servo.h"
#include "vehicle_config.h"

/* ────────────────────────────────────────────────────────────
 * Motion 域聚合入口
 *
 * 子模块：ackermann.c（阿克曼几何分配）
 *        speed_pi.c（速度 PI 控制器）
 * ──────────────────────────────────────────────────────────── */

/* ── 安全限幅参数 ── */

#define SPEED_MAX_MPS     VEHICLE_MAX_SPEED_MPS
#define PWM_MAX           1000      /* PWM 千分比上限     */
#define PWM_MIN           -1000     /* PWM 千分比下限     */
#define START_BOOST_CYCLES ((uint16_t)((VEHICLE_MOTOR_START_BOOST_MS + 9U) / 10U))

static float steer_cmd_left_limit_rad(void)
{
    return VEHICLE_STEER_LEFT_CMD_LIMIT_RAD;
}

static float steer_cmd_right_limit_rad(void)
{
    return VEHICLE_STEER_RIGHT_CMD_LIMIT_RAD;
}

static float absf_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool limit_wheel_targets(float *left_mps, float *right_mps)
{
    bool limited = false;

    if ((left_mps == 0) || (right_mps == 0))
    {
        return false;
    }

    if (*left_mps > SPEED_MAX_MPS)
    {
        *left_mps = SPEED_MAX_MPS;
        limited = true;
    }
    else if (*left_mps < -SPEED_MAX_MPS)
    {
        *left_mps = -SPEED_MAX_MPS;
        limited = true;
    }

    if (*right_mps > SPEED_MAX_MPS)
    {
        *right_mps = SPEED_MAX_MPS;
        limited = true;
    }
    else if (*right_mps < -SPEED_MAX_MPS)
    {
        *right_mps = -SPEED_MAX_MPS;
        limited = true;
    }

    return limited;
}

static float steer_to_servo_permille(float target_steer_rad)
{
    float target_deg;
    float servo_deg;
    float permille;

    if (VEHICLE_MANUAL_INPUT_MAX_DEG <= 0.0f)
    {
        return 0.0f;
    }

    target_deg = target_steer_rad / VEHICLE_DEG_TO_RAD;
    /* 目标转角是相对轮子居中的角度；舵机输出需要叠加中心修正量。 */
    servo_deg = VEHICLE_STEER_CENTER_DEG + target_deg;
    permille = (servo_deg / VEHICLE_MANUAL_INPUT_MAX_DEG) * 1000.0f;

    if (permille > 1000.0f) permille = 1000.0f;
    if (permille < -1000.0f) permille = -1000.0f;
    return permille;
}

static uint16_t servo_pulse_from_permille(float permille)
{
    uint16_t pulse =
        (uint16_t)(VEHICLE_SERVO_CENTER_US +
                   (int32_t)(permille * (float)VEHICLE_SERVO_RANGE_US / 1000.0f));

    if (pulse < VEHICLE_SERVO_MIN_US) pulse = VEHICLE_SERVO_MIN_US;
    if (pulse > VEHICLE_SERVO_MAX_US) pulse = VEHICLE_SERVO_MAX_US;
    return pulse;
}

static int16_t apply_pwm_slew(int16_t requested, int16_t previous)
{
    int16_t max_step = (int16_t)VEHICLE_MOTOR_PWM_SLEW_PER_10MS;
    int16_t delta;

    if (max_step <= 0)
    {
        return requested;
    }

    delta = (int16_t)(requested - previous);
    if (delta > max_step)  return (int16_t)(previous + max_step);
    if (delta < -max_step) return (int16_t)(previous - max_step);
    return requested;
}

static int16_t apply_start_boost(float target_mps, float actual_mps, int16_t requested_pwm)
{
    int16_t boost = (int16_t)VEHICLE_MOTOR_START_BOOST_PWM;
    int16_t min_pwm;

    if (boost <= 0)
    {
        return requested_pwm;
    }

    if ((target_mps > -VEHICLE_SPEED_PI_TARGET_DEADBAND) &&
        (target_mps <  VEHICLE_SPEED_PI_TARGET_DEADBAND))
    {
        return requested_pwm;
    }

    if (absf_local(actual_mps) > VEHICLE_MOTOR_START_ACTUAL_MAX_MPS)
    {
        return requested_pwm;
    }

    min_pwm = (target_mps >= 0.0f) ? boost : (int16_t)-boost;
    if ((target_mps >= 0.0f) && (requested_pwm < min_pwm)) return min_pwm;
    if ((target_mps <  0.0f) && (requested_pwm > min_pwm)) return min_pwm;
    return requested_pwm;
}

static bool target_active(float target_mps)
{
    return !((target_mps > -VEHICLE_SPEED_PI_TARGET_DEADBAND) &&
             (target_mps <  VEHICLE_SPEED_PI_TARGET_DEADBAND));
}

static bool target_sign_changed(float previous, float current)
{
    return ((previous > 0.0f) && (current < 0.0f)) ||
           ((previous < 0.0f) && (current > 0.0f));
}

static bool update_start_boost(float target_mps,
                               float actual_mps,
                               float *previous_target,
                               uint16_t *counter)
{
    bool active;

    if ((previous_target == 0) || (counter == 0))
    {
        return false;
    }

    active = target_active(target_mps);
    if (!active)
    {
        *counter = 0U;
        *previous_target = target_mps;
        return false;
    }

    if (!target_active(*previous_target) ||
        target_sign_changed(*previous_target, target_mps))
    {
        *counter = START_BOOST_CYCLES;
    }

    if (absf_local(actual_mps) > VEHICLE_MOTOR_START_ACTUAL_MAX_MPS)
    {
        *counter = 0U;
    }

    *previous_target = target_mps;

    if (*counter == 0U)
    {
        return false;
    }

    --(*counter);
    return true;
}

static MotionState s_state;
static bool        s_emergency_brake;
static int16_t     s_left_pwm_slew;
static int16_t     s_right_pwm_slew;
static float       s_left_prev_target;
static float       s_right_prev_target;
static uint16_t    s_left_start_boost_count;
static uint16_t    s_right_start_boost_count;

/* ── 初始化 ── */

void Motion_Init(void)
{
    float center_permille = steer_to_servo_permille(0.0f);

    s_state.target_speed_mps  = 0.0f;
    s_state.target_steer_rad  = 0.0f;
    s_state.left_target_mps   = 0.0f;
    s_state.right_target_mps  = 0.0f;
    s_state.left_pwm          = 0;
    s_state.right_pwm         = 0;
    s_state.servo_pulse_us    = servo_pulse_from_permille(center_permille);
    s_state.limited           = false;
    s_emergency_brake         = false;
    s_left_pwm_slew           = 0;
    s_right_pwm_slew          = 0;
    s_left_prev_target        = 0.0f;
    s_right_prev_target       = 0.0f;
    s_left_start_boost_count  = 0U;
    s_right_start_boost_count = 0U;

    SpeedPi_Init();

    /* 上电保持停止 */
    BspMotor_StopAll();
    BspServo_SetSteerPermille((int16_t)center_permille);
}

/* ── 10ms 周期 ── */

void Motion_Task10ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    float target_speed, target_steer;

    /* 紧急刹车 → 绕过 Decision，直接停车 */
    if (s_emergency_brake)
    {
        target_speed = 0.0f;
        target_steer = 0.0f;
        pool->fault.emergency_stop = true;
    }
    else if (pool->target.valid)
    {
        target_speed = pool->target.speed_mps;
        target_steer = pool->target.steer_angle_rad;
    }
    else
    {
        target_speed = 0.0f;
        target_steer = 0.0f;
    }

    /* ── 步骤 1：目标限幅 ── */
    if      (target_speed >  SPEED_MAX_MPS)  target_speed =  SPEED_MAX_MPS;
    else if (target_speed < -SPEED_MAX_MPS)  target_speed = -SPEED_MAX_MPS;

    if      (target_steer >  steer_cmd_right_limit_rad())  target_steer =  steer_cmd_right_limit_rad();
    else if (target_steer < -steer_cmd_left_limit_rad())   target_steer = -steer_cmd_left_limit_rad();

    /* ── 步骤 2：阿克曼左右轮分配 ── */
    float L_target, R_target;
    Ackermann_Compute(target_speed, target_steer, &L_target, &R_target);
    bool wheel_speed_limited = limit_wheel_targets(&L_target, &R_target);

    /* ── 步骤 3：左右轮 PI 闭环 ── */
    float L_actual = pool->estimation.valid ? pool->estimation.left_speed_mps  : 0.0f;
    float R_actual = pool->estimation.valid ? pool->estimation.right_speed_mps : 0.0f;

    int16_t L_pwm = SpeedPi_Compute(0, L_target, L_actual);
    int16_t R_pwm = SpeedPi_Compute(1, R_target, R_actual);

    /* ── 步骤 4：PWM 输出限幅 ── */
    if (L_pwm > PWM_MAX) L_pwm = PWM_MAX;
    if (L_pwm < PWM_MIN) L_pwm = PWM_MIN;
    if (R_pwm > PWM_MAX) R_pwm = PWM_MAX;
    if (R_pwm < PWM_MIN) R_pwm = PWM_MIN;

    bool L_start_boost = update_start_boost(L_target,
                                            L_actual,
                                            &s_left_prev_target,
                                            &s_left_start_boost_count);
    bool R_start_boost = update_start_boost(R_target,
                                            R_actual,
                                            &s_right_prev_target,
                                            &s_right_start_boost_count);

    /* 非起步阶段走斜率限制；起步助推绕过斜率限制，避免低速单帧命令迟迟越不过静摩擦。 */
    if (!target_active(L_target))
    {
        s_left_pwm_slew = 0;
        s_left_start_boost_count = 0U;
        L_pwm = 0;
    }
    else if (L_start_boost)
    {
        L_pwm = apply_start_boost(L_target, L_actual, L_pwm);
        if (L_pwm > PWM_MAX) L_pwm = PWM_MAX;
        if (L_pwm < PWM_MIN) L_pwm = PWM_MIN;
        s_left_pwm_slew = L_pwm;
    }
    else
    {
        L_pwm = apply_pwm_slew(L_pwm, s_left_pwm_slew);
        s_left_pwm_slew = L_pwm;
    }

    if (!target_active(R_target))
    {
        s_right_pwm_slew = 0;
        s_right_start_boost_count = 0U;
        R_pwm = 0;
    }
    else if (R_start_boost)
    {
        R_pwm = apply_start_boost(R_target, R_actual, R_pwm);
        if (R_pwm > PWM_MAX) R_pwm = PWM_MAX;
        if (R_pwm < PWM_MIN) R_pwm = PWM_MIN;
        s_right_pwm_slew = R_pwm;
    }
    else
    {
        R_pwm = apply_pwm_slew(R_pwm, s_right_pwm_slew);
        s_right_pwm_slew = R_pwm;
    }

    bool limited = wheel_speed_limited ||
                   (L_pwm == PWM_MAX || L_pwm == PWM_MIN ||
                    R_pwm == PWM_MAX || R_pwm == PWM_MIN);

    /* ── 步骤 5：舵机计算 ── */
    float permille = steer_to_servo_permille(target_steer);
    uint16_t pulse = servo_pulse_from_permille(permille);

    /* ── 步骤 6：写 BSP Actuator ── */
    BspMotor_SetDuty(BSP_MOTOR_LEFT,  L_pwm);
    BspMotor_SetDuty(BSP_MOTOR_RIGHT, R_pwm);
    BspServo_SetSteerPermille((int16_t)permille);

    /* ── 更新状态 ── */
    s_state.target_speed_mps  = target_speed;
    s_state.target_steer_rad  = target_steer;
    s_state.left_target_mps   = L_target;
    s_state.right_target_mps  = R_target;
    s_state.left_pwm          = L_pwm;
    s_state.right_pwm         = R_pwm;
    s_state.servo_pulse_us    = pulse;
    s_state.limited           = limited;

    pool->motion.target_speed_mps = s_state.target_speed_mps;
    pool->motion.target_steer_rad = s_state.target_steer_rad;
    pool->motion.left_target_mps  = s_state.left_target_mps;
    pool->motion.right_target_mps = s_state.right_target_mps;
    pool->motion.left_pwm         = s_state.left_pwm;
    pool->motion.right_pwm        = s_state.right_pwm;
    pool->motion.servo_pulse_us   = s_state.servo_pulse_us;
    pool->motion.limited          = s_state.limited;

    pool->fault.servo_limit = limited;
}

/* ── 状态查询 ── */

MotionState Motion_GetState(void)
{
    return s_state;
}

void Motion_SetCommand(const MotionCommand *cmd)
{
    if (cmd == 0) return;
    if (cmd->emergency_brake)
    {
        float center_permille = steer_to_servo_permille(0.0f);
        s_emergency_brake = true;
        s_left_pwm_slew = 0;
        s_right_pwm_slew = 0;
        s_left_start_boost_count = 0U;
        s_right_start_boost_count = 0U;
        s_left_prev_target = 0.0f;
        s_right_prev_target = 0.0f;
        BspMotor_StopAll();
        BspServo_SetSteerPermille((int16_t)center_permille);
    }
}
