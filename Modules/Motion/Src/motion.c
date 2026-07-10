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

static float steer_cmd_left_limit_rad(void)
{
    return VEHICLE_STEER_LEFT_CMD_LIMIT_RAD;
}

static float steer_cmd_right_limit_rad(void)
{
    return VEHICLE_STEER_RIGHT_CMD_LIMIT_RAD;
}

static MotionState s_state;
static bool        s_emergency_brake;

/* ── 初始化 ── */

void Motion_Init(void)
{
    s_state.target_speed_mps  = 0.0f;
    s_state.target_steer_rad  = 0.0f;
    s_state.left_target_mps   = 0.0f;
    s_state.right_target_mps  = 0.0f;
    s_state.left_pwm          = 0;
    s_state.right_pwm         = 0;
    s_state.servo_pulse_us    = VEHICLE_SERVO_CENTER_US;
    s_state.limited           = false;
    s_emergency_brake         = false;

    SpeedPi_Init();

    /* 上电保持停止 */
    BspMotor_StopAll();
    BspServo_SetSteerPermille(0);
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

    bool limited = (L_pwm == PWM_MAX || L_pwm == PWM_MIN ||
                    R_pwm == PWM_MAX || R_pwm == PWM_MIN);

    /* ── 步骤 5：舵机计算 ── */
    float steer_deg = target_steer / VEHICLE_DEG_TO_RAD;
    float permille;
    uint16_t pulse;

    if (VEHICLE_MANUAL_INPUT_MAX_DEG <= 0.0f)
    {
        permille = 0.0f;
    }
    else
    {
        /* 机械角只负责限幅和 Ackermann 计算；舵机输出按蓝牙输入满量程缩放。
         * 例如 INPUT_MAX=45，机械极限=1° 时，满命令只输出 1/45 行程，避免顶死。 */
        permille = (steer_deg / VEHICLE_MANUAL_INPUT_MAX_DEG) * 1000.0f;
    }

    if (permille > 1000.0f) permille = 1000.0f;
    if (permille < -1000.0f) permille = -1000.0f;

    pulse =
        (uint16_t)(VEHICLE_SERVO_CENTER_US +
                   (int32_t)(permille * (float)VEHICLE_SERVO_RANGE_US / 1000.0f));
    if (pulse < VEHICLE_SERVO_MIN_US) pulse = VEHICLE_SERVO_MIN_US;
    if (pulse > VEHICLE_SERVO_MAX_US) pulse = VEHICLE_SERVO_MAX_US;

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

    /* 堵转检测：有目标速度但编码器反馈为 0 持续 500ms */
    static uint8_t stall_cnt;
    if (target_speed != 0.0f && pool->estimation.valid &&
        pool->estimation.left_speed_mps == 0.0f && pool->estimation.right_speed_mps == 0.0f)
    {
        if (++stall_cnt > 50) pool->fault.motor_stall = true;
    }
    else
    {
        stall_cnt = 0;
        pool->fault.motor_stall = false;
    }

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
        s_emergency_brake = true;
        BspMotor_StopAll();
        BspServo_SetSteerPermille(0);
    }
}
