/* ────────────────────────────────────────────────────────────
 * 速度 PI 控制器（Motion 域内部使用）
 *
 * 每轮一个独立 PI 控制器。输入目标速度 m/s + 实际速度 m/s，
 * 输出 PWM 千分比（-1000~1000）。
 *
 * 输出 = 前馈 + P 项 + I 项（可选）
 *
 * 前馈计算：
 *   feedforward = sign(target) × (FF_MIN_PWM + FF_GAIN × |target|)
 *   其中 FF_MIN_PWM 克服静摩擦，FF_GAIN 提供与速度成正比的 PWM 预估值。
 *
 * P 项：Kp × error（error = target - actual）
 *
 * I 项：Ki × ∫error dt（默认关闭，VEHICLE_SPEED_PI_I_ENABLE=0）
 *
 * 所有参数在 vehicle_config.h 中统一管理。标定顺序见该文件注释。
 * 本文件不加硬编码常数，全部引用 vehicle_config.h 宏。
 *
 * 参数来源（vehicle_config.h）：
 *   VEHICLE_SPEED_PI_KP                   = 比例增益
 *   VEHICLE_SPEED_PI_KI                   = 积分增益
 *   VEHICLE_SPEED_PI_I_ENABLE             = I 项开关（0=关/1=开）
 *   VEHICLE_SPEED_PI_I_MAX                = 积分饱和上限
 *   VEHICLE_SPEED_PI_TARGET_DEADBAND     = 目标速度死区（m/s）
 *   VEHICLE_SPEED_PI_FF_MIN_PWM           = 前馈最小 PWM（克服静摩擦）
 *   VEHICLE_SPEED_PI_FF_GAIN_PWM_PER_MPS = 前馈速度斜率（PWM/m/s）
 * ──────────────────────────────────────────────────────────── */

#include "motion_internal.h"

#include "vehicle_config.h"

#include <stdint.h>

/* 输出硬限幅（千分比上限），仅本文件内部可见 */
static const float OUTPUT_MAX = 1000.0f;

static float s_integral[2];          /* 左(0) 右(1) 积分累加器 */

void SpeedPi_Init(void)
{
    s_integral[0] = 0.0f;
    s_integral[1] = 0.0f;
}

int16_t SpeedPi_Compute(int motor_id, float target, float actual)
{
    if ((motor_id < 0) || (motor_id >= 2)) return 0;

    /* 死区：目标速度极小 → 直接停车，消除零速抖动 */
    if ((target > -VEHICLE_SPEED_PI_TARGET_DEADBAND) &&
        (target <  VEHICLE_SPEED_PI_TARGET_DEADBAND))
    {
        s_integral[motor_id] = 0.0f;
        return 0;
    }

    float error      = target - actual;
    float abs_target = (target < 0.0f) ? -target : target;
    float sign       = (target < 0.0f) ? -1.0f : 1.0f;

    /* ── 前馈 = sign × (minPWM + 斜率 × |target|) ── */
    float feedforward = sign * (VEHICLE_SPEED_PI_FF_MIN_PWM +
                                (VEHICLE_SPEED_PI_FF_GAIN_PWM_PER_MPS * abs_target));

    /* ── P 项 = Kp × error ── */
    float p_out = VEHICLE_SPEED_PI_KP * error;

    /* ── I 项（默认关闭，实车验证 P+前馈稳定后再开） ── */
#if VEHICLE_SPEED_PI_I_ENABLE
    s_integral[motor_id] += VEHICLE_SPEED_PI_KI * error * 0.01f;    /* dt = 10ms */
    if (s_integral[motor_id] >  VEHICLE_SPEED_PI_I_MAX) s_integral[motor_id] =  VEHICLE_SPEED_PI_I_MAX;
    if (s_integral[motor_id] < -VEHICLE_SPEED_PI_I_MAX) s_integral[motor_id] = -VEHICLE_SPEED_PI_I_MAX;
#else
    s_integral[motor_id] = 0.0f;
#endif

    float output = feedforward + p_out + s_integral[motor_id];

    /* 输出限幅 */
    if (output >  OUTPUT_MAX) output =  OUTPUT_MAX;
    if (output < -OUTPUT_MAX) output = -OUTPUT_MAX;

    return (int16_t)output;
}
