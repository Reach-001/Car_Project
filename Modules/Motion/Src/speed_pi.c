/* ────────────────────────────────────────────────────────────
 * 速度 PI 控制器（Motion 域内部使用）
 *
 * 每轮一个独立 PI 控制器，输入目标速度 m/s + 实际速度 m/s，
 * 输出 PWM 千分比（-1000~1000）。
 *
 * 暂用简单 P 控制器实现（I 项预留），后续可加入积分限幅。
 *
 * PI 参数：标定需在实车上进行。
 *   Kp 太大 → 振荡；Kp 太小 → 响应慢。
 * ──────────────────────────────────────────────────────────── */

#include "motion_internal.h"

#include <stdint.h>

static const float KP            = 200.0f;   /* 比例增益                       */
static const float KI            = 20.0f;    /* 积分增益（预留）                */
static const float I_MAX         = 500.0f;   /* 积分饱和上限                    */
static const float OUTPUT_MAX    = 1000.0f;  /* 输出上限                       */
static const float TARGET_DEADBAND = 0.01f;  /* 小目标直接停车，避免抖动         */
static const float FF_MIN_PWM    = 520.0f;   /* 电机启动/低速基础前馈，需实车标定 */
static const float FF_GAIN_PWM_PER_MPS = 120.0f; /* 速度前馈斜率，需实车标定       */

static float s_integral[2];                  /* 左(0) 右(1) 积分累加器          */

void SpeedPi_Init(void)
{
    s_integral[0] = 0.0f;
    s_integral[1] = 0.0f;
}

int16_t SpeedPi_Compute(int motor_id, float target, float actual)
{
    if ((motor_id < 0) || (motor_id >= 2))
    {
        return 0;
    }

    if ((target > -TARGET_DEADBAND) && (target < TARGET_DEADBAND))
    {
        s_integral[motor_id] = 0.0f;
        return 0;
    }

    float error = target - actual;
    float abs_target = (target < 0.0f) ? -target : target;
    float sign = (target < 0.0f) ? -1.0f : 1.0f;
    float feedforward = sign * (FF_MIN_PWM + (FF_GAIN_PWM_PER_MPS * abs_target));

    /* ── P 项 ── */
    float p_out = KP * error;

    /* ── I 项（预留，当前不启用） ── */
    s_integral[motor_id] += KI * error * 0.01f;    /* dt = 10ms */
    if (s_integral[motor_id] >  I_MAX) s_integral[motor_id] =  I_MAX;
    if (s_integral[motor_id] < -I_MAX) s_integral[motor_id] = -I_MAX;

    float output = feedforward + p_out /* + s_integral[motor_id] */;

    /* 输出限幅 */
    if (output >  OUTPUT_MAX) output =  OUTPUT_MAX;
    if (output < -OUTPUT_MAX) output = -OUTPUT_MAX;

    return (int16_t)output;
}
