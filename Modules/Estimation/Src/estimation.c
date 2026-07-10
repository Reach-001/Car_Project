#include "estimation.h"

#include "bsp_encoder.h"

/* ────────────────────────────────────────────────────────────
 * Estimation 域实现
 *
 * 速度公式：speed = delta / PPR / gear_ratio × wheel_circumference / dt
 *   PPR（编码器线数）   = 13 pulses/rev
 *   减速比               = 30:1
 *   轮径                 = 0.065m
 *   周長                 = 0.065 × π ≈ 0.204m
 *   dt                   = 0.01s (10ms Task 周期)
 *
 * 方向因子：±1，用于修正编码器正负方向。
 * 实车标定时根据"正转=前进"修正 sign。
 * ──────────────────────────────────────────────────────────── */

/* ── 标定参数（全部 static const） ── */

static const float  WHEEL_DIAMETER_M        = 0.065f;   /* 轮径 m          */
static const float  ENCODER_PPR             = 13.0f;    /* 编码器线数      */
static const float  GEAR_RATIO              = 30.0f;    /* 减速比          */
static const float  TASK_PERIOD_S           = 0.01f;    /* 10ms 周期       */
static const int8_t LEFT_DIR_SIGN           =  1;       /* 左轮方向修正    */
static const int8_t RIGHT_DIR_SIGN          = -1;       /* 右轮前进原始 delta 为负 */

/* 预计算常数：每脉冲对应行驶距离（m/pulse） */
static const float  METERS_PER_PULSE =
    (3.14159265f * WHEEL_DIAMETER_M) / (ENCODER_PPR * GEAR_RATIO);

/* ── 内部状态 ── */

static EstimationState s_state;

/* ── 初始化 ── */

void Estimation_Init(void)
{
    s_state.left_speed_mps      = 0.0f;
    s_state.right_speed_mps     = 0.0f;
    s_state.body_speed_mps      = 0.0f;
    s_state.left_encoder_delta  = 0;
    s_state.right_encoder_delta = 0;
    s_state.valid               = false;
}

/* ── 10ms 周期 ── */

void Estimation_Task10ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    BspEncoderSample enc = BspEncoder_Read();

    /* 带方向因子的增量 */
    int32_t l_delta = enc.left_delta  * LEFT_DIR_SIGN;
    int32_t r_delta = enc.right_delta * RIGHT_DIR_SIGN;

    /* 速度 = (delta × 米每脉冲) / 周期 */
    s_state.left_speed_mps  = ((float)l_delta * METERS_PER_PULSE) / TASK_PERIOD_S;
    s_state.right_speed_mps = ((float)r_delta * METERS_PER_PULSE) / TASK_PERIOD_S;
    s_state.body_speed_mps  = (s_state.left_speed_mps + s_state.right_speed_mps) * 0.5f;
    s_state.left_encoder_delta  = l_delta;
    s_state.right_encoder_delta = r_delta;
    s_state.valid = true;

    /* 写入状态池 */
    pool->estimation.left_speed_mps      = s_state.left_speed_mps;
    pool->estimation.right_speed_mps     = s_state.right_speed_mps;
    pool->estimation.body_speed_mps      = s_state.body_speed_mps;
    pool->estimation.left_encoder_delta  = s_state.left_encoder_delta;
    pool->estimation.right_encoder_delta = s_state.right_encoder_delta;
    pool->estimation.valid               = true;
}

/* ── 状态查询 ── */

EstimationState Estimation_GetState(void)
{
    return s_state;
}
