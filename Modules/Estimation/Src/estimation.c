/* ────────────────────────────────────────────────────────────
 * Estimation 域实现
 *
 * 编码器脉冲 → 速度 m/s。
 *
 * 速度公式：
 *   每脉冲行驶距离 = 轮周长 / (编码器线数 × 减速比)
 *     = π × WHEEL_DIAMETER / (PPR × GEAR_RATIO)
 *     = 3.1416 × 0.065 / (13 × 20) = 0.000785 m/pulse
 *   速度 = delta × 米每脉冲 / dt
 *     = delta × 0.000785 / 0.01 = delta × 0.0785 m/s
 *
 * 例：delta=36 → speed ≈ 36 × 0.0785 ≈ 2.83 m/s（偏高，待实车验证）
 *
 * 方向符号：根据实车测试调整 ENCODER_SIGN。
 *   发 MANUAL + 60,0 前进，看 debug CH5/CH8 actual 符号：
 *     actual 为正 → 保持 +1；actual 为负 → 改为 -1
 *
 * 参数说明（全部来自 vehicle_config.h）：
 *   VEHICLE_WHEEL_DIAMETER_M    = 轮径（m），量实车道
 *   VEHICLE_ENCODER_PPR         = 编码器每转脉冲数
 *   VEHICLE_GEAR_RATIO          = 减速比（电机端：轮端）
 *   VEHICLE_LEFT_ENCODER_SIGN   = 左轮方向符号（±1）
 *   VEHICLE_RIGHT_ENCODER_SIGN  = 右轮方向符号（±1）
 *   VEHICLE_LEFT_ENCODER_SPEED_SCALE  = 左轮反馈比例校准
 *   VEHICLE_RIGHT_ENCODER_SPEED_SCALE = 右轮反馈比例校准
 *
 *   任务周期固定 10ms = 0.01s
 * ──────────────────────────────────────────────────────────── */

#include "estimation.h"

#include "bsp_encoder.h"
#include "vehicle_config.h"

/* ════════════════════════════════════════════════════════════
 * 预计算参数
 * ════════════════════════════════════════════════════════════ */

/* Task 周期（s），固定 10ms */
static const float  TASK_PERIOD_S  = 0.01f;

/* 每脉冲行驶距离 = π × D / (PPR × gear_ratio)，单位 m/pulse */
static const float  METERS_PER_PULSE =
    (3.14159265f * VEHICLE_WHEEL_DIAMETER_M) /
    (VEHICLE_ENCODER_PPR * VEHICLE_GEAR_RATIO);

/* ── 内部状态 ── */
static EstimationState s_state;

void Estimation_Init(void)
{
    s_state.left_speed_mps      = 0.0f;
    s_state.right_speed_mps     = 0.0f;
    s_state.body_speed_mps      = 0.0f;
    s_state.left_encoder_delta  = 0;
    s_state.right_encoder_delta = 0;
    s_state.valid               = false;
}

void Estimation_Task10ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    BspEncoderSample enc = BspEncoder_Read();

    /* 带方向符号的增量 */
    int32_t l_delta = enc.left_delta  * VEHICLE_LEFT_ENCODER_SIGN;
    int32_t r_delta = enc.right_delta * VEHICLE_RIGHT_ENCODER_SIGN;

    /* speed = delta × m/pulse / dt */
    s_state.left_speed_mps       = (((float)l_delta * METERS_PER_PULSE) / TASK_PERIOD_S) *
                                   VEHICLE_LEFT_ENCODER_SPEED_SCALE;
    s_state.right_speed_mps      = (((float)r_delta * METERS_PER_PULSE) / TASK_PERIOD_S) *
                                   VEHICLE_RIGHT_ENCODER_SPEED_SCALE;
    s_state.body_speed_mps       = (s_state.left_speed_mps + s_state.right_speed_mps) * 0.5f;
    s_state.left_encoder_delta   = l_delta;
    s_state.right_encoder_delta  = r_delta;
    s_state.valid                = true;

    pool->estimation.left_speed_mps      = s_state.left_speed_mps;
    pool->estimation.right_speed_mps     = s_state.right_speed_mps;
    pool->estimation.body_speed_mps      = s_state.body_speed_mps;
    pool->estimation.left_encoder_delta  = s_state.left_encoder_delta;
    pool->estimation.right_encoder_delta = s_state.right_encoder_delta;
    pool->estimation.valid               = true;
}

EstimationState Estimation_GetState(void)
{
    return s_state;
}
