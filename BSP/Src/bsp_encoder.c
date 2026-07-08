#include "bsp_encoder.h"

#include "tim.h"     /* htim2, htim4 */

/* ────────────────────────────────────────────────────────────
 * 编码器驱动实现
 *
 * TIM2：32 位编码器，ARR=2^32-1，无溢出风险
 * TIM4：16 位编码器，ARR=65535，需 int16_t 转换处理溢出回绕
 *
 * delta 供上层 Estimation 模块计算车速 m/s：
 *   speed = (delta / pulses_per_rev / gear_ratio) × wheel_circumference / dt
 * ──────────────────────────────────────────────────────────── */

static int32_t s_last_left;      /* 上次左轮计数值 */
static int32_t s_last_right;     /* 上次右轮计数值 */

/* ── 初始化 ── */

void BspEncoder_Init(void)
{
    /* 启动编码器模式 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);    /* 左轮 32 位 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);    /* 右轮 16 位 */

    /* 记录初始值，避免首次 Read 出现虚假 delta */
    s_last_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);           /* 32 位直接读 */
    s_last_right = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);  /* 16 位转有符号再扩展到 32 位 */
}

/* ── 读取接口 ── */

BspEncoderSample BspEncoder_Read(void)
{
    /* 读当前计数值（TIM4 用 int16_t 处理溢出回绕） */
    int32_t left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t right = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    BspEncoderSample sample;
    sample.left_count  = left;
    sample.right_count = right;
    sample.left_delta  = left  - s_last_left;    /* 增量 = 当前 - 上次 */
    sample.right_delta = right - s_last_right;

    /* 更新上次值 */
    s_last_left  = left;
    s_last_right = right;

    return sample;
}
