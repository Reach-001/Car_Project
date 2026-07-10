/* ────────────────────────────────────────────────────────────
 * 编码器驱动实现
 *
 * 左轮编码器：TIM2（32 位），PA0=CH1, PA1=CH2，ARR=2^32-1
 * 右轮编码器：TIM4（16 位），PB6=CH1, PB7=CH2，ARR=65535
 *
 * 每次 Read 返回累计值和与上次的增量（delta）。
 * Estimation 域用 delta 和 TASK_PERIOD_S 算速度 m/s。
 *
 * 计数器回绕处理：
 *   先用原始无符号 CNT 做差，再把差值解释成有符号增量。
 *   TIM4: delta = (int16_t)(curr_raw - last_raw)
 *   这样 65535→0、32767→32768 都不会产生异常尖峰。
 *
 * 注意：两次 Read 间隔内增量必须小于计数器半幅。
 * TIM4 半幅为 32767 pulse，10ms 采样率下远超安全范围。
 *
 * 参数说明：本文件无硬编码参数。编码器特性（PPR/轮径/减速比）在
 * vehicle_config.h 和 estimation.c 中管理。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_encoder.h"

#include "tim.h"     /* htim2, htim4 */

static uint32_t s_last_left_raw;       /* 上次左轮原始计数值（32 位） */
static uint16_t s_last_right_raw;      /* 上次右轮原始计数值（16 位） */
static int32_t  s_left_count_accum;    /* 左轮连续累计计数 */
static int32_t  s_right_count_accum;   /* 右轮连续累计计数 */

void BspEncoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);       /* 左轮 TIM2 32 位编码器 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);       /* 右轮 TIM4 16 位编码器 */
    s_last_left_raw  = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_last_right_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    s_left_count_accum = 0;
    s_right_count_accum = 0;
}

BspEncoderSample BspEncoder_Read(void)
{
    uint32_t left_raw  = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t right_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    int32_t left_delta  = (int32_t)(left_raw - s_last_left_raw);
    int32_t right_delta = (int32_t)(int16_t)(right_raw - s_last_right_raw);

    s_left_count_accum  += left_delta;
    s_right_count_accum += right_delta;

    BspEncoderSample sample;
    sample.left_count  = s_left_count_accum;
    sample.right_count = s_right_count_accum;
    sample.left_delta  = left_delta;
    sample.right_delta = right_delta;

    s_last_left_raw  = left_raw;
    s_last_right_raw = right_raw;
    return sample;
}
