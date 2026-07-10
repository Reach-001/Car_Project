/* ────────────────────────────────────────────────────────────
 * 编码器驱动实现
 *
 * 左轮编码器：TIM2（32 位），PA0=CH1, PA1=CH2，ARR=2^32-1，无溢出
 * 右轮编码器：TIM4（16 位），PB6=CH1, PB7=CH2，ARR=65535，需 int16_t 转换
 *
 * 每次 Read 返回累计值和与上次的增量（delta）。
 * Estimation 域用 delta 和 TASK_PERIOD_S 算速度 m/s。
 *
 * TIM4 16-bit 溢出处理：
 *   (int16_t) 把 16-bit 无符号读成有符号 → 再(int32_t)扩展到 32-bit
 *   例：CNT=0xFFFE(-2 in int16) + delta=3 → CNT=0x0001 → int16=1 → int32=1
 *   增量计算 = 1 - (-2) = 3 ✓（自动绕过了 65535→0 的溢出）
 *
 * 注意：此方法要求两次 Read 间隔内编码器脉冲数不超过 32767（半幅），
 * 否则 delta 会错。10ms 采样率下远超安全范围。
 *
 * 参数说明：本文件无硬编码参数。编码器特性（PPR/轮径/减速比）在
 * vehicle_config.h 和 estimation.c 中管理。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_encoder.h"

#include "tim.h"     /* htim2, htim4 */

static int32_t s_last_left;      /* 上次左轮计数值（32 位） */
static int32_t s_last_right;     /* 上次右轮计数值（通过 int16_t 转换后存储） */

void BspEncoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);       /* 左轮 TIM2 32 位编码器 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);       /* 右轮 TIM4 16 位编码器 */
    s_last_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    s_last_right = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

BspEncoderSample BspEncoder_Read(void)
{
    int32_t left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t right = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    BspEncoderSample sample;
    sample.left_count  = left;
    sample.right_count = right;
    sample.left_delta  = left  - s_last_left;
    sample.right_delta = right - s_last_right;

    s_last_left  = left;
    s_last_right = right;
    return sample;
}
