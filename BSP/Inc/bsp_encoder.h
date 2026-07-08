#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * 编码器驱动 —— 硬件抽象层头文件
 *
 * 硬件：左轮 TIM2 编码器模式（PA0=CH1, PA1=CH2）— 32 位计数器
 *       右轮 TIM4 编码器模式（PB6=CH1, PB7=CH2）— 16 位计数器
 *
 * 每次 Read 返回"当前累计值 + 与上次的增量"。
 * 上层由 Estimation 模块将增量转为 m/s 速度。
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    int32_t left_count;      /* 左编码器累计脉冲数   */
    int32_t right_count;     /* 右编码器累计脉冲数   */
    int32_t left_delta;      /* 左编码器本次增量     */
    int32_t right_delta;     /* 右编码器本次增量     */
} BspEncoderSample;

/** 初始化编码器，启动 TIM2/TIM4 的编码器模式，记录初始计数值 */
void BspEncoder_Init(void);

/** 读取编码器当前值并计算增量
 *  @return 包含累计值 + 增量的结构体
 *  调用间隔固定（10ms）时，delta 可用作速度计算的输入 */
BspEncoderSample BspEncoder_Read(void);

#endif /* BSP_ENCODER_H */
