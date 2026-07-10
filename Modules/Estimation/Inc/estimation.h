#ifndef ESTIMATION_H
#define ESTIMATION_H

#include <stdbool.h>
#include <stdint.h>

#include "system_state_pool.h"

/* ────────────────────────────────────────────────────────────
 * Estimation 域 —— 编码器脉冲 → 速度 m/s
 *
 * 职责：读取编码器增量，利用轮径、减速比、编码器线数
 *       将脉冲转换为左右轮速度（m/s）和车身速度。
 *
 * 所有标定参数为内部 static 常量，不暴露到外部。
 *
 * 依赖：BSP/bsp_encoder.h
 * 禁止：调用 Motion、Decision
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    float   left_speed_mps;
    float   right_speed_mps;
    float   body_speed_mps;
    int32_t left_encoder_delta;
    int32_t right_encoder_delta;
    bool    valid;
} EstimationState;

/* ── 生命周期 ── */

void Estimation_Init(void);

/** 10ms 周期调用：读编码器 → 算速度 → 写入 pool->estimation */
void Estimation_Task10ms(SystemStatePool *pool);

EstimationState Estimation_GetState(void);

#endif /* ESTIMATION_H */
