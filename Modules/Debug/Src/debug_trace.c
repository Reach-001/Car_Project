/* ────────────────────────────────────────────────────────────
 * Debug 调试输出（Debug 域）
 *
 * 100ms 周期输出 10 通道小端浮点数组帧，通过蓝牙 UART 发出。
 * 默认关闭，通过 DEBUG,1 / DEBUG,0 命令开关。
 *
 * 帧格式：
 *   CH1~CH10: float32 little-endian × 10 通道 = 40 字节
 *   帧尾: 0x00 0x00 0x80 0x7F = 4 字节（用于串口绘图工具帧同步）
 *   总帧长: 44 字节
 *
 * 通道定义（按顺序）：
 *   CH1  = mode              （系统模式 0~5）
 *   CH2  = target_body_mps   （目标车身速度 m/s）
 *   CH3  = left_encoder_delta （左编码器 10ms 原始增量）
 *   CH4  = left_target_mps   （左轮目标速度 m/s）
 *   CH5  = left_actual_mps   （左轮编码器速度 m/s）
 *   CH6  = left_pwm_norm     （左轮 PWM，-1~1）
 *   CH7  = right_target_mps  （右轮目标速度 m/s）
 *   CH8  = right_actual_mps  （右轮编码器速度 m/s）
 *   CH9  = right_pwm_norm    （右轮 PWM，-1~1）
 *   CH10 = right_encoder_delta（右编码器 10ms 原始增量）
 *
 * 帧尾说明：
 *   0x0000807F 在 little-endian float 中 = +∞/NaN 区间的值。
 *   串口绘图工具收到后可作为帧同步标志。
 *
 * 参数说明：
 *   DEBUG_FLOAT_CHANNELS = 每帧浮点通道数。当前 10 通道 = 10×4+4=44 字节/帧
 *   DEBUG_FRAME_TAILx    = 帧尾四字节标识。改协议时改这里
 * ──────────────────────────────────────────────────────────── */

#include "debug_trace.h"

#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

/* ════════════════════════════════════════════════════════════
 * 帧格式参数
 * ════════════════════════════════════════════════════════════ */

#define DEBUG_FLOAT_CHANNELS 10U                     /* 浮点通道数                        */
#define DEBUG_FRAME_TAIL0    0x00U                   /* 帧尾字节 0                        */
#define DEBUG_FRAME_TAIL1    0x00U                   /* 帧尾字节 1                        */
#define DEBUG_FRAME_TAIL2    0x80U                   /* 帧尾字节 2                        */
#define DEBUG_FRAME_TAIL3    0x7FU                   /* 帧尾字节 3（0x00 00 80 7F）      */

/* ── 初始化 ── */

void DebugTrace_Init(void) {}
void DebugTrace_Task20ms(SystemStatePool *pool) { (void)pool; }

/* ── 小端 float 写入 ── */

static void put_float_le(uint8_t *dst, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    dst[0] = (uint8_t)(raw & 0xFFU);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFU);
    dst[2] = (uint8_t)((raw >> 16) & 0xFFU);
    dst[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

/* ── 100ms 输出任务 ── */

void DebugTrace_Task100ms(SystemStatePool *pool)
{
    uint8_t frame[(DEBUG_FLOAT_CHANNELS * 4U) + 4U];     /* 10×4 + 4 帧尾 = 44 字节 */
    uint16_t offset = 0U;

    if ((pool == 0) || !pool->debug.enabled || !BspUart_TxDone(BSP_UART_BT))
        return;

    /* 10 个浮点通道，小端序。CH3/CH10 输出原始 delta，用于定位编码器计数链路。 */
    put_float_le(&frame[offset], (float)pool->mode);                        offset += 4U;
    put_float_le(&frame[offset], pool->target.speed_mps);                   offset += 4U;
    put_float_le(&frame[offset], (float)pool->estimation.left_encoder_delta); offset += 4U;
    put_float_le(&frame[offset], pool->motion.left_target_mps);             offset += 4U;
    put_float_le(&frame[offset], pool->estimation.left_speed_mps);          offset += 4U;
    put_float_le(&frame[offset], (float)pool->motion.left_pwm / 1000.0f);    offset += 4U;
    put_float_le(&frame[offset], pool->motion.right_target_mps);            offset += 4U;
    put_float_le(&frame[offset], pool->estimation.right_speed_mps);         offset += 4U;
    put_float_le(&frame[offset], (float)pool->motion.right_pwm / 1000.0f);   offset += 4U;
    put_float_le(&frame[offset], (float)pool->estimation.right_encoder_delta); offset += 4U;

    /* 帧尾 */
    frame[offset++] = DEBUG_FRAME_TAIL0;
    frame[offset++] = DEBUG_FRAME_TAIL1;
    frame[offset++] = DEBUG_FRAME_TAIL2;
    frame[offset++] = DEBUG_FRAME_TAIL3;

    (void)BspUart_WriteBuffer(BSP_UART_BT, frame, offset);
}
