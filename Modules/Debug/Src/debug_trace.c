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
 *   CH3  = actual_body_mps   （实际车身速度 m/s，两轮平均）
 *   CH4  = left_target_mps   （左轮目标速度 m/s）
 *   CH5  = left_actual_mps   （左轮实际速度 m/s）
 *   CH6  = left_pwm          （左轮 PWM 千分比）
 *   CH7  = right_target_mps  （右轮目标速度 m/s）
 *   CH8  = right_actual_mps  （右轮实际速度 m/s）
 *   CH9  = right_pwm         （右轮 PWM 千分比）
 *   CH10 = fault_mask        （故障码位图）
 *
 *   fault_mask 位定义（bit=1 表示该故障激活）：
 *     bit0 = heartbeat_lost     通信心跳丢失
 *     bit1 = obstacle_too_close 障碍物过近
 *     bit2 = sensor_invalid     传感器无效
 *     bit3 = motor_stall        电机堵转
 *     bit4 = servo_limit        舵机限幅
 *     bit5 = emergency_stop     紧急停车
 *     bit6 = motion_limited     PWM 达到限幅边界
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

/* ── 故障码位图 ── */

static uint16_t build_fault_mask(const SystemStatePool *pool)
{
    uint16_t mask = 0U;
    if (pool->fault.heartbeat_lost)     mask |= (1U << 0);      /* bit0 */
    if (pool->fault.obstacle_too_close) mask |= (1U << 1);      /* bit1 */
    if (pool->fault.sensor_invalid)     mask |= (1U << 2);      /* bit2 */
    if (pool->fault.motor_stall)        mask |= (1U << 3);      /* bit3 */
    if (pool->fault.servo_limit)        mask |= (1U << 4);      /* bit4 */
    if (pool->fault.emergency_stop)     mask |= (1U << 5);      /* bit5 */
    if (pool->motion.limited)           mask |= (1U << 6);      /* bit6 */
    return mask;
}

/* ── 100ms 输出任务 ── */

void DebugTrace_Task100ms(SystemStatePool *pool)
{
    uint8_t frame[(DEBUG_FLOAT_CHANNELS * 4U) + 4U];     /* 10×4 + 4 帧尾 = 44 字节 */
    uint16_t offset = 0U;

    if ((pool == 0) || !pool->debug.enabled || !BspUart_TxDone(BSP_UART_BT))
        return;

    /* 10 个浮点通道，小端序 */
    put_float_le(&frame[offset], (float)pool->mode);                        offset += 4U;
    put_float_le(&frame[offset], pool->target.speed_mps);                   offset += 4U;
    put_float_le(&frame[offset], pool->estimation.body_speed_mps);          offset += 4U;
    put_float_le(&frame[offset], pool->motion.left_target_mps);             offset += 4U;
    put_float_le(&frame[offset], pool->estimation.left_speed_mps);          offset += 4U;
    put_float_le(&frame[offset], (float)pool->motion.left_pwm);             offset += 4U;
    put_float_le(&frame[offset], pool->motion.right_target_mps);            offset += 4U;
    put_float_le(&frame[offset], pool->estimation.right_speed_mps);         offset += 4U;
    put_float_le(&frame[offset], (float)pool->motion.right_pwm);            offset += 4U;
    put_float_le(&frame[offset], (float)build_fault_mask(pool));            offset += 4U;

    /* 帧尾 */
    frame[offset++] = DEBUG_FRAME_TAIL0;
    frame[offset++] = DEBUG_FRAME_TAIL1;
    frame[offset++] = DEBUG_FRAME_TAIL2;
    frame[offset++] = DEBUG_FRAME_TAIL3;

    (void)BspUart_WriteBuffer(BSP_UART_BT, frame, offset);
}
