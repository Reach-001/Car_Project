#include "debug_trace.h"

#include "bsp_uart.h"

#include <stdint.h>
#include <string.h>

#define DEBUG_FLOAT_CHANNELS 10U
#define DEBUG_FRAME_TAIL0    0x00U
#define DEBUG_FRAME_TAIL1    0x00U
#define DEBUG_FRAME_TAIL2    0x80U
#define DEBUG_FRAME_TAIL3    0x7FU

void DebugTrace_Init(void)
{
}

static void put_float_le(uint8_t *dst, float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    dst[0] = (uint8_t)(raw & 0xFFU);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFU);
    dst[2] = (uint8_t)((raw >> 16) & 0xFFU);
    dst[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

static uint16_t build_fault_mask(const SystemStatePool *pool)
{
    uint16_t mask = 0U;

    if (pool->fault.heartbeat_lost)     mask |= (1U << 0);
    if (pool->fault.obstacle_too_close) mask |= (1U << 1);
    if (pool->fault.sensor_invalid)     mask |= (1U << 2);
    if (pool->fault.motor_stall)        mask |= (1U << 3);
    if (pool->fault.servo_limit)        mask |= (1U << 4);
    if (pool->fault.emergency_stop)     mask |= (1U << 5);
    if (pool->motion.limited)           mask |= (1U << 6);

    return mask;
}

void DebugTrace_Task100ms(SystemStatePool *pool)
{
    uint8_t frame[(DEBUG_FLOAT_CHANNELS * 4U) + 4U];
    uint16_t offset = 0U;

    if ((pool == 0) || !BspUart_TxDone(BSP_UART_BT))
    {
        return;
    }

    /* 小端浮点数组协议：
     * CH1 mode, CH2 target_body_mps, CH3 actual_body_mps,
     * CH4 left_target_mps, CH5 left_actual_mps, CH6 left_pwm,
     * CH7 right_target_mps, CH8 right_actual_mps, CH9 right_pwm,
     * CH10 fault_mask, tail = 00 00 80 7F.
     */
    put_float_le(&frame[offset], (float)pool->mode);                         offset += 4U;
    put_float_le(&frame[offset], pool->target.speed_mps);                    offset += 4U;
    put_float_le(&frame[offset], pool->estimation.body_speed_mps);           offset += 4U;
    put_float_le(&frame[offset], pool->motion.left_target_mps);              offset += 4U;
    put_float_le(&frame[offset], pool->estimation.left_speed_mps);           offset += 4U;
    put_float_le(&frame[offset], (float)pool->motion.left_pwm);              offset += 4U;
    put_float_le(&frame[offset], pool->motion.right_target_mps);             offset += 4U;
    put_float_le(&frame[offset], pool->estimation.right_speed_mps);          offset += 4U;
    put_float_le(&frame[offset], (float)pool->motion.right_pwm);             offset += 4U;
    put_float_le(&frame[offset], (float)build_fault_mask(pool));             offset += 4U;
    frame[offset++] = DEBUG_FRAME_TAIL0;
    frame[offset++] = DEBUG_FRAME_TAIL1;
    frame[offset++] = DEBUG_FRAME_TAIL2;
    frame[offset++] = DEBUG_FRAME_TAIL3;

    (void)BspUart_WriteBuffer(BSP_UART_BT, frame, offset);
}
