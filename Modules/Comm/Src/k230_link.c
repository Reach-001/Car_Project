/* ────────────────────────────────────────────────────────────
 * K230 通信子模块（Comm 域内部使用）
 *
 * 数据流：K230 UART DMA → RingBuffer → 行协议解析 → K230Result
 *
 * 暂不实现完整的 K230 视觉协议解析（由用户后续补充）。
 * 当前占位：收到任何行数据 → 标记为自定义结果。
 * ──────────────────────────────────────────────────────────── */

#include "comm_internal.h"

#include "bsp_uart.h"
#include "system_state_pool.h"
#include "stm32g4xx_hal.h"     /* HAL_GetTick */

#include <stddef.h>             /* size_t */

static ProtocolLineParser s_parser;
static K230Result         s_latest;
static uint32_t           s_last_rx_ms;
static uint32_t           s_rx_count;

void K230Link_Task(void)
{
    uint8_t byte;
    while (BspUart_ReadByte(BSP_UART_K230, &byte))
    {
        ++s_rx_count;
        s_last_rx_ms = HAL_GetTick();
        ProtocolLineParser_PushByte(&s_parser, byte);
    }

    char line[256];
    if (ProtocolLineParser_TakeLine(&s_parser, line, sizeof(line)))
    {
        s_latest.type         = K230_RESULT_CUSTOM;
        s_latest.value0       = 0;
        s_latest.value1       = 0;
        s_latest.value2       = 0;
        s_latest.timestamp_ms = HAL_GetTick();
        s_latest.valid        = true;
    }
}

bool K230Link_GetLatestResult(K230Result *res)
{
    if ((res == 0) || !s_latest.valid) return false;
    *res = s_latest;
    return true;
}

void K230Link_GetStatus(bool *conn, uint32_t *last, uint32_t *cnt)
{
    if (conn)  *conn  = ((uint32_t)(HAL_GetTick() - s_last_rx_ms) < 2000U);
    if (last)  *last  = s_last_rx_ms;
    if (cnt)   *cnt   = s_rx_count;
}
