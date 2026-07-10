/* ────────────────────────────────────────────────────────────
 * 蓝牙通信子模块（Comm 域内部使用）
 *
 * 数据流：BT UART DMA → RingBuffer → 行协议解析 → BtCommand
 *
 * 暂不实现完整的蓝牙遥控器协议解析（由用户后续补充）。
 * 当前占位：收到任何行数据 → 标记为自定义命令。
 * ──────────────────────────────────────────────────────────── */

#include "comm_internal.h"

#include "bsp_uart.h"
#include "system_state_pool.h"
#include "stm32g4xx_hal.h"     /* HAL_GetTick */

#include <stddef.h>             /* size_t */

static ProtocolLineParser s_parser;
static BtCommand           s_pending;
static uint32_t            s_last_rx_ms;
static uint32_t            s_rx_count;

void BtLink_Task(void)
{
    uint8_t byte;
    while (BspUart_ReadByte(BSP_UART_BT, &byte))
    {
        ++s_rx_count;
        s_last_rx_ms = HAL_GetTick();
        ProtocolLineParser_PushByte(&s_parser, byte);
    }

    char line[256];
    if (ProtocolLineParser_TakeLine(&s_parser, line, sizeof(line)))
    {
        /* 收到一行，填充占位命令 */
        s_pending.type         = BT_COMMAND_CUSTOM;
        s_pending.arg0         = 0;
        s_pending.arg1         = 0;
        s_pending.arg2         = 0;
        s_pending.timestamp_ms = HAL_GetTick();
        s_pending.valid        = true;
    }
}

bool BtLink_TakeCommand(BtCommand *cmd)
{
    if ((cmd == 0) || !s_pending.valid) return false;
    *cmd = s_pending;
    s_pending.valid = false;
    return true;
}

void BtLink_GetStatus(bool *conn, uint32_t *last, uint32_t *cnt)
{
    if (conn)  *conn  = ((uint32_t)(HAL_GetTick() - s_last_rx_ms) < 2000U);
    if (last)  *last  = s_last_rx_ms;
    if (cnt)   *cnt   = s_rx_count;
}
