#include "comm.h"
#include "comm_internal.h"

#include "bsp_uart.h"

/* ────────────────────────────────────────────────────────────
 * Comm 域聚合入口
 *
 * 每 20ms 调用一次，轮询 UART RingBuffer：
 *   蓝牙（USART3）→ bt_link 解析 → pool->comm.bt_command + pool->event
 *   K230（USART2）→ k230_link 解析 → pool->comm.k230_result + pool->event
 *
 * 子模块内部函数在 bt_link.c / k230_link.c 中实现。
 * ──────────────────────────────────────────────────────────── */

static CommState s_state;

/* ── 初始化 ── */

void Comm_Init(void)
{
    s_state.bt_cmd.type  = BT_COMMAND_NONE;
    s_state.bt_cmd.valid = false;
    s_state.k230_res.type  = K230_RESULT_NONE;
    s_state.k230_res.valid = false;
    s_state.bt_connected   = false;
    s_state.k230_connected = false;
    s_state.bt_last_rx_ms  = 0U;
    s_state.k230_last_rx_ms = 0U;
    s_state.bt_rx_bytes    = 0U;
    s_state.k230_rx_bytes  = 0U;
}

/* ── 20ms 周期 ── */

void Comm_Task20ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    /* ── 蓝牙处理 ── */
    BtLink_Task();

    BtCommand bt_cmd;
    if (BtLink_TakeCommand(&bt_cmd))
    {
        if (bt_cmd.type == BT_COMMAND_DEBUG_OUTPUT)
        {
            pool->debug.enabled = (bt_cmd.arg0 != 0);
            pool->comm.debug_command = bt_cmd;
            s_state.bt_cmd           = bt_cmd;
        }
        else
        {
            pool->comm.bt_command        = bt_cmd;
            pool->event.bt_command_ready = true;
            s_state.bt_cmd               = bt_cmd;
        }
    }

    BtLink_GetStatus(&s_state.bt_connected, &s_state.bt_last_rx_ms, &s_state.bt_rx_bytes);

    /* ── K230 处理 ── */
    K230Link_Task();

    K230Result k230_res;
    if (K230Link_GetLatestResult(&k230_res))
    {
        pool->comm.k230_result          = k230_res;
        pool->event.k230_result_ready   = true;
        s_state.k230_res                = k230_res;
    }

    K230Link_GetStatus(&s_state.k230_connected, &s_state.k230_last_rx_ms, &s_state.k230_rx_bytes);
}

/* ── 状态查询 ── */

CommState Comm_GetState(void)
{
    return s_state;
}

void Comm_SetCommand(const CommCommand *cmd)
{
    (void)cmd;
}
