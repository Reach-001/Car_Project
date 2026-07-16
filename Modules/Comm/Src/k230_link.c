/* ────────────────────────────────────────────────────────────
 * K230 通信子模块（Comm 域内部使用）
 *
 * 数据流：K230 UART DMA → RingBuffer → 行协议解析 → K230Result
 *
 * 当前解析 K230 输出的 "类型:数值" 指令：
 *   S:n 停车 n 秒；V:n 速度；P:n 泊车模式；L/R:n 转弯标记。
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

#define K230_LINE_IDLE_FRAME_MS 50U

static bool parse_int16_prefix(const char *text, int16_t *out, const char **end_out)
{
    int32_t value = 0;
    int32_t sign = 1;
    const char *p = text;

    if ((p == 0) || (out == 0)) return false;

    while ((*p == ' ') || (*p == '\t')) { ++p; }
    if (*p == '-')
    {
        sign = -1;
        ++p;
    }
    else if (*p == '+')
    {
        ++p;
    }

    if ((*p < '0') || (*p > '9')) return false;
    while ((*p >= '0') && (*p <= '9'))
    {
        value = (value * 10) + (int32_t)(*p - '0');
        if (value > 32767) return false;
        ++p;
    }

    *out = (int16_t)(value * sign);
    if (end_out != 0)
    {
        *end_out = p;
    }
    return true;
}

static char to_upper_ascii(char c)
{
    if ((c >= 'a') && (c <= 'z'))
    {
        return (char)(c - ('a' - 'A'));
    }
    return c;
}

static bool is_ascii_alpha(char c)
{
    return ((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z'));
}

static bool parse_k230_command(const char *line, K230Result *result)
{
    char cmd;
    int16_t value;
    const char *p;

    if ((line == 0) || (result == 0) || (line[0] == '\0'))
    {
        return false;
    }

    p = line;
    while (*p != '\0')
    {
        const char *q;

        cmd = to_upper_ascii(*p);
        if ((p != line) && is_ascii_alpha(*(p - 1)))
        {
            ++p;
            continue;
        }

        if ((cmd != 'S') && (cmd != 'V') && (cmd != 'P') &&
            (cmd != 'L') && (cmd != 'R'))
        {
            ++p;
            continue;
        }

        q = p + 1;
        while ((*q == ' ') || (*q == '\t')) { ++q; }
        if (*q != ':')
        {
            ++p;
            continue;
        }

        ++q;
        if (!parse_int16_prefix(q, &value, 0))
        {
            ++p;
            continue;
        }

        result->value0 = value;
        result->value1 = 0;
        result->value2 = 0;
        result->timestamp_ms = HAL_GetTick();
        result->valid = true;

        if (cmd == 'S')
        {
            result->type = K230_RESULT_CMD_STOP;
            return true;
        }
        if (cmd == 'V')
        {
            result->type = K230_RESULT_CMD_SPEED;
            return true;
        }
        if (cmd == 'P')
        {
            result->type = K230_RESULT_CMD_PARK;
            return true;
        }
        if ((cmd == 'L') || (cmd == 'R'))
        {
            result->type = K230_RESULT_CMD_TURN;
            result->value0 = (cmd == 'L') ? -1 : 1;
            result->value1 = value;
            return true;
        }

        ++p;
    }

    return false;
}

static void k230_parse_ready_line(void)
{
    char line[256];
    K230Result parsed;

    if (ProtocolLineParser_TakeLine(&s_parser, line, sizeof(line)) &&
        parse_k230_command(line, &parsed))
    {
        s_latest = parsed;
    }
}

void K230Link_Task(void)
{
    uint8_t byte;

    while (BspUart_ReadByte(BSP_UART_K230, &byte))
    {
        ++s_rx_count;
        s_last_rx_ms = HAL_GetTick();

        if (ProtocolLineParser_PushByte(&s_parser, byte) &&
            s_parser.ready)
        {
            k230_parse_ready_line();
        }
    }

    if (!s_parser.ready &&
        (s_parser.length > 0U) &&
        ((uint32_t)(HAL_GetTick() - s_last_rx_ms) >= K230_LINE_IDLE_FRAME_MS))
    {
        s_parser.data[s_parser.length] = '\0';
        s_parser.ready = true;
        k230_parse_ready_line();
    }
}

bool K230Link_GetLatestResult(K230Result *res)
{
    if ((res == 0) || !s_latest.valid) return false;
    *res = s_latest;
    s_latest.valid = false;
    return true;
}

bool K230Link_RequestDetect(void)
{
    static const uint8_t cmd[] = "A:1\r\n";

    if (!BspUart_TxDone(BSP_UART_K230))
    {
        return false;
    }

    return BspUart_WriteBuffer(BSP_UART_K230, cmd, (uint16_t)(sizeof(cmd) - 1U));
}

void K230Link_GetStatus(bool *conn, uint32_t *last, uint32_t *cnt)
{
    if (conn)  *conn  = (s_last_rx_ms != 0U) &&
                        ((uint32_t)(HAL_GetTick() - s_last_rx_ms) < 2000U);
    if (last)  *last  = s_last_rx_ms;
    if (cnt)   *cnt   = s_rx_count;
}
