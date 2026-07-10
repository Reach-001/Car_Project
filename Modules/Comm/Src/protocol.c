/* ────────────────────────────────────────────────────────────
 * 通用行协议解析器（Comm 域内部使用）
 *
 * 以 \n 或 \r 作为行结束符，逐字节喂入。
 * 满行后 ready=true，提取后自动复位。
 * ──────────────────────────────────────────────────────────── */

#include "comm_internal.h"

void ProtocolLineParser_Init(ProtocolLineParser *p)
{
    if (p == 0) return;
    p->length = 0U;
    p->ready  = false;
    p->data[0] = '\0';
}

bool ProtocolLineParser_PushByte(ProtocolLineParser *p, uint8_t byte)
{
    if ((p == 0) || p->ready) return false;

    if ((byte == '\n') || (byte == '\r'))
    {
        if (p->length > 0U)
        {
            p->data[p->length] = '\0';
            p->ready = true;
            return true;
        }
        return false;
    }

    if (p->length >= (PROTOCOL_LINE_MAX_LEN - 1U))
    {
        ProtocolLineParser_Init(p);
        return false;
    }

    p->data[p->length++] = (char)byte;
    return false;
}

bool ProtocolLineParser_TakeLine(ProtocolLineParser *p, char *buf, size_t sz)
{
    if ((p == 0) || (buf == 0) || (sz == 0U) || !p->ready) return false;

    size_t i = 0U;
    while ((i + 1U) < sz && p->data[i] != '\0') { buf[i] = p->data[i]; ++i; }
    buf[i] = '\0';

    ProtocolLineParser_Init(p);
    return true;
}
