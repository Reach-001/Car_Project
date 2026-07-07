#include "k230_link.h"

#include "protocol.h"
#include "stm32g4xx_hal.h"

static ProtocolLineParser s_parser;
static K230Result s_latest_result;

void K230Link_Init(void)
{
    ProtocolLineParser_Init(&s_parser);
    s_latest_result.type = K230_RESULT_NONE;
    s_latest_result.value0 = 0;
    s_latest_result.value1 = 0;
    s_latest_result.value2 = 0;
    s_latest_result.timestamp_ms = 0U;
    s_latest_result.valid = false;
}

void K230Link_OnRxByte(uint8_t byte)
{
    (void)ProtocolLineParser_PushByte(&s_parser, byte);
}

void K230Link_Task(void)
{
    char line[PROTOCOL_LINE_MAX_LEN];

    if (ProtocolLineParser_TakeLine(&s_parser, line, sizeof(line)))
    {
        s_latest_result.type = K230_RESULT_CUSTOM;
        s_latest_result.value0 = 0;
        s_latest_result.value1 = 0;
        s_latest_result.value2 = 0;
        s_latest_result.timestamp_ms = HAL_GetTick();
        s_latest_result.valid = true;
    }
}

bool K230Link_GetLatestResult(K230Result *result)
{
    if ((result == 0) || !s_latest_result.valid)
    {
        return false;
    }

    *result = s_latest_result;
    return true;
}

void K230Link_SendStatus(void)
{
}
