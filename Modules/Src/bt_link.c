#include "bt_link.h"

#include "protocol.h"
#include "stm32g4xx_hal.h"

static ProtocolLineParser s_parser;
static BtCommand s_pending_command;

void BtLink_Init(void)
{
    ProtocolLineParser_Init(&s_parser);
    s_pending_command.type = BT_COMMAND_NONE;
    s_pending_command.arg0 = 0;
    s_pending_command.arg1 = 0;
    s_pending_command.arg2 = 0;
    s_pending_command.timestamp_ms = 0U;
    s_pending_command.valid = false;
}

void BtLink_OnRxByte(uint8_t byte)
{
    (void)ProtocolLineParser_PushByte(&s_parser, byte);
}

void BtLink_Task(void)
{
    char line[PROTOCOL_LINE_MAX_LEN];

    if (ProtocolLineParser_TakeLine(&s_parser, line, sizeof(line)))
    {
        s_pending_command.type = BT_COMMAND_CUSTOM;
        s_pending_command.arg0 = 0;
        s_pending_command.arg1 = 0;
        s_pending_command.arg2 = 0;
        s_pending_command.timestamp_ms = HAL_GetTick();
        s_pending_command.valid = true;
    }
}

bool BtLink_TakeCommand(BtCommand *command)
{
    if ((command == 0) || !s_pending_command.valid)
    {
        return false;
    }

    *command = s_pending_command;
    s_pending_command.valid = false;
    return true;
}

void BtLink_SendStatus(void)
{
}
