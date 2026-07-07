#ifndef BT_LINK_H
#define BT_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BT_COMMAND_NONE = 0,
    BT_COMMAND_STOP,
    BT_COMMAND_SET_MODE,
    BT_COMMAND_MANUAL_MOVE,
    BT_COMMAND_START_TASK,
    BT_COMMAND_PAUSE_TASK,
    BT_COMMAND_SET_PARAM,
    BT_COMMAND_CUSTOM
} BtCommandType;

typedef struct
{
    BtCommandType type;
    int16_t arg0;
    int16_t arg1;
    int16_t arg2;
    uint32_t timestamp_ms;
    bool valid;
} BtCommand;

void BtLink_Init(void);
void BtLink_OnRxByte(uint8_t byte);
void BtLink_Task(void);
bool BtLink_TakeCommand(BtCommand *command);
void BtLink_SendStatus(void);

#endif
