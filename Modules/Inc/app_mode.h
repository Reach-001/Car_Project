#ifndef APP_MODE_H
#define APP_MODE_H

#include <stdint.h>

#include "bt_link.h"
#include "k230_link.h"

typedef enum
{
    APP_MODE_STOP = 0,
    APP_MODE_MANUAL,
    APP_MODE_LINE_FOLLOW,
    APP_MODE_INSPECTION,
    APP_MODE_AVOIDANCE,
    APP_MODE_ERROR
} AppMode;

typedef struct
{
    AppMode mode;
    uint32_t mode_enter_ms;
    uint32_t last_command_ms;
} AppModeState;

void AppMode_Init(void);
void AppMode_SetMode(AppMode mode);
AppModeState AppMode_GetState(void);
void AppMode_HandleBtCommand(const BtCommand *command);
void AppMode_HandleK230Result(const K230Result *result);
void AppMode_Task20ms(void);

#endif
