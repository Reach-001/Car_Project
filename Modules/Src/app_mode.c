#include "app_mode.h"

#include "chassis.h"
#include "stm32g4xx_hal.h"

static AppModeState s_state;

void AppMode_Init(void)
{
    s_state.mode = APP_MODE_STOP;
    s_state.mode_enter_ms = HAL_GetTick();
    s_state.last_command_ms = 0U;
    Chassis_Stop();
}

void AppMode_SetMode(AppMode mode)
{
    if (s_state.mode == mode)
    {
        return;
    }

    s_state.mode = mode;
    s_state.mode_enter_ms = HAL_GetTick();

    if ((mode == APP_MODE_STOP) || (mode == APP_MODE_ERROR))
    {
        Chassis_Stop();
    }
}

AppModeState AppMode_GetState(void)
{
    return s_state;
}

void AppMode_HandleBtCommand(const BtCommand *command)
{
    if ((command == 0) || !command->valid)
    {
        return;
    }

    s_state.last_command_ms = command->timestamp_ms;

    if (command->type == BT_COMMAND_STOP)
    {
        AppMode_SetMode(APP_MODE_STOP);
    }
}

void AppMode_HandleK230Result(const K230Result *result)
{
    if ((result == 0) || !result->valid)
    {
        return;
    }
}

void AppMode_Task20ms(void)
{
}
