#include "bsp_io.h"

#include "main.h"

void BspIo_Init(void)
{
    BspIo_SetStateLed(false);
    BspIo_SetBuzzer(false);
}

void BspIo_SetStateLed(bool on)
{
    HAL_GPIO_WritePin(STATE_LED_GPIO_Port, STATE_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BspIo_SetBuzzer(bool on)
{
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

BspKeyState BspIo_ReadKeys(void)
{
    BspKeyState state;
    state.key1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET;
    state.key2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET;
    state.key3 = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_SET;
    return state;
}

BspTrackState BspIo_ReadTrack(void)
{
    BspTrackState state;
    state.sensor[0] = HAL_GPIO_ReadPin(TRACK_1_GPIO_Port, TRACK_1_Pin) == GPIO_PIN_SET;
    state.sensor[1] = HAL_GPIO_ReadPin(TRACK_2_GPIO_Port, TRACK_2_Pin) == GPIO_PIN_SET;
    state.sensor[2] = HAL_GPIO_ReadPin(TRACK_3_GPIO_Port, TRACK_3_Pin) == GPIO_PIN_SET;
    state.sensor[3] = HAL_GPIO_ReadPin(TRACK_4_GPIO_Port, TRACK_4_Pin) == GPIO_PIN_SET;
    state.sensor[4] = HAL_GPIO_ReadPin(TRACK_5_GPIO_Port, TRACK_5_Pin) == GPIO_PIN_SET;
    state.bits = 0U;

    for (uint32_t i = 0U; i < 5U; ++i)
    {
        if (state.sensor[i])
        {
            state.bits |= (uint8_t)(1U << i);
        }
    }

    return state;
}
