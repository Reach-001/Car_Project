#include "bsp_led.h"

#include "main.h"
#include "stm32g4xx_hal.h"

void BspLed_Init(void)
{
    BspLed_SetStateLed(false);
}

void BspLed_SetStateLed(bool on)
{
    HAL_GPIO_WritePin(STATE_LED_GPIO_Port, STATE_LED_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
