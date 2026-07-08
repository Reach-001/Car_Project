#include "bsp_led.h"

#include "main.h"              /* STATE_LED_Pin, STATE_LED_GPIO_Port */
#include "stm32g4xx_hal.h"     /* HAL_GPIO_WritePin */

/* ────────────────────────────────────────────────────────────
 * LED 驱动实现
 *
 * STATE_LED = PC6，推挽输出
 * LED 亮 = GPIO_SET（高电平），灭 = GPIO_RESET（低电平）
 * ──────────────────────────────────────────────────────────── */

void BspLed_Init(void)
{
    /* 上电默认灭 */
    BspLed_SetStateLed(false);
}

void BspLed_SetStateLed(bool on)
{
    HAL_GPIO_WritePin(STATE_LED_GPIO_Port, STATE_LED_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
