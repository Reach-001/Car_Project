#include "bsp_led.h"

#include "main.h"              /* LED1/2/3/STATE_LED Pin + Port 宏 */
#include "stm32g4xx_hal.h"     /* HAL_GPIO_WritePin, HAL_GPIO_TogglePin */

/* ────────────────────────────────────────────────────────────
 * LED 驱动实现
 *
 *   LED1       PA4     GPIOA     低电平点亮
 *   LED2       PA5     GPIOA     低电平点亮
 *   LED3       PC4     GPIOC     低电平点亮
 *   STATE_LED  PC6     GPIOC     高电平点亮
 *
 * Set(on=true) 统一语义 = "灯亮"，内部根据 active_low 反相电平。
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    bool          active_low;     /* true = 低电平点亮，false = 高电平点亮 */
} LedCfg;

static const LedCfg s_led_cfg[BSP_LED_COUNT] = {
    [BSP_LED_1]     = {LED1_GPIO_Port,     LED1_Pin,     true},   /* 低电平亮 */
    [BSP_LED_2]     = {LED2_GPIO_Port,     LED2_Pin,     true},   /* 低电平亮 */
    [BSP_LED_3]     = {LED3_GPIO_Port,     LED3_Pin,     true},   /* 低电平亮 */
    [BSP_LED_STATE] = {STATE_LED_GPIO_Port, STATE_LED_Pin, false}, /* 高电平亮 */
};

/* on=true 对应灯亮，内部根据 active_low 映射为实际 GPIO 电平 */
static GPIO_PinState led_level(const LedCfg *cfg, bool on)
{
    if (cfg->active_low)
    {
        /* 低电平点亮：on=true → GPIO_RESET, on=false → GPIO_SET */
        return on ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else
    {
        /* 高电平点亮：on=true → GPIO_SET, on=false → GPIO_RESET */
        return on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
}

/* ── 初始化 ── */

void BspLed_Init(void)
{
    /* CubeMX 已配好 GPIO 模式，这里复位输出电平（全灭 = active_low 的给高，普通给低） */
    for (uint32_t i = 0U; i < (uint32_t)BSP_LED_COUNT; ++i)
    {
        HAL_GPIO_WritePin(s_led_cfg[i].port, s_led_cfg[i].pin,
                          led_level(&s_led_cfg[i], false));
    }
}

/* ── 控制接口 ── */

void BspLed_Set(BspLedId led, bool on)
{
    if ((uint32_t)led >= (uint32_t)BSP_LED_COUNT) { return; }

    HAL_GPIO_WritePin(s_led_cfg[led].port, s_led_cfg[led].pin,
                      led_level(&s_led_cfg[led], on));
}

void BspLed_Toggle(BspLedId led)
{
    if ((uint32_t)led >= (uint32_t)BSP_LED_COUNT) { return; }

    /* 读当前电平 → 取反 → 写回，兼容 active_low */
    GPIO_PinState current = HAL_GPIO_ReadPin(s_led_cfg[led].port, s_led_cfg[led].pin);
    GPIO_PinState next    = (current == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(s_led_cfg[led].port, s_led_cfg[led].pin, next);
}
