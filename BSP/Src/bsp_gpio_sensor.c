#include "bsp_gpio_sensor.h"

#include "main.h"              /* TRACK_x, HCSR04_TRIG, HCSR04_ECHO 等宏 */
#include "stm32g4xx_hal.h"     /* HAL_GPIO_ReadPin, HAL_GPIO_WritePin */

/* ────────────────────────────────────────────────────────────
 * GPIO 传感器驱动实现
 *
 * 所有函数都是对 HAL GPIO API 的薄封装。
 * 上层 Module 通过这里统一访问传感器 GPIO，不直接调 HAL。
 * ──────────────────────────────────────────────────────────── */

/* ── 初始化 ── */

void BspGpioSensor_Init(void)
{
    BspGpioSensor_TrigSet(false);
}

/* ── 循迹传感器 ── */

BspTrackRaw BspGpioSensor_ReadTrack(void)
{
    BspTrackRaw raw;

    /* 逐路读取 GPIO 电平（高 = 黑线检测） */
    raw.sensor[0] = (HAL_GPIO_ReadPin(TRACK_1_GPIO_Port, TRACK_1_Pin) == GPIO_PIN_SET);
    raw.sensor[1] = (HAL_GPIO_ReadPin(TRACK_2_GPIO_Port, TRACK_2_Pin) == GPIO_PIN_SET);
    raw.sensor[2] = (HAL_GPIO_ReadPin(TRACK_3_GPIO_Port, TRACK_3_Pin) == GPIO_PIN_SET);
    raw.sensor[3] = (HAL_GPIO_ReadPin(TRACK_4_GPIO_Port, TRACK_4_Pin) == GPIO_PIN_SET);
    raw.sensor[4] = (HAL_GPIO_ReadPin(TRACK_5_GPIO_Port, TRACK_5_Pin) == GPIO_PIN_SET);

    /* 构建 bit 位图 */
    raw.bits = 0U;
    for (uint32_t i = 0U; i < 5U; ++i)
    {
        if (raw.sensor[i])
        {
            raw.bits |= (uint8_t)(1U << i);
        }
    }

    return raw;
}

/* ── 超声波 TRIG（PB2，推挽输出） ── */

void BspGpioSensor_TrigSet(bool high)
{
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── 超声波 ECHO（PB11，EXTI 输入，电平由 ISR 读取） ── */

bool BspGpioSensor_EchoRead(void)
{
    return (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_SET);
}
