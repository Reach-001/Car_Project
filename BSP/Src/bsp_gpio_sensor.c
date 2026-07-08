#include "bsp_gpio_sensor.h"

#include "main.h"
#include "stm32g4xx_hal.h"

/* ── Track sensor ── */

BspTrackRaw BspGpioSensor_ReadTrack(void)
{
    BspTrackRaw raw;

    raw.sensor[0] = (HAL_GPIO_ReadPin(TRACK_1_GPIO_Port, TRACK_1_Pin) == GPIO_PIN_SET);
    raw.sensor[1] = (HAL_GPIO_ReadPin(TRACK_2_GPIO_Port, TRACK_2_Pin) == GPIO_PIN_SET);
    raw.sensor[2] = (HAL_GPIO_ReadPin(TRACK_3_GPIO_Port, TRACK_3_Pin) == GPIO_PIN_SET);
    raw.sensor[3] = (HAL_GPIO_ReadPin(TRACK_4_GPIO_Port, TRACK_4_Pin) == GPIO_PIN_SET);
    raw.sensor[4] = (HAL_GPIO_ReadPin(TRACK_5_GPIO_Port, TRACK_5_Pin) == GPIO_PIN_SET);

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

/* ── Ultrasonic TRIG (PB2, push-pull output) ── */

void BspGpioSensor_TrigSet(bool high)
{
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── Ultrasonic ECHO (PB11, EXTI input, read by ISR callback) ── */

bool BspGpioSensor_EchoRead(void)
{
    return (HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin) == GPIO_PIN_SET);
}
