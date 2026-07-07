#include "ultrasonic.h"

#include "main.h"
#include "stm32g4xx_hal.h"

#define ULTRASONIC_TRIGGER_INTERVAL_MS 60U
#define ULTRASONIC_TIMEOUT_US 30000U
#define ULTRASONIC_SOUND_SPEED_MM_PER_US_X1000 343U

static volatile uint32_t s_echo_start_us;
static volatile uint32_t s_echo_width_us;
static volatile bool s_echo_done;
static volatile bool s_waiting_echo;
static uint32_t s_triggered_us;
static uint32_t s_last_trigger_ms;
static UltrasonicSample s_sample;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t micros(void)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U)
    {
        cycles_per_us = 1U;
    }
    return DWT->CYCCNT / cycles_per_us;
}

static void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us)
    {
    }
}

void Ultrasonic_Init(void)
{
    dwt_init();
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    s_echo_start_us = 0U;
    s_echo_width_us = 0U;
    s_echo_done = false;
    s_waiting_echo = false;
    s_triggered_us = 0U;
    s_last_trigger_ms = 0U;
    s_sample.status = ULTRASONIC_STATUS_IDLE;
    s_sample.distance_mm = 0U;
    s_sample.echo_us = 0U;
    s_sample.last_update_ms = 0U;
    s_sample.valid = false;
}

void Ultrasonic_Trigger(void)
{
    if (s_waiting_echo)
    {
        return;
    }

    s_echo_done = false;
    s_waiting_echo = true;
    s_triggered_us = micros();
    s_sample.status = ULTRASONIC_STATUS_WAIT_ECHO;

    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    delay_us(10U);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    s_last_trigger_ms = HAL_GetTick();
}

void Ultrasonic_OnEchoEdge(void)
{
    uint32_t now_us = micros();
    GPIO_PinState echo_state = HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin);

    if (echo_state == GPIO_PIN_SET)
    {
        s_echo_start_us = now_us;
    }
    else
    {
        if (s_echo_start_us != 0U)
        {
            s_echo_width_us = now_us - s_echo_start_us;
            s_echo_done = true;
            s_waiting_echo = false;
        }
    }
}

void Ultrasonic_Task10ms(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (s_echo_done)
    {
        uint32_t echo = s_echo_width_us;
        s_echo_done = false;
        s_sample.echo_us = echo;
        s_sample.distance_mm = (uint16_t)((echo * ULTRASONIC_SOUND_SPEED_MM_PER_US_X1000) / 2000U);
        s_sample.last_update_ms = now_ms;
        s_sample.status = ULTRASONIC_STATUS_READY;
        s_sample.valid = true;
    }

    if (s_waiting_echo && ((uint32_t)(micros() - s_triggered_us) > ULTRASONIC_TIMEOUT_US))
    {
        s_waiting_echo = false;
        s_sample.status = ULTRASONIC_STATUS_TIMEOUT;
        s_sample.valid = false;
        s_sample.last_update_ms = now_ms;
    }

    if (!s_waiting_echo && ((uint32_t)(now_ms - s_last_trigger_ms) >= ULTRASONIC_TRIGGER_INTERVAL_MS))
    {
        Ultrasonic_Trigger();
    }
}

UltrasonicSample Ultrasonic_GetSample(void)
{
    return s_sample;
}

bool Ultrasonic_IsObstacleNear(uint16_t threshold_mm)
{
    return s_sample.valid && (s_sample.distance_mm <= threshold_mm);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == HCSR04_ECHO_Pin)
    {
        Ultrasonic_OnEchoEdge();
    }
}
