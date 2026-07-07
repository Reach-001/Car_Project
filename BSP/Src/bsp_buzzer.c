#include "bsp_buzzer.h"

#include "main.h"
#include "stm32g4xx_hal.h"

#define BUZZER_DEFAULT_FREQ_HZ 2000U
#define BUZZER_MIN_FREQ_HZ 200U
#define BUZZER_MAX_FREQ_HZ 5000U
#define BUZZER_TIM_CLOCK_HZ 1000000U

typedef struct
{
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t repeat;
} BuzzerPatternStep;

static const BuzzerPatternStep s_patterns[] = {
    [BUZZER_PATTERN_NONE] = {0U, 0U, 0U},
    [BUZZER_PATTERN_OK] = {80U, 0U, 1U},
    [BUZZER_PATTERN_ERROR] = {120U, 120U, 3U},
    [BUZZER_PATTERN_START] = {60U, 80U, 2U},
    [BUZZER_PATTERN_OBSTACLE] = {50U, 50U, 5U},
};

static TIM_HandleTypeDef s_htim7;
static BuzzerPatternStep s_active;
static uint32_t s_next_change_ms;
static uint16_t s_frequency_hz;
static uint8_t s_remaining;
static volatile bool s_tone_enabled;
static volatile bool s_gpio_high;
static bool s_envelope_on;

static void buzzer_gpio_write(bool high)
{
    s_gpio_high = high;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t frequency_to_period(uint16_t frequency_hz)
{
    uint32_t half_period_us;

    if (frequency_hz < BUZZER_MIN_FREQ_HZ)
    {
        frequency_hz = BUZZER_MIN_FREQ_HZ;
    }
    else if (frequency_hz > BUZZER_MAX_FREQ_HZ)
    {
        frequency_hz = BUZZER_MAX_FREQ_HZ;
    }

    half_period_us = BUZZER_TIM_CLOCK_HZ / ((uint32_t)frequency_hz * 2U);
    if (half_period_us == 0U)
    {
        half_period_us = 1U;
    }

    return half_period_us - 1U;
}

static void buzzer_timer_config(uint16_t frequency_hz)
{
    uint32_t period = frequency_to_period(frequency_hz);

    __HAL_TIM_DISABLE_IT(&s_htim7, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&s_htim7);
    __HAL_TIM_SET_AUTORELOAD(&s_htim7, period);
    __HAL_TIM_SET_COUNTER(&s_htim7, 0U);
    __HAL_TIM_CLEAR_FLAG(&s_htim7, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&s_htim7, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&s_htim7);
}

static void tone_start(void)
{
    s_tone_enabled = true;
    s_envelope_on = true;
    buzzer_timer_config(s_frequency_hz);
}

static void tone_stop(void)
{
    s_tone_enabled = false;
    s_envelope_on = false;
    __HAL_TIM_DISABLE_IT(&s_htim7, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&s_htim7);
    buzzer_gpio_write(false);
}

void BspBuzzer_Init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();

    s_htim7.Instance = TIM7;
    s_htim7.Init.Prescaler = 169U;
    s_htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim7.Init.Period = frequency_to_period(BUZZER_DEFAULT_FREQ_HZ);
    s_htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    (void)HAL_TIM_Base_Init(&s_htim7);

    HAL_NVIC_SetPriority(TIM7_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    s_active = s_patterns[BUZZER_PATTERN_NONE];
    s_next_change_ms = 0U;
    s_frequency_hz = BUZZER_DEFAULT_FREQ_HZ;
    s_remaining = 0U;
    s_tone_enabled = false;
    s_envelope_on = false;
    buzzer_gpio_write(false);
}

void BspBuzzer_Set(bool on)
{
    s_active = s_patterns[BUZZER_PATTERN_NONE];
    s_remaining = 0U;
    s_next_change_ms = 0U;

    if (on)
    {
        tone_start();
    }
    else
    {
        tone_stop();
    }
}

void BspBuzzer_SetFrequency(uint16_t frequency_hz)
{
    if (frequency_hz < BUZZER_MIN_FREQ_HZ)
    {
        frequency_hz = BUZZER_MIN_FREQ_HZ;
    }
    else if (frequency_hz > BUZZER_MAX_FREQ_HZ)
    {
        frequency_hz = BUZZER_MAX_FREQ_HZ;
    }

    s_frequency_hz = frequency_hz;
    if (s_tone_enabled)
    {
        buzzer_timer_config(s_frequency_hz);
    }
}

void BspBuzzer_Beep(uint16_t on_ms)
{
    s_active.on_ms = on_ms;
    s_active.off_ms = 0U;
    s_active.repeat = 1U;
    s_remaining = 1U;
    tone_start();
    s_next_change_ms = HAL_GetTick() + on_ms;
}

void BspBuzzer_Play(BuzzerPattern pattern)
{
    if ((uint32_t)pattern >= (sizeof(s_patterns) / sizeof(s_patterns[0])))
    {
        return;
    }

    s_active = s_patterns[pattern];
    s_remaining = s_active.repeat;
    if (s_remaining == 0U)
    {
        tone_stop();
        return;
    }

    tone_start();
    s_next_change_ms = HAL_GetTick() + s_active.on_ms;
}

void BspBuzzer_Task10ms(void)
{
    uint32_t now = HAL_GetTick();

    if (s_remaining == 0U)
    {
        return;
    }

    if ((int32_t)(now - s_next_change_ms) < 0)
    {
        return;
    }

    if (s_envelope_on)
    {
        tone_stop();
        if (s_active.off_ms == 0U)
        {
            s_remaining = 0U;
        }
        else
        {
            s_next_change_ms = now + s_active.off_ms;
        }
    }
    else
    {
        --s_remaining;
        if (s_remaining == 0U)
        {
            tone_stop();
        }
        else
        {
            tone_start();
            s_next_change_ms = now + s_active.on_ms;
        }
    }
}

void BspBuzzer_IrqHandler(void)
{
    if ((__HAL_TIM_GET_FLAG(&s_htim7, TIM_FLAG_UPDATE) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(&s_htim7, TIM_IT_UPDATE) != RESET))
    {
        __HAL_TIM_CLEAR_IT(&s_htim7, TIM_IT_UPDATE);
        if (s_tone_enabled)
        {
            buzzer_gpio_write(!s_gpio_high);
        }
        else
        {
            buzzer_gpio_write(false);
        }
    }
}

bool BspBuzzer_IsActive(void)
{
    return s_tone_enabled || (s_remaining > 0U);
}
