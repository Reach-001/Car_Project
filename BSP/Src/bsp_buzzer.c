#include "bsp_buzzer.h"

#include "main.h"
#include "stm32g4xx_hal.h"

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

static BuzzerPatternStep s_active;
static uint32_t s_next_change_ms;
static uint8_t s_remaining;
static bool s_on;

static void set_output(bool on)
{
    s_on = on;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BspBuzzer_Init(void)
{
    s_active = s_patterns[BUZZER_PATTERN_NONE];
    s_next_change_ms = 0U;
    s_remaining = 0U;
    set_output(false);
}

void BspBuzzer_Set(bool on)
{
    s_active = s_patterns[BUZZER_PATTERN_NONE];
    s_remaining = 0U;
    s_next_change_ms = 0U;
    set_output(on);
}

void BspBuzzer_Beep(uint16_t on_ms)
{
    s_active.on_ms = on_ms;
    s_active.off_ms = 0U;
    s_active.repeat = 1U;
    s_remaining = 1U;
    set_output(true);
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
        set_output(false);
        return;
    }

    set_output(true);
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

    if (s_on)
    {
        set_output(false);
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
            set_output(false);
        }
        else
        {
            set_output(true);
            s_next_change_ms = now + s_active.on_ms;
        }
    }
}

bool BspBuzzer_IsActive(void)
{
    return s_on || (s_remaining > 0U);
}
