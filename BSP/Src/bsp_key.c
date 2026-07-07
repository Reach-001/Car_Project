#include "bsp_key.h"

#include "main.h"
#include "stm32g4xx_hal.h"

#define KEY_DEBOUNCE_TICKS 3U
#define KEY_CLICK_MAX_MS 800U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    bool active_high;
    bool stable_pressed;
    bool last_raw_pressed;
    uint8_t same_count;
    uint32_t pressed_at_ms;
    BspKeyInfo info;
} KeyContext;

static KeyContext s_keys[BSP_KEY_COUNT] = {
    {KEY1_GPIO_Port, KEY1_Pin, true, false, false, 0U, 0U, {0}},
    {KEY2_GPIO_Port, KEY2_Pin, true, false, false, 0U, 0U, {0}},
    {KEY3_GPIO_Port, KEY3_Pin, true, false, false, 0U, 0U, {0}},
};

static bool read_key_raw(const KeyContext *key)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(key->port, key->pin);
    bool high = state == GPIO_PIN_SET;
    return key->active_high ? high : !high;
}

static KeyContext *get_key(BspKeyId key)
{
    if ((uint32_t)key >= (uint32_t)BSP_KEY_COUNT)
    {
        return 0;
    }

    return &s_keys[key];
}

void BspKey_Init(void)
{
    for (uint32_t i = 0U; i < (uint32_t)BSP_KEY_COUNT; ++i)
    {
        KeyContext *key = &s_keys[i];
        bool raw = read_key_raw(key);
        key->stable_pressed = raw;
        key->last_raw_pressed = raw;
        key->same_count = 0U;
        key->pressed_at_ms = raw ? HAL_GetTick() : 0U;
        key->info.pressed = raw;
        key->info.pressed_event = false;
        key->info.released_event = false;
        key->info.clicked_event = false;
        key->info.pressed_time_ms = 0U;
    }
}

void BspKey_Task10ms(void)
{
    uint32_t now = HAL_GetTick();

    for (uint32_t i = 0U; i < (uint32_t)BSP_KEY_COUNT; ++i)
    {
        KeyContext *key = &s_keys[i];
        bool raw = read_key_raw(key);

        if (raw == key->last_raw_pressed)
        {
            if (key->same_count < KEY_DEBOUNCE_TICKS)
            {
                ++key->same_count;
            }
        }
        else
        {
            key->last_raw_pressed = raw;
            key->same_count = 0U;
        }

        if ((key->same_count >= KEY_DEBOUNCE_TICKS) && (raw != key->stable_pressed))
        {
            key->stable_pressed = raw;
            key->info.pressed = raw;

            if (raw)
            {
                key->pressed_at_ms = now;
                key->info.pressed_event = true;
                key->info.pressed_time_ms = 0U;
            }
            else
            {
                uint32_t duration = now - key->pressed_at_ms;
                key->info.released_event = true;
                key->info.clicked_event = duration <= KEY_CLICK_MAX_MS;
                key->info.pressed_time_ms = duration;
            }
        }

        if (key->stable_pressed)
        {
            key->info.pressed_time_ms = now - key->pressed_at_ms;
        }
    }
}

bool BspKey_IsPressed(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    return (ctx != 0) && ctx->info.pressed;
}

bool BspKey_TakePressedEvent(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    bool event = (ctx != 0) && ctx->info.pressed_event;
    if (ctx != 0)
    {
        ctx->info.pressed_event = false;
    }
    return event;
}

bool BspKey_TakeReleasedEvent(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    bool event = (ctx != 0) && ctx->info.released_event;
    if (ctx != 0)
    {
        ctx->info.released_event = false;
    }
    return event;
}

bool BspKey_TakeClickedEvent(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    bool event = (ctx != 0) && ctx->info.clicked_event;
    if (ctx != 0)
    {
        ctx->info.clicked_event = false;
    }
    return event;
}

BspKeyInfo BspKey_GetInfo(BspKeyId key)
{
    BspKeyInfo empty = {0};
    KeyContext *ctx = get_key(key);
    if (ctx == 0)
    {
        return empty;
    }

    return ctx->info;
}
