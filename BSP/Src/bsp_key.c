/* ────────────────────────────────────────────────────────────
 * 按键消抖驱动实现
 *
 * 消抖算法：
 *   1. 每 10ms 读一次 GPIO 电平
 *   2. 连续 KEY_DEBOUNCE_TICKS 次（默认3次=30ms）读到同值 → 确认状态变化
 *   3. 按下→释放时间 ≤ KEY_CLICK_MAX_MS（默认800ms）→ 判定为"单击"
 *
 * 电平约定：外部上拉（未按=高，按下=低）→ active_high=false
 *           直连 VCC（按下=高）           → active_high=true
 *
 * 参数说明：
 *   KEY_DEBOUNCE_TICKS = 消抖确认所需连续相同采样次数。
 *                        默认 3次 × 10ms = 30ms 消抖时间。
 *                        减小 → 响应快但可能误触发；
 *                        增大 → 更稳但按键感延迟。
 *
 *   KEY_CLICK_MAX_MS   = 单击判定最大按下时长（ms）。
 *                        按下后在此时长内释放 → 单击；
 *                        超过此时长 → 长按。
 *                        默认 800ms，和鼠标单击感接近。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_key.h"

#include "main.h"              /* KEY1_Pin, KEY1_GPIO_Port 等宏 */
#include "stm32g4xx_hal.h"     /* HAL_GPIO_ReadPin, HAL_GetTick */

/* ════════════════════════════════════════════════════════════
 * 消抖参数
 * ════════════════════════════════════════════════════════════ */

#define KEY_DEBOUNCE_TICKS 3U              /* 消抖确认采样次数（3次×10ms=30ms） */
#define KEY_CLICK_MAX_MS   800U            /* 单击判定最大按下时长（ms）          */

/* 单个按键的内部上下文 */
typedef struct
{
    GPIO_TypeDef *port;                    /* GPIO 端口基地址      */
    uint16_t      pin;                     /* GPIO 引脚号           */
    bool          active_high;             /* 高电平 = 按下？（KEY1~3=false） */
    bool          stable_pressed;          /* 去抖后确认的按下状态 */
    bool          last_raw_pressed;        /* 上一次原始采样值     */
    uint8_t       same_count;              /* 连续相同采样计数     */
    uint32_t      pressed_at_ms;           /* 最近一次按下的时刻   */
    BspKeyInfo    info;                    /* 对外暴露的状态信息   */
} KeyContext;

/* 四个按键静态表：KEY1~3 外部上拉(active_high=false)，KEY4 直连 VCC(active_high=true) */
static KeyContext s_keys[BSP_KEY_COUNT] = {
    {KEY1_GPIO_Port,      KEY1_Pin,      false, false, false, 0U, 0U, {0}},
    {KEY2_GPIO_Port,      KEY2_Pin,      false, false, false, 0U, 0U, {0}},
    {KEY3_GPIO_Port,      KEY3_Pin,      false, false, false, 0U, 0U, {0}},
    {User_Key_GPIO_Port, User_Key_Pin,   true,  false, false, 0U, 0U, {0}},
};

/* ── 内部辅助函数 ── */

static bool read_key_raw(const KeyContext *key)
{
    bool high = (HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_SET);
    return key->active_high ? high : !high;
}

static KeyContext *get_key(BspKeyId key)
{
    if ((uint32_t)key >= (uint32_t)BSP_KEY_COUNT) return 0;
    return &s_keys[key];
}

/* ── 初始化 ── */

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

/* ── 10ms 周期任务 ── */

void BspKey_Task10ms(void)
{
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0U; i < (uint32_t)BSP_KEY_COUNT; ++i)
    {
        KeyContext *key = &s_keys[i];
        bool raw = read_key_raw(key);

        /* 消抖计数 */
        if (raw == key->last_raw_pressed)
        {
            if (key->same_count < KEY_DEBOUNCE_TICKS) ++key->same_count;
        }
        else { key->last_raw_pressed = raw; key->same_count = 0U; }

        /* 消抖通过 → 边沿检测 */
        if ((key->same_count >= KEY_DEBOUNCE_TICKS) && (raw != key->stable_pressed))
        {
            key->stable_pressed = raw;
            key->info.pressed = raw;
            if (raw)
            {   /* 按下 */
                key->pressed_at_ms = now;
                key->info.pressed_event = true;
                key->info.pressed_time_ms = 0U;
            }
            else
            {   /* 释放 → 按持续时判定单击/长按 */
                uint32_t duration = now - key->pressed_at_ms;
                key->info.released_event = true;
                key->info.clicked_event = (duration <= KEY_CLICK_MAX_MS);
                key->info.pressed_time_ms = duration;
            }
        }

        /* 持续按下计时 */
        if (key->stable_pressed)
        {
            key->info.pressed_time_ms = now - key->pressed_at_ms;
        }
    }
}

/* ── 状态查询 / 事件获取（Take 语义：读后即清） ── */

bool BspKey_IsPressed(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    return (ctx != 0) && ctx->info.pressed;
}

bool BspKey_TakePressedEvent(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    bool event = (ctx != 0) && ctx->info.pressed_event;
    if (ctx != 0) ctx->info.pressed_event = false;
    return event;
}

bool BspKey_TakeReleasedEvent(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    bool event = (ctx != 0) && ctx->info.released_event;
    if (ctx != 0) ctx->info.released_event = false;
    return event;
}

bool BspKey_TakeClickedEvent(BspKeyId key)
{
    KeyContext *ctx = get_key(key);
    bool event = (ctx != 0) && ctx->info.clicked_event;
    if (ctx != 0) ctx->info.clicked_event = false;
    return event;
}

BspKeyInfo BspKey_GetInfo(BspKeyId key)
{
    BspKeyInfo empty = {0};
    KeyContext *ctx = get_key(key);
    if (ctx == 0) return empty;
    return ctx->info;
}
