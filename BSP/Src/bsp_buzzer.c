#include "bsp_buzzer.h"

#include "main.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

/* ────────────────────────────────────────────────────────────
 * 无源蜂鸣器驱动实现
 *
 * TIM7 (CubeMX IOC 管理)：Prescaler=169, Period=249 → 默认 2000Hz
 * HAL_TIM_PeriodElapsedCallback → 翻转 PB10 → 产生方波
 *
 * 状态管理：
 *   s_active = true  → TIM7 运行，ISR 翻转 GPIO → 蜂鸣器发声
 *   s_active = false → TIM7 停止，GPIO 拉低 → 静音
 *
 * 提示音状态机：on_ms 发声 → off_ms 静音 → repeat 次。
 * 由 BspBuzzer_Task10ms() 每 10ms 驱动。
 * ──────────────────────────────────────────────────────────── */

#define BUZZER_DEFAULT_FREQ_HZ 2000U
#define BUZZER_MIN_FREQ_HZ      200U
#define BUZZER_MAX_FREQ_HZ     5000U
#define BUZZER_TIM_CLOCK_HZ   1000000U    /* 170MHz / 170 = 1MHz */

typedef struct {
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t  repeat;
} BuzzerPatternStep;

static const BuzzerPatternStep s_patterns[] = {
    [BUZZER_PATTERN_NONE]     = {  0U,   0U, 0U},
    [BUZZER_PATTERN_OK]       = { 80U,   0U, 1U},
    [BUZZER_PATTERN_ERROR]    = {120U, 120U, 3U},
    [BUZZER_PATTERN_START]    = { 60U,  80U, 2U},
    [BUZZER_PATTERN_OBSTACLE] = { 50U,  50U, 5U},
};

/* ── 内部状态 ── */

static BuzzerPatternStep s_active_pat;       /* 当前播放的模式     */
static uint32_t          s_next_change_ms;   /* 下次 on/off 切换时刻 */
static uint16_t          s_frequency_hz;     /* 当前频率           */
static uint8_t           s_remaining;        /* 剩余重复次数       */
static volatile bool     s_active;           /* 定时器是否在运行   */
static volatile bool     s_gpio_high;        /* 当前 PB10 电平     */
static bool              s_envelope_on;      /* 当前处于 on 阶段   */

/* ── 硬件层 ── */

static void gpio_write(bool high)
{
    s_gpio_high = high;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t freq_to_arr(uint16_t hz)
{
    if      (hz < BUZZER_MIN_FREQ_HZ) hz = BUZZER_MIN_FREQ_HZ;
    else if (hz > BUZZER_MAX_FREQ_HZ) hz = BUZZER_MAX_FREQ_HZ;
    uint32_t half = BUZZER_TIM_CLOCK_HZ / ((uint32_t)hz * 2U);
    if (half == 0U) half = 1U;
    return half - 1U;
}

static void timer_set_arr(uint16_t hz)
{
    __HAL_TIM_SET_AUTORELOAD(&htim7, freq_to_arr(hz));
}

/* ── 发声 / 静音 ── */

static void tone_on(void)
{
    s_envelope_on = true;

    if (s_active)
    {
        /* 已经在运行，只改参数不重启定时器，避免波形中断 */
        timer_set_arr(s_frequency_hz);
        return;
    }

    s_active = true;
    timer_set_arr(s_frequency_hz);
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);
}

static void tone_off(void)
{
    s_envelope_on = false;
    s_active      = false;

    HAL_TIM_Base_Stop_IT(&htim7);
    gpio_write(false);
}

/* ── 初始化 ── */

void BspBuzzer_Init(void)
{
    s_active_pat      = s_patterns[BUZZER_PATTERN_NONE];
    s_next_change_ms  = 0U;
    s_frequency_hz    = BUZZER_DEFAULT_FREQ_HZ;
    s_remaining       = 0U;
    s_active          = false;
    s_envelope_on     = false;
    gpio_write(false);
    timer_set_arr(s_frequency_hz);
    /* TIM7 不启动，等 Play 时再启动 */
}

/* ── 控制 ── */

void BspBuzzer_Set(bool on)
{
    s_active_pat = s_patterns[BUZZER_PATTERN_NONE];
    s_remaining  = 0U;
    s_next_change_ms = 0U;
    if (on) tone_on(); else tone_off();
}

void BspBuzzer_SetFrequency(uint16_t hz)
{
    if      (hz < BUZZER_MIN_FREQ_HZ) hz = BUZZER_MIN_FREQ_HZ;
    else if (hz > BUZZER_MAX_FREQ_HZ) hz = BUZZER_MAX_FREQ_HZ;
    s_frequency_hz = hz;
    if (s_active) {
        timer_set_arr(hz);
        __HAL_TIM_SET_COUNTER(&htim7, 0U);
        __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    }
}

void BspBuzzer_Beep(uint16_t on_ms)
{
    s_active_pat.on_ms  = on_ms;
    s_active_pat.off_ms = 0U;
    s_active_pat.repeat = 1U;
    s_remaining = 1U;
    tone_on();
    s_next_change_ms = HAL_GetTick() + on_ms;
}

void BspBuzzer_Play(BuzzerPattern pattern)
{
    if ((uint32_t)pattern >= (sizeof(s_patterns) / sizeof(s_patterns[0]))) return;
    s_active_pat = s_patterns[pattern];
    s_remaining  = s_active_pat.repeat;
    if (s_remaining == 0U) { tone_off(); return; }
    tone_on();
    s_next_change_ms = HAL_GetTick() + s_active_pat.on_ms;
}

/* ── 10ms 状态机 ── */

void BspBuzzer_Task10ms(void)
{
    uint32_t now = HAL_GetTick();
    if (s_remaining == 0U) return;
    if ((int32_t)(now - s_next_change_ms) < 0) return;

    if (s_envelope_on) {
        /* on → off */
        tone_off();
        if (s_active_pat.off_ms == 0U) { s_remaining = 0U; }
        else { s_next_change_ms = now + s_active_pat.off_ms; }
    } else {
        /* off → on */
        --s_remaining;
        if (s_remaining == 0U) { tone_off(); }
        else { tone_on(); s_next_change_ms = now + s_active_pat.on_ms; }
    }
}

/* ── HAL 回调 ── */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7) return;
    if (!s_active) { gpio_write(false); return; }
    /* 翻转 PB10 → 方波 → 发声 */
    gpio_write(!s_gpio_high);
}

bool BspBuzzer_IsActive(void)
{
    return s_active || (s_remaining > 0U);
}
