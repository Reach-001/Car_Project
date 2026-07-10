/* ────────────────────────────────────────────────────────────
 * 无源蜂鸣器驱动实现
 *
 * 定时器：TIM7（CubeMX IOC 管理），Prescaler=169, Period=249
 *   170MHz / 170 = 1MHz 计数频率
 *   1MHz / (250×2) = 2000Hz 方波
 *
 * 频率公式：170MHz / (Prescaler+1) / (Period+1) / 2
 * 频率表（给定 Period 对应的频率）：
 *   Period=2499 → 200Hz   （人耳可闻下限）
 *   Period=249  → 2000Hz   （默认频率）
 *   Period=99   → 5000Hz   （刺耳上限）
 *
 * ISR 回调 HAL_TIM_PeriodElapsedCallback → 翻转 PB10 GPIO → 方波 → 发声。
 * 提示音状态机由 BspBuzzer_Task10ms() 每 10ms 驱动切换 on/off。
 *
 * 参数说明：
 *   BUZZER_DEFAULT_FREQ_HZ = 默认鸣响频率（Hz）。2000Hz 是较舒适的提示音
 *   BUZZER_MIN_FREQ_HZ     = 最低允许频率。低于 200Hz 人耳基本听不见
 *   BUZZER_MAX_FREQ_HZ     = 最高允许频率。高于 5000Hz 刺耳且蜂鸣器可能不响应
 *   BUZZER_TIM_CLOCK_HZ    = TIM7 计数频率。170MHz / (169+1) = 1MHz
 * ──────────────────────────────────────────────────────────── */

#include "bsp_buzzer.h"

#include "main.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

/* ════════════════════════════════════════════════════════════
 * 频率参数
 * ════════════════════════════════════════════════════════════ */

#define BUZZER_DEFAULT_FREQ_HZ 2000U            /* 默认鸣响频率（Hz）            */
#define BUZZER_MIN_FREQ_HZ      200U            /* 最低频率（Hz），听阈下限        */
#define BUZZER_MAX_FREQ_HZ     5000U            /* 最高频率（Hz），防刺耳保护      */
#define BUZZER_TIM_CLOCK_HZ   1000000U           /* TIM7 计数频率 = 170M/170 = 1MHz */

/* ════════════════════════════════════════════════════════════
 * 预设提示音模式表 —— 索引 = BuzzerPattern 枚举值
 *   on_ms   = 发声时间（ms）
 *   off_ms  = 静音时间（ms）
 *   repeat  = 重复次数
 * BUZZER_PATTERN_NONE      = 静音
 * BUZZER_PATTERN_OK        = 短"嘀"一声 80ms
 * BUZZER_PATTERN_ERROR     = "嘀-嘀-嘀"三声，每声 120ms 间隔 120ms
 * BUZZER_PATTERN_START     = "嘀嘀"两声，每声 60ms 间隔 80ms
 * BUZZER_PATTERN_OBSTACLE  = "嘀嘀嘀嘀嘀"五声，每声 50ms 间隔 50ms（急促）
 * ════════════════════════════════════════════════════════════ */

typedef struct {
    uint16_t on_ms;        /* 单次发声时长（ms）  */
    uint16_t off_ms;       /* 单次静音间隔（ms） */
    uint8_t  repeat;       /* 重复次数           */
} BuzzerPatternStep;

static const BuzzerPatternStep s_patterns[] = {
    [BUZZER_PATTERN_NONE]     = {  0U,   0U, 0U},
    [BUZZER_PATTERN_OK]       = { 80U,   0U, 1U},
    [BUZZER_PATTERN_ERROR]    = {120U, 120U, 3U},
    [BUZZER_PATTERN_START]    = { 60U,  80U, 2U},
    [BUZZER_PATTERN_OBSTACLE] = { 50U,  50U, 5U},
};

/* ── 模块内部状态（全部 static） ── */

static BuzzerPatternStep s_active;
static uint32_t          s_next_change_ms;  /* 下次 on/off 切换时刻（ms） */
static uint16_t          s_frequency_hz;    /* 当前频率（Hz）              */
static uint8_t           s_remaining;       /* 剩余重复次数                */
static volatile bool     s_active_run;      /* 定时器是否在运行（ISR 读）  */
static volatile bool     s_gpio_high;       /* 当前 PB10 电平（ISR 读）   */
static bool              s_envelope_on;     /* 当前处于 on 阶段             */

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
    if (s_active_run) { timer_set_arr(s_frequency_hz); return; }  /* 已运行只改频率 */
    s_active_run = true;
    timer_set_arr(s_frequency_hz);
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);
}

static void tone_off(void)
{
    s_envelope_on = false;
    s_active_run  = false;
    HAL_TIM_Base_Stop_IT(&htim7);
    gpio_write(false);
}

/* ── 初始化 ── */

void BspBuzzer_Init(void)
{
    s_active          = s_patterns[BUZZER_PATTERN_NONE];
    s_next_change_ms  = 0U;
    s_frequency_hz    = BUZZER_DEFAULT_FREQ_HZ;
    s_remaining       = 0U;
    s_active_run      = false;
    s_envelope_on     = false;
    gpio_write(false);
    timer_set_arr(s_frequency_hz);
}

/* ── 控制接口 ── */

void BspBuzzer_Set(bool on)
{
    s_active = s_patterns[BUZZER_PATTERN_NONE];
    s_remaining = 0U;
    s_next_change_ms = 0U;
    if (on) tone_on(); else tone_off();
}

void BspBuzzer_SetFrequency(uint16_t hz)
{
    if      (hz < BUZZER_MIN_FREQ_HZ) hz = BUZZER_MIN_FREQ_HZ;
    else if (hz > BUZZER_MAX_FREQ_HZ) hz = BUZZER_MAX_FREQ_HZ;
    s_frequency_hz = hz;
    if (s_active_run) { timer_set_arr(hz); __HAL_TIM_SET_COUNTER(&htim7, 0U); __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE); }
}

void BspBuzzer_Beep(uint16_t on_ms)
{
    s_active.on_ms  = on_ms;
    s_active.off_ms = 0U;
    s_active.repeat = 1U;
    s_remaining = 1U;
    tone_on();
    s_next_change_ms = HAL_GetTick() + on_ms;
}

void BspBuzzer_Play(BuzzerPattern pattern)
{
    if ((uint32_t)pattern >= (sizeof(s_patterns) / sizeof(s_patterns[0]))) return;
    s_active    = s_patterns[pattern];
    s_remaining = s_active.repeat;
    if (s_remaining == 0U) { tone_off(); return; }
    tone_on();
    s_next_change_ms = HAL_GetTick() + s_active.on_ms;
}

/* ── 10ms 状态机 ── */

void BspBuzzer_Task10ms(void)
{
    uint32_t now = HAL_GetTick();
    if (s_remaining == 0U) return;
    if ((int32_t)(now - s_next_change_ms) < 0) return;

    if (s_envelope_on) {
        tone_off();
        if (s_active.off_ms == 0U) { s_remaining = 0U; }
        else { s_next_change_ms = now + s_active.off_ms; }
    } else {
        --s_remaining;
        if (s_remaining == 0U) { tone_off(); }
        else { tone_on(); s_next_change_ms = now + s_active.on_ms; }
    }
}

/* ── HAL 回调 ── */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7) return;
    if (!s_active_run) { gpio_write(false); return; }
    gpio_write(!s_gpio_high);
}

bool BspBuzzer_IsActive(void)
{
    return s_active_run || (s_remaining > 0U);
}
