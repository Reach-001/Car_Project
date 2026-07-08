#include "bsp_buzzer.h"

#include "main.h"              /* Buzzer_Pin, Buzzer_GPIO_Port */
#include "stm32g4xx_hal.h"     /* HAL_TIM_*, __HAL_TIM_*, HAL_GPIO_* */
#include "tim.h"               /* htim7（CubeMX 生成） */

/* ────────────────────────────────────────────────────────────
 * 无源蜂鸣器驱动实现
 *
 * TIM7 由 CubeMX IOC 管理（Prescaler=169, Period=249, 默认 2000Hz）
 * HAL_TIM_PeriodElapsedCallback → 翻转 PB10 → 产生方波
 *
 * 频率公式：(170MHz / (169+1)) / (Period+1) / 2 = 1MHz / (Period+1) / 2
 *   200Hz → Period = 2499
 *  2000Hz → Period = 249
 *  5000Hz → Period = 99
 *
 * 运行时改频率：HAL 停定时器 → 写新 ARR → 清计数器 → HAL 启定时器
 *
 * 提示音状态机：on_ms 发声 → off_ms 静音 → 重复 repeat 次。
 * 由 BspBuzzer_Task10ms() 每 10ms 驱动切换。
 * ──────────────────────────────────────────────────────────── */

#define BUZZER_DEFAULT_FREQ_HZ 2000U        /* 默认频率                */
#define BUZZER_MIN_FREQ_HZ     200U         /* 最低频率（避免人耳不可闻） */
#define BUZZER_MAX_FREQ_HZ     5000U        /* 最高频率（避免刺耳）     */
#define BUZZER_TIM_CLOCK_HZ    1000000U     /* TIM7 计数频率 = 170M/170 = 1MHz */

/* 提示音模式：一个 on/off 周期 */
typedef struct
{
    uint16_t on_ms;      /* 发声时长（ms） */
    uint16_t off_ms;     /* 静音时长（ms） */
    uint8_t  repeat;     /* 重复次数       */
} BuzzerPatternStep;

/* 预设模式表（索引 = BuzzerPattern 枚举值） */
static const BuzzerPatternStep s_patterns[] = {
    [BUZZER_PATTERN_NONE]     = {0U,   0U,   0U},
    [BUZZER_PATTERN_OK]       = {80U,  0U,   1U},
    [BUZZER_PATTERN_ERROR]    = {120U, 120U, 3U},
    [BUZZER_PATTERN_START]    = {60U,  80U,  2U},
    [BUZZER_PATTERN_OBSTACLE] = {50U,  50U,  5U},
};

/* ── 模块内部状态（全部 static） ── */

static BuzzerPatternStep s_active;           /* 当前正播放的模式参数    */
static uint32_t          s_next_change_ms;   /* 下一次 on/off 切换时刻 */
static uint16_t          s_frequency_hz;     /* 当前频率               */
static uint8_t           s_remaining;        /* 剩余重复次数           */
static volatile bool     s_gpio_high;        /* 当前 PB10 电平         */
static bool              s_envelope_on;      /* 当前处于发声阶段       */

/* ── 内部辅助函数 ── */

/** 写 PB10 GPIO */
static void buzzer_gpio_write(bool high)
{
    s_gpio_high = high;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** 频率 → TIM7 ARR 值（半周期-1）
 *  period = TIM_CLK / (freq × 2) - 1 */
static uint32_t frequency_to_period(uint16_t frequency_hz)
{
    if      (frequency_hz < BUZZER_MIN_FREQ_HZ) { frequency_hz = BUZZER_MIN_FREQ_HZ; }
    else if (frequency_hz > BUZZER_MAX_FREQ_HZ) { frequency_hz = BUZZER_MAX_FREQ_HZ; }

    uint32_t half_period = BUZZER_TIM_CLOCK_HZ / ((uint32_t)frequency_hz * 2U);
    if (half_period == 0U) { half_period = 1U; }
    return half_period - 1U;
}

/** 配置 TIM7 ARR 为指定频率的周期值 */
static void buzzer_set_arr(uint16_t frequency_hz)
{
    uint32_t new_arr = frequency_to_period(frequency_hz);
    __HAL_TIM_SET_AUTORELOAD(&htim7, new_arr);
}

/** 开始发声：启动 TIM7 中断 */
static void tone_start(void)
{
    s_envelope_on = true;

    /* 先停再配：停定时器 → 设新频率 ARR → 清计数器 → 清标志 → 启动 */
    HAL_TIM_Base_Stop_IT(&htim7);
    buzzer_set_arr(s_frequency_hz);
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7);
}

/** 停止发声：停定时器 + 拉低 GPIO */
static void tone_stop(void)
{
    s_envelope_on = false;
    HAL_TIM_Base_Stop_IT(&htim7);
    buzzer_gpio_write(false);
}

/* ── 初始化 ── */

void BspBuzzer_Init(void)
{
    /* TIM7 的时钟、NVIC、基本参数由 CubeMX MX_TIM7_Init() 完成。
     * 这里只做模块内部状态复位，不操作 RCC/NVIC。 */

    s_active = s_patterns[BUZZER_PATTERN_NONE];
    s_next_change_ms = 0U;
    s_frequency_hz   = BUZZER_DEFAULT_FREQ_HZ;
    s_remaining      = 0U;
    s_envelope_on    = false;
    buzzer_gpio_write(false);

    /* 预载默认频率的 ARR（不启动定时器，等 tone_start 时再启动） */
    buzzer_set_arr(s_frequency_hz);
}

/* ── 控制接口 ── */

void BspBuzzer_Set(bool on)
{
    s_active = s_patterns[BUZZER_PATTERN_NONE];
    s_remaining = 0U;
    s_next_change_ms = 0U;

    if (on) { tone_start(); }
    else    { tone_stop();  }
}

void BspBuzzer_SetFrequency(uint16_t frequency_hz)
{
    if      (frequency_hz < BUZZER_MIN_FREQ_HZ) { frequency_hz = BUZZER_MIN_FREQ_HZ; }
    else if (frequency_hz > BUZZER_MAX_FREQ_HZ) { frequency_hz = BUZZER_MAX_FREQ_HZ; }

    s_frequency_hz = frequency_hz;

    /* 如果正在发声，立即更新频率 */
    if (s_envelope_on)
    {
        buzzer_set_arr(s_frequency_hz);
        __HAL_TIM_SET_COUNTER(&htim7, 0U);
        __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    }
}

void BspBuzzer_Beep(uint16_t on_ms)
{
    s_active.on_ms   = on_ms;
    s_active.off_ms  = 0U;
    s_active.repeat  = 1U;
    s_remaining      = 1U;
    tone_start();
    s_next_change_ms = HAL_GetTick() + on_ms;
}

void BspBuzzer_Play(BuzzerPattern pattern)
{
    if ((uint32_t)pattern >= (sizeof(s_patterns) / sizeof(s_patterns[0])))
    {
        return;
    }

    s_active    = s_patterns[pattern];
    s_remaining = s_active.repeat;

    if (s_remaining == 0U) { tone_stop(); return; }

    tone_start();
    s_next_change_ms = HAL_GetTick() + s_active.on_ms;
}

/* ── 10ms 周期状态机 ── */

void BspBuzzer_Task10ms(void)
{
    uint32_t now = HAL_GetTick();

    if (s_remaining == 0U) { return; }

    if ((int32_t)(now - s_next_change_ms) < 0) { return; }

    if (s_envelope_on)
    {
        /* 发声 → 静音 */
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
        /* 静音 → 下一轮发声 */
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

/* ── HAL 定时器回调：TIM7 溢出时 HAL 自动调用 ── */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 只处理 TIM7（CubeMX 可能会注册多个定时器） */
    if (htim->Instance == TIM7)
    {
        /* 翻转 PB10 → 产生方波 → 蜂鸣器发声 */
        buzzer_gpio_write(!s_gpio_high);
    }
}

/* ── 状态查询 ── */

bool BspBuzzer_IsActive(void)
{
    return s_envelope_on || (s_remaining > 0U);
}
