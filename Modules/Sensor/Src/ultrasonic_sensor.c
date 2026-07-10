/* ────────────────────────────────────────────────────────────
 * 超声波传感器子模块（Sensor 域内部使用，不对外暴露头文件）
 *
 * 迁移自旧 Modules/Src/ultrasonic.c
 * 依赖：BSP/bsp_gpio_sensor.h
 *
 * 工作原理：TRIG 发 10us 高脉冲 → ECHO 回波高电平脉宽 →
 *         距离 = 脉宽(us) × 343 / 2000 (mm)
 *
 * EXTI 双边沿 ISR（在 Core/Src/stm32g4xx_it.c 中）调用
 * HAL_GPIO_EXTI_Callback → Ultrasonic_OnEchoEdge()
 *
 * 周期触发：每 60ms 自动发一次 TRIG 脉冲。
 * 超时：30ms 无回波 → 标记无效。
 * ──────────────────────────────────────────────────────────── */

#include "main.h"
#include "sensor_internal.h"
#include "stm32g4xx_hal.h"
#include "system_state_pool.h"

#define TRIGGER_INTERVAL_MS  60U
#define ECHO_TIMEOUT_US      30000U
#define SOUND_SPEED_MM_PER_US_X1000  343U

/* ── 内部状态 ── */

static volatile uint32_t s_echo_start_us;
static volatile uint32_t s_echo_width_us;
static volatile bool     s_echo_done;
static volatile bool     s_waiting_echo;
static uint32_t          s_triggered_us;
static uint32_t          s_last_trigger_ms;
static uint16_t          s_distance_mm;
static bool              s_valid;

/* ── DWT 微秒时钟 ── */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t micros(void)
{
    uint32_t cpm = SystemCoreClock / 1000000U;
    if (cpm == 0U) cpm = 1U;
    return DWT->CYCCNT / cpm;
}

static void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) {}
}

/* ── Ultrosonic_Init（内部，由 Sensor_Init 间接调用） ── */

void Ultrasonic_Task10ms(void) __attribute__((unused));

/* 模块初始化：由 Ultrasonic_Task10ms 首次调用时懒初始化 */
static bool s_dwt_inited;

static void ultrasonic_init_internal(void)
{
    if (s_dwt_inited) return;
    s_dwt_inited = true;

    dwt_init();
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    s_echo_start_us   = 0U;
    s_echo_width_us   = 0U;
    s_echo_done       = false;
    s_waiting_echo    = false;
    s_triggered_us    = 0U;
    s_last_trigger_ms = 0U;
    s_distance_mm     = 0U;
    s_valid           = false;
}

/* ── TRIG 触发 ── */

static void trigger(void)
{
    if (s_waiting_echo) return;

    s_echo_done    = false;
    s_waiting_echo = true;
    s_triggered_us = micros();

    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    delay_us(10U);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
    s_last_trigger_ms = HAL_GetTick();
}

/* ── EXTI 回调 ── */

void Ultrasonic_OnEchoEdge(void)
{
    uint32_t now_us = micros();
    GPIO_PinState echo = HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin);

    if (echo == GPIO_PIN_SET)
    {
        s_echo_start_us = now_us;
    }
    else
    {
        if (s_echo_start_us != 0U)
        {
            s_echo_width_us = now_us - s_echo_start_us;
            s_echo_done     = true;
            s_waiting_echo  = false;
        }
    }
}

/* ── 周期任务 ── */

void Ultrasonic_Task10ms(void)
{
    ultrasonic_init_internal();

    uint32_t now_ms = HAL_GetTick();

    /* 处理完成测量 */
    if (s_echo_done)
    {
        uint32_t echo = s_echo_width_us;
        s_echo_done    = false;
        s_distance_mm  = (uint16_t)((echo * SOUND_SPEED_MM_PER_US_X1000) / 2000U);
        s_valid        = true;
    }

    /* 超时处理 */
    if (s_waiting_echo && ((uint32_t)(micros() - s_triggered_us) > ECHO_TIMEOUT_US))
    {
        s_waiting_echo = false;
        s_valid        = false;
    }

    /* 自动触发 */
    if (!s_waiting_echo && ((uint32_t)(now_ms - s_last_trigger_ms) >= TRIGGER_INTERVAL_MS))
    {
        trigger();
    }
}

/* ── 写入状态池 ── */

void Ultrasonic_WriteToPool(SystemStatePool *pool)
{
    if (pool == 0) return;

    pool->sensor.ultrasonic_mm    = s_distance_mm;
    pool->sensor.ultrasonic_valid = s_valid;
    pool->sensor.obstacle_near    = s_valid && (s_distance_mm < 300U);
}

/* ── HAL EXTI 回调（全局唯一，由 stm32g4xx_it.c 中间接调用） ── */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == HCSR04_ECHO_Pin)
    {
        Ultrasonic_OnEchoEdge();
    }
}
