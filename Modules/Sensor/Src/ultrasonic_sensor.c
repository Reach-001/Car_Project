/* ────────────────────────────────────────────────────────────
 * 超声波传感器子模块（Sensor 域内部使用，不对外暴露头文件）
 *
 * 工作原理：TRIG 发 10us 高脉冲 → ECHO 回波高电平脉宽 →
 *         距离(mm) = 脉宽(us) × 声速(mm/ms) / 2000
 *                = 脉宽(us) × 343 / 2000
 *
 * 周期触发：每 60ms 自动发一次 TRIG。
 * 超时保护：30ms 无回波 → 标记无效。
 * 障碍判定：距离 < 300mm → obstacle_near=true。
 *
 * 参数说明：
 *   TRIGGER_INTERVAL_MS = TRIG 自动触发间隔（ms）。
 *                         HC-SR04 手册建议 ≥60ms 等待上次回波完成。
 *   ECHO_TIMEOUT_US     = 回波超时时间（us）。
 *                         30ms = 30000us，对应约 5m 测距范围。
 *                         超时则本次测量失败，下一周期重新触发。
 *   SOUND_SPEED_MM_PER_US_X1000 = 声速 ×1000（mm/s）。
 *                         343m/s = 343000mm/s = 0.343mm/us。
 *                         乘 1000 避免浮点，计算时除以 2000 得到距离。
 *   OBSTACLE_NEAR_MM    = 障碍物判定阈值（mm）。距离 < 300mm 则认为有障碍。
 * ──────────────────────────────────────────────────────────── */

#include "main.h"
#include "sensor_internal.h"
#include "stm32g4xx_hal.h"
#include "system_state_pool.h"

/* ════════════════════════════════════════════════════════════
 * 超声波参数
 * ════════════════════════════════════════════════════════════ */

#define TRIGGER_INTERVAL_MS             60U         /* 自动触发间隔（ms），≥60ms 避重触发 */
#define ECHO_TIMEOUT_US                 30000U      /* 回波超时（us），30ms≈5m 测距范围    */
#define SOUND_SPEED_MM_PER_US_X1000     343U        /* 声速×1000（mm/s），0.343mm/us       */
#define OBSTACLE_NEAR_MM                300U        /* 障碍判定阈值（mm），<300mm 告警      */

/* ── 内部状态 ── */

static volatile uint32_t s_echo_start_us;           /* ECHO 上升沿时刻（us）   */
static volatile uint32_t s_echo_width_us;           /* ECHO 高电平宽度（us）   */
static volatile bool     s_echo_done;               /* 本次测量完成标志        */
static volatile bool     s_waiting_echo;            /* 正在等待回波标志        */
static uint32_t          s_triggered_us;            /* 本次触发时刻（us）      */
static uint32_t          s_last_trigger_ms;         /* 上次触发时刻（ms）      */
static uint16_t          s_distance_mm;             /* 最新距离（mm）          */
static bool              s_valid;                   /* 数据有效标志            */

/* ── DWT 微秒时钟（DWT = Data Watchpoint and Trace，CPU 调试器自带的时钟计数器） ── */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t micros(void)
{
    uint32_t cpm = SystemCoreClock / 1000000U;      /* CPU 周期数 / us */
    if (cpm == 0U) cpm = 1U;
    return DWT->CYCCNT / cpm;
}

static void delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) {}
}

/* ── 懒初始化：首次调用 Task10ms 时初始化 DWT ── */

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

/* ── TRIG 触发：拉高 10us → 拉低 → 等待 ECHO 回波 ── */

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

/* ── EXTI 回调：ECHO 电平变化时被 HAL 调用 ── */

void Ultrasonic_OnEchoEdge(void)
{
    uint32_t now_us = micros();
    GPIO_PinState echo = HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin);
    if (echo == GPIO_PIN_SET) { s_echo_start_us = now_us; }         /* 上升沿 → 记录开始 */
    else if (s_echo_start_us != 0U) {                               /* 下降沿 → 计算脉宽 */
        s_echo_width_us = now_us - s_echo_start_us;
        s_echo_done     = true;
        s_waiting_echo  = false;
    }
}

/* ── 10ms 周期任务 ── */

void Ultrasonic_Task10ms(void)
{
    ultrasonic_init_internal();
    uint32_t now_ms = HAL_GetTick();

    /* 回波完成 → 计算距离 */
    if (s_echo_done) {
        uint32_t echo = s_echo_width_us;
        s_echo_done   = false;
        s_distance_mm = (uint16_t)((echo * SOUND_SPEED_MM_PER_US_X1000) / 2000U);
        s_valid       = true;
    }

    /* 超时 → 标记无效 */
    if (s_waiting_echo && ((uint32_t)(micros() - s_triggered_us) > ECHO_TIMEOUT_US)) {
        s_waiting_echo = false;
        s_valid        = false;
    }

    /* 空闲且距上次触发 ≥ TRIGGER_INTERVAL_MS → 发新脉冲 */
    if (!s_waiting_echo && ((uint32_t)(now_ms - s_last_trigger_ms) >= TRIGGER_INTERVAL_MS)) {
        trigger();
    }
}

/* ── 写入状态池 ── */

void Ultrasonic_WriteToPool(SystemStatePool *pool)
{
    if (pool == 0) return;
    pool->sensor.ultrasonic_mm    = s_distance_mm;
    pool->sensor.ultrasonic_valid = s_valid;
    pool->sensor.obstacle_near    = s_valid && (s_distance_mm < OBSTACLE_NEAR_MM);
}

/* ── HAL EXTI 回调（全局唯一） ── */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == HCSR04_ECHO_Pin) { Ultrasonic_OnEchoEdge(); }
}
