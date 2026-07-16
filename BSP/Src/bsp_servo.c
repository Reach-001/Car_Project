/* ────────────────────────────────────────────────────────────
 * 舵机 PWM 驱动实现
 *
 * TIM1 参数（CubeMX IOC 配置）：
 *   Prescaler = 169
 *   Period    = 19999
 *   频率      = 170MHz / (169+1) / (19999+1) = 50Hz
 *   周期      = 20ms
 *   分辨率    = 1us/CCR unit（170M/170=1MHz）
 *
 * 脉宽 → CCR 映射：
 *   CCR = pulse_us（计数频率 1MHz，1 CCR = 1us）
 *
 * 舵机标准范围：1000us（最左）～ 1500us（中位）～ 2000us（最右）。
 * 中位、行程、限幅统一从 vehicle_config.h 取，本文件不做硬编码。
 *
 * 安全保护（双层）：
 *   第一层：本函数 steer_to_compare() → ±1000 软件限幅
 *   第二层：vehile_config.h 的 SERVO_MIN_US/MAX_US 硬件硬限制
 *   即使上层 Motion 算出非法值，BSP 层绝不允许输出超出 1000~2000us 的脉冲。
 *
 * 参数说明（来自 vehicle_config.h）：
 *   VEHICLE_SERVO_CENTER_US = 中位脉宽。舵机归中时 = 1500us
 *   VEHICLE_SERVO_RANGE_US  = 半幅行程。±1000‰ 对应 ±500us
 *   VEHICLE_SERVO_MIN_US    = 硬件最小脉宽。舵机绝不超过此值
 *   VEHICLE_SERVO_MAX_US    = 硬件最大脉宽。舵机绝不超过此值
 * ──────────────────────────────────────────────────────────── */

#include "bsp_servo.h"

#include "tim.h"     /* htim1, TIM_CHANNEL_3 */
#include "vehicle_config.h"

/* ════════════════════════════════════════════════════════════
 * 安全限幅参数
 * ════════════════════════════════════════════════════════════ */

#define STEER_PERMILLE_MAX  1000            /* 千分比上限（对应最右极限） */
#define STEER_PERMILLE_MIN -1000            /* 千分比下限（对应最左极限） */

static int16_t s_steer_permille;            /* 当前转角（千分比）          */
static bool    s_available;                 /* TIM1 是否可用               */

/* 千分比 → 脉宽 us → CCR */
static uint32_t steer_to_compare(int16_t steer_permille)
{
    int32_t pulse_us;

    /* 第一层：软件限幅（千分比） */
    if      (steer_permille > STEER_PERMILLE_MAX)  steer_permille = STEER_PERMILLE_MAX;
    else if (steer_permille < STEER_PERMILLE_MIN)  steer_permille = STEER_PERMILLE_MIN;

    /* 千分比 → 脉宽：1500 + steer×500/1000 */
    pulse_us = (int32_t)VEHICLE_SERVO_CENTER_US +
               ((int32_t)steer_permille * (int32_t)VEHICLE_SERVO_RANGE_US) / 1000;

    /* 第二层：硬件绝对安全限幅（即使上层传了错误值也不越界） */
    if      (pulse_us < (int32_t)VEHICLE_SERVO_MIN_US) pulse_us = (int32_t)VEHICLE_SERVO_MIN_US;
    else if (pulse_us > (int32_t)VEHICLE_SERVO_MAX_US) pulse_us = (int32_t)VEHICLE_SERVO_MAX_US;

    return (uint32_t)pulse_us;
}

/* ── 初始化 ── */

bool BspServo_Init(void)
{
    HAL_StatusTypeDef status;

    s_steer_permille = 0;

    if (!s_available)
    {
        status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        /* 正常 App 已启动 PWM 后，运行时 TEST,5 会再次初始化舵机。
         * HAL_BUSY 表示通道已在输出，不能因此把舵机标为不可用。 */
        s_available = (status == HAL_OK) || (status == HAL_BUSY);
    }

    BspServo_SetSteerPermille(0);       /* 上电归中位 */
    return s_available;
}

bool BspServo_IsAvailable(void) { return s_available; }

/* ── 驱动接口 ── */

void BspServo_SetSteerPermille(int16_t steer_permille)
{
    if      (steer_permille > STEER_PERMILLE_MAX)  steer_permille = STEER_PERMILLE_MAX;
    else if (steer_permille < STEER_PERMILLE_MIN)  steer_permille = STEER_PERMILLE_MIN;

    s_steer_permille = steer_permille;
    if (s_available)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                              steer_to_compare(s_steer_permille));
    }
}
