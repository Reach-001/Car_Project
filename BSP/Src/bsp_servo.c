#include "bsp_servo.h"

#include "tim.h"     /* htim1, TIM_CHANNEL_3 */

/* ────────────────────────────────────────────────────────────
 * 舵机 PWM 驱动实现
 *
 * TIM1 配置：Prescaler=169, Period=19999
 *           170MHz / 170 / 20000 = 50Hz
 * 脉宽范围：1000us ~ 2000us（对应 CCR 1000~2000）
 * 中位 1500us，半幅 500us
 * ──────────────────────────────────────────────────────────── */

#define SERVO_CENTER_US 1500            /* 中位脉宽 1.5ms            */
#define SERVO_RANGE_US  500             /* 半幅范围 0.5ms            */
#define SERVO_MIN_US    (SERVO_CENTER_US - SERVO_RANGE_US)  /* 1000us */
#define SERVO_MAX_US    (SERVO_CENTER_US + SERVO_RANGE_US)  /* 2000us */

static int16_t s_steer_permille;        /* 当前转角（千分比） */
static bool    s_available;             /* TIM1 初始化是否成功 */

/* 千分比（-1000~1000）→ TIM1 CCR 比较值（脉宽 us） */
static uint32_t steer_to_compare(int16_t steer_permille)
{
    int32_t pulse_us;

    /* 软件限幅 */
    if (steer_permille > 1000)  { steer_permille = 1000; }
    else if (steer_permille < -1000) { steer_permille = -1000; }

    /* pulse_us = 1500 + steer × 500 / 1000 */
    pulse_us = SERVO_CENTER_US + ((int32_t)steer_permille * SERVO_RANGE_US) / 1000;

    /* 硬件绝对安全限幅（即使上层计算错误，BSP 层也保护舵机） */
    if      (pulse_us < SERVO_MIN_US) { pulse_us = SERVO_MIN_US; }
    else if (pulse_us > SERVO_MAX_US) { pulse_us = SERVO_MAX_US; }

    return (uint32_t)pulse_us;
}

/* ── 初始化 ── */

bool BspServo_Init(void)
{
    s_steer_permille = 0;

    /* 启动 TIM1 CH3 PWM */
    s_available = (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) == HAL_OK);

    /* 默认归中位 */
    BspServo_SetSteerPermille(0);
    return s_available;
}

bool BspServo_IsAvailable(void)
{
    return s_available;
}

/* ── 驱动接口 ── */

void BspServo_SetSteerPermille(int16_t steer_permille)
{
    /* 软件限幅（第一层） */
    if      (steer_permille > 1000)  { steer_permille = 1000; }
    else if (steer_permille < -1000) { steer_permille = -1000; }

    s_steer_permille = steer_permille;

    /* 舵机不可用时跳过写寄存器 */
    if (s_available)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                              steer_to_compare(s_steer_permille));
    }
}
