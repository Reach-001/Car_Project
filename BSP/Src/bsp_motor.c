#include "bsp_motor.h"

#include "tim.h"     /* htim3, TIM_CHANNEL_x 宏 */

/* ────────────────────────────────────────────────────────────
 * 电机 PWM 驱动实现
 *
 * 左电机：PA6(TIM3_CH1) 正转 / PA7(TIM3_CH2) 反转
 * 右电机：PB0(TIM3_CH3) 正转 / PB1(TIM3_CH4) 反转
 *
 * TIM3 频率：170MHz / (16+1) / (499+1) = 20kHz
 * duty 计算：duty_permille × ARR / 1000 = duty × 499 / 1000
 * ──────────────────────────────────────────────────────────── */

#define MOTOR_DUTY_MAX   1000    /* 千分比上限（对应 100% 占空比） */
#define MOTOR_TIM_PERIOD 499U    /* TIM3 ARR 值（CubeMX 配置）     */

/* 将千分比 duty 转换为 TIM3 CCR 比较值 */
static uint32_t duty_to_compare(int16_t duty_permille)
{
    int32_t duty = duty_permille;

    /* 取绝对值 */
    if (duty < 0)
    {
        duty = -duty;
    }
    if (duty > MOTOR_DUTY_MAX)
    {
        duty = MOTOR_DUTY_MAX;
    }

    return (uint32_t)((duty * (int32_t)MOTOR_TIM_PERIOD) / MOTOR_DUTY_MAX);
}

/* ── 初始化 ── */

void BspMotor_Init(void)
{
    /* 启动四路 PWM 输出 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* PA6 左电机正转 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* PA7 左电机反转 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   /* PB0 右电机正转 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   /* PB1 右电机反转 */

    /* 上电默认停止 */
    BspMotor_StopAll();
}

/* ── 驱动接口 ── */

void BspMotor_SetDuty(BspMotorId motor, int16_t duty_permille)
{
    uint32_t forward = 0U;                  /* 正向通道 CCR 值 */
    uint32_t reverse = 0U;                  /* 反向通道 CCR 值 */
    uint32_t compare = duty_to_compare(duty_permille);

    /* 正数 → 正向通道输出；负数 → 反向通道输出 */
    if (duty_permille >= 0)
    {
        forward = compare;
    }
    else
    {
        reverse = compare;
    }

    if (motor == BSP_MOTOR_LEFT)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, forward);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, reverse);
    }
    else  /* BSP_MOTOR_RIGHT */
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, forward);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, reverse);
    }
}

void BspMotor_StopAll(void)
{
    BspMotor_SetDuty(BSP_MOTOR_LEFT,  0);
    BspMotor_SetDuty(BSP_MOTOR_RIGHT, 0);
}
