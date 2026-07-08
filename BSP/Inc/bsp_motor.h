#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * 电机 PWM 驱动 —— 硬件抽象层头文件（不暴露 HAL 类型）
 *
 * 硬件：TIM3 CH1/CH2 = 左电机正反转，CH3/CH4 = 右电机正反转
 *       频率 20kHz，PWM 分辨率为 duty 千分比（-1000 ~ 1000）
 *
 * 换芯片移植：只需修改 bsp_motor.c 的实现，本头文件不用动
 * ──────────────────────────────────────────────────────────── */

/* 电机枚举：物理上只有左/右两个电机 */
typedef enum
{
    BSP_MOTOR_LEFT  = 0,   /* 左电机：TIM3 CH1(正转) / CH2(反转) */
    BSP_MOTOR_RIGHT         /* 右电机：TIM3 CH3(正转) / CH4(反转) */
} BspMotorId;

/* ── 生命周期 ── */

/** 初始化电机 PWM 输出，启动 TIM3 四个通道，然后调用 StopAll 归零 */
void BspMotor_Init(void);

/* ── 控制接口 ── */

/** 设置指定电机的占空比
 *  @param motor         目标电机（LEFT / RIGHT）
 *  @param duty_permille 占空比千分比：
 *                          正数 = 正转（0~1000）
 *                          负数 = 反转（-1000~0）
 *                          0    = 停止 */
void BspMotor_SetDuty(BspMotorId motor, int16_t duty_permille);

/** 紧急停止：左右电机同时归零 */
void BspMotor_StopAll(void);

#endif /* BSP_MOTOR_H */
