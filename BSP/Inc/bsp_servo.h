#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * 舵机 PWM 驱动 —— 硬件抽象层头文件
 *
 * 硬件：TIM1 CH3 → PA10，50Hz PWM（周期 20ms）
 *       1000us = 最左极限，1500us = 中位，2000us = 最右极限
 *
 * 接口使用千分比（-1000~1000），内部映射为脉宽。
 * 换芯片移植：只改 bsp_servo.c，本文件不用动。
 * ──────────────────────────────────────────────────────────── */

/** 初始化舵机 PWM，启动 TIM1 CH3，归中位 */
bool BspServo_Init(void);

/** 舵机是否可用（TIM1 初始化成功） */
bool BspServo_IsAvailable(void);

/** 设置舵机转角（千分比）
 *  @param steer_permille 转角千分比：
 *      -1000 = 1000us（最左）
 *         0  = 1500us（中位）
 *      1000  = 2000us（最右）
 *  超出范围自动截断。实车需要标定机械中位和左右极限。 */
void BspServo_SetSteerPermille(int16_t steer_permille);

#endif /* BSP_SERVO_H */
