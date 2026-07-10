#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "system_state_pool.h"

/* ────────────────────────────────────────────────────────────
 * Motion 域 —— 目标速度/转角 → PWM/舵机输出（唯一操作硬件的逻辑域）
 *
 * 执行顺序：
 *   1. 读取 pool->target.speed_mps / steer_angle_rad
 *   2. 目标限幅（软安全）
 *   3. 阿克曼几何：车身速度 → 左右轮目标速度
 *   4. 左右轮速度 PI 闭环
 *   5. PWM 千分比输出 + 限幅
 *   6. 舵机脉宽映射 + 硬限幅
 *   7. 写 BSP Actuator
 *
 * 依赖：BSP/bsp_motor.h、BSP/bsp_servo.h
 * 禁止：调用 Decision、Comm、Sensor、HMI
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    float    target_speed_mps;   /* 目标车身速度     */
    float    target_steer_rad;  /* 目标前轮转角     */
    float    left_target_mps;   /* 左轮目标速度     */
    float    right_target_mps;  /* 右轮目标速度     */
    int16_t  left_pwm;          /* 左轮 PWM 千分比   */
    int16_t  right_pwm;         /* 右轮 PWM 千分比   */
    uint16_t servo_pulse_us;    /* 舵机脉宽 us       */
    bool     limited;           /* 是否达到限幅     */
} MotionState;

/* ── 生命周期 ── */

void Motion_Init(void);

/** 10ms 周期调用：执行运动控制和输出 */
void Motion_Task10ms(SystemStatePool *pool);

MotionState Motion_GetState(void);

/* ── 紧急命令 ── */

typedef struct
{
    bool emergency_brake;       /* 紧急刹车（绕过 Decision） */
} MotionCommand;

void Motion_SetCommand(const MotionCommand *cmd);

#endif /* MOTION_H */
