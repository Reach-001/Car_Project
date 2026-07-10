#ifndef SENSOR_DOMAIN_H
#define SENSOR_DOMAIN_H

#include <stdbool.h>
#include <stdint.h>

#include "system_state_pool.h"

/* ────────────────────────────────────────────────────────────
 * Sensor 域 —— 读取物理传感器，输出清洁的物理量
 *
 * 职责：读取循迹、超声波原始信号，完成滤波和有效性判断，
 *       输出到 pool->sensor。不决定小车动作。
 *
 * 依赖：BSP/bsp_gpio_sensor.h、BSP/bsp_encoder.h
 * 禁止：调用 Motion、Decision、Comm
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    uint8_t  track_bits;
    int16_t  track_error;
    bool     track_valid;
    uint16_t ultrasonic_mm;
    bool     ultrasonic_valid;
    bool     obstacle_near;
} SensorState;

/* ── 生命周期 ── */

void Sensor_Init(void);

/** 20ms 周期调用：读取所有传感器，写入 pool->sensor */
void Sensor_Task20ms(SystemStatePool *pool);

/** 获取传感器状态快照（调试用） */
SensorState Sensor_GetState(void);

#endif /* SENSOR_DOMAIN_H */
