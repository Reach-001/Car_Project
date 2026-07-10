#ifndef HMI_H
#define HMI_H

#include <stdbool.h>
#include <stdint.h>

#include "system_state_pool.h"

/* ────────────────────────────────────────────────────────────
 * HMI 域 —— 人机交互：按键、LED、蜂鸣器
 *
 * 职责：
 *   按键 → 写入 pool->event.key_*
 *   LED / 蜂鸣器 → 根据 pool->mode / pool->fault 状态更新
 *
 * 依赖：BSP/bsp_key.h、BSP/bsp_led.h、BSP/bsp_buzzer.h
 * 禁止：调用 Motion、Decision、Comm
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    bool led1_on;
    bool led2_on;
    bool led3_on;
    bool state_led_on;
    bool buzzer_active;
} HmiState;

/* ── 生命周期 ── */

void Hmi_Init(void);

/** 10ms 周期调用：读取按键事件（去抖需要 10ms 频率） */
void Hmi_KeyTask10ms(SystemStatePool *pool);

/** 500ms 周期调用：处理按键事件 + 更新 LED/蜂鸣器 */
void Hmi_Task500ms(SystemStatePool *pool);

HmiState Hmi_GetState(void);

#endif /* HMI_H */
