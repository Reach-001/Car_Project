#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * LED 驱动 —— 硬件抽象层头文件
 *
 * 硬件：PC6 = STATE_LED（状态指示灯）
 *       PA4/LED1, PA5/LED2, PC4/LED3 — 板上预留，暂未驱动
 *
 * 上层通过 BspLed_SetStateLed() 控制心跳灯闪烁
 * ──────────────────────────────────────────────────────────── */

/** 初始化 LED，默认关闭 */
void BspLed_Init(void);

/** 设置状态 LED（PC6）的亮灭状态 */
void BspLed_SetStateLed(bool on);

#endif /* BSP_LED_H */
