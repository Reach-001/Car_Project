#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * LED 驱动 —— 硬件抽象层头文件
 *
 * 硬件：PA4=LED1, PA5=LED2, PC4=LED3, PC6=STATE_LED
 *       推挽输出，高电平点亮
 *
 * BspLed_Init() 一次性初始化全部四个 LED（默认全灭）。
 * 每个 LED 独立控制开关。
 * ──────────────────────────────────────────────────────────── */

typedef enum
{
    BSP_LED_1 = 0,          /* PA4 */
    BSP_LED_2,              /* PA5 */
    BSP_LED_3,              /* PC4 */
    BSP_LED_STATE,          /* PC6 — 状态指示灯（心跳） */
    BSP_LED_COUNT
} BspLedId;

/* ── 生命周期 ── */

/** 初始化全部四个 LED，默认全灭 */
void BspLed_Init(void);

/* ── 控制接口 ── */

/** 设置指定 LED 的亮灭 */
void BspLed_Set(BspLedId led, bool on);

/** 翻转指定 LED 的电平 */
void BspLed_Toggle(BspLedId led);

#endif /* BSP_LED_H */
