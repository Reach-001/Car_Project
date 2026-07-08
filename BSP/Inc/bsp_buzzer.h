#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * 无源蜂鸣器驱动 —— 硬件抽象层头文件
 *
 * 硬件：PB10 GPIO 输出 + TIM7 中断翻转电平 → 方波 → 声音
 *       TIM7 由 CubeMX IOC 管理（基本定时器，内部时钟源）
 *       默认频率 2000Hz，范围 200~5000Hz
 *
 * CubeMX 配置：TIM7 Prescaler=169, Period=249
 *   → 170MHz / 170 = 1MHz 计数频率
 *   → 1MHz / (2 × 2000Hz) = 250 ticks = Period 249
 *   → 2000Hz 方波（每个中断翻转一次 GPIO）
 *
 * ISR 路线：TIM7_IRQHandler（CubeMX 生成）
 *        → HAL_TIM_IRQHandler(&htim7)
 *        → HAL_TIM_PeriodElapsedCallback（本模块实现）
 *        → 翻转 PB10 GPIO
 *
 * 预设提示音模式：OK / ERROR / START / OBSTACLE
 * 上层只需调用 BspBuzzer_Play(pattern)
 * ──────────────────────────────────────────────────────────── */

typedef enum
{
    BUZZER_PATTERN_NONE     = 0,   /* 静音                          */
    BUZZER_PATTERN_OK,             /* "嘀" 一声 80ms               */
    BUZZER_PATTERN_ERROR,          /* "嘀-嘀-嘀" 三声 120ms/间隔   */
    BUZZER_PATTERN_START,          /* "嘀嘀" 两声 60ms/间隔        */
    BUZZER_PATTERN_OBSTACLE        /* "嘀嘀嘀嘀嘀" 五声 50ms/间隔  */
} BuzzerPattern;

/* ── 生命周期 ── */

/** 初始化蜂鸣器：启动 TIM7 中断（但不立即发声） */
void BspBuzzer_Init(void);

/* ── 控制接口 ── */

/** 开/关（连续鸣响，不推荐频繁使用） */
void BspBuzzer_Set(bool on);

/** 设置频率（Hz），范围 200~5000。运行时动态修改 TIM7 ARR */
void BspBuzzer_SetFrequency(uint16_t frequency_hz);

/** 蜂鸣指定时长（ms），自动停止 */
void BspBuzzer_Beep(uint16_t on_ms);

/** 播放预设提示音模式 */
void BspBuzzer_Play(BuzzerPattern pattern);

/* ── 周期任务 ── */

/** 10ms 周期调用，驱动提示音的状态机（on/off 交替） */
void BspBuzzer_Task10ms(void);

/** 是否正在发声 */
bool BspBuzzer_IsActive(void);

#endif /* BSP_BUZZER_H */
