#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * 按键消抖驱动 —— 硬件抽象层头文件
 *
 * 硬件：PA11=KEY1, PA12=KEY2, PA15=KEY3
 *       外部上拉电阻 → 未按时高电平，按下时低电平
 *
 * 去抖策略：状态需连续 N 次采样一致才确认变化
 * 事件模型：Take 语义（读后即清），避免重复处理
 * 周期调用：每 10ms 调一次 BspKey_Task10ms()
 * ──────────────────────────────────────────────────────────── */

typedef enum
{
    BSP_KEY_1 = 0,          /* PA11 */
    BSP_KEY_2,              /* PA12 */
    BSP_KEY_3,              /* PA15 */
    BSP_KEY_COUNT           /* 按键总数 */
} BspKeyId;

/* 按键状态信息（只读快照） */
typedef struct
{
    bool     pressed;           /* 当前是否按下                  */
    bool     pressed_event;     /* 按下事件（已发生但未消费）    */
    bool     released_event;    /* 释放事件（已发生但未消费）    */
    bool     clicked_event;     /* 单击事件（按下+释放≤800ms）  */
    uint32_t pressed_time_ms;   /* 已持续按下的时间（ms）        */
} BspKeyInfo;

/* ── 生命周期 ── */

void BspKey_Init(void);

/** 10ms 周期轮询任务，处理消抖和事件生成 */
void BspKey_Task10ms(void);

/* ── 状态查询 ── */

/** 当前是否按下 */
bool BspKey_IsPressed(BspKeyId key);

/* ── 事件获取（Take 语义：读取后自动清除事件标志） ── */

bool BspKey_TakePressedEvent(BspKeyId key);
bool BspKey_TakeReleasedEvent(BspKeyId key);
bool BspKey_TakeClickedEvent(BspKeyId key);

/** 获取完整状态快照（调试用，不清除事件） */
BspKeyInfo BspKey_GetInfo(BspKeyId key);

#endif /* BSP_KEY_H */
