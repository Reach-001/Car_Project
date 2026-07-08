#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * ISR 安全的环形缓冲区（Ring Buffer） — 通用服务模块
 *
 * 设计原则：单生产者 + 单消费者 = 无锁线程安全
 *   生产者（ISR）只写 head → PushFromIsr() 只在 ISR 中调用
 *   消费者（Task）只写 tail → Pop/Available/Flush 只在 Task 中调用
 *   两个指针独立移动，无需关中断或加锁。
 *
 * 存储空间由外部提供，sizeof(RingBuffer) 不含数据存储。
 * 容量减 1 用于区分"空"和"满"（满 = (head+1) % capacity == tail）
 *
 * 使用示例:
 *   static uint8_t buf[128];
 *   RingBuffer rb;
 *   RingBuffer_Init(&rb, buf, sizeof(buf));
 *   ...
 *   ISR: RingBuffer_PushFromIsr(&rb, byte);
 *   Task: RingBuffer_Pop(&rb, &byte);
 * ──────────────────────────────────────────────────────────── */

typedef struct
{
    uint8_t         *buffer;    /* 外部提供的存储空间起始地址 */
    uint16_t         capacity;  /* 缓冲区总容量（字节数）     */
    volatile uint16_t head;     /* 写指针（只被 ISR 修改）   */
             uint16_t tail;     /* 读指针（只被 Task 修改）  */
} RingBuffer;

/* ── 生命周期 ── */

/** 初始化环形缓冲区
 *  @param rb       指向 RingBuffer 实例
 *  @param storage  外部提供的数据存储空间
 *  @param capacity storage 的大小（字节数），最小 2 */
void RingBuffer_Init(RingBuffer *rb, uint8_t *storage, uint16_t capacity);

/* ── ISR 侧（生产者，只能由 ISR 调用） ── */

/** 写入一个字节。缓冲区满时返回 false（丢字节），否则返回 true */
bool RingBuffer_PushFromIsr(RingBuffer *rb, uint8_t byte);

/* ── Task 侧（消费者，只能由 Task 调用） ── */

/** 读取一个字节。缓冲区空时返回 false，否则返回 true 并取出数据 */
bool RingBuffer_Pop(RingBuffer *rb, uint8_t *byte);

/** 当前缓冲区中的可读字节数 */
uint16_t RingBuffer_Available(const RingBuffer *rb);

/** 清空缓冲区（head 和 tail 归零） */
void RingBuffer_Flush(RingBuffer *rb);

#endif /* RING_BUFFER_H */
