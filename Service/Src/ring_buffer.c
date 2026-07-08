#include "ring_buffer.h"

/* ────────────────────────────────────────────────────────────
 * 环形缓冲区实现
 *
 * 空条件：head == tail
 * 满条件：(head + 1) % capacity == tail
 *         保留一个空位来区分"满"和"空"
 *
 * head 只被 ISR（生产者）写，tail 只被 Task（消费者）写。
 * 两方各自操作自己的指针，根本不需要锁。
 * ──────────────────────────────────────────────────────────── */

void RingBuffer_Init(RingBuffer *rb, uint8_t *storage, uint16_t capacity)
{
    if ((rb == 0) || (storage == 0) || (capacity < 2U))
    {
        return;     /* 参数无效，静默忽略 */
    }

    rb->buffer   = storage;
    rb->capacity = capacity;
    rb->head     = 0U;
    rb->tail     = 0U;
}

/* ── ISR 写 ── */

bool RingBuffer_PushFromIsr(RingBuffer *rb, uint8_t byte)
{
    if (rb == 0) { return false; }

    uint16_t next = rb->head + 1U;
    if (next >= rb->capacity) { next = 0U; }    /* 回绕 */

    /* 满 → 丢弃该字节 */
    if (next == rb->tail) { return false; }

    rb->buffer[rb->head] = byte;
    rb->head = next;
    return true;
}

/* ── Task 读 ── */

bool RingBuffer_Pop(RingBuffer *rb, uint8_t *byte)
{
    if ((rb == 0) || (byte == 0)) { return false; }

    /* 空 → 无数据可读 */
    if (rb->head == rb->tail) { return false; }

    *byte = rb->buffer[rb->tail];

    uint16_t next = rb->tail + 1U;
    if (next >= rb->capacity) { next = 0U; }    /* 回绕 */

    rb->tail = next;
    return true;
}

uint16_t RingBuffer_Available(const RingBuffer *rb)
{
    if (rb == 0) { return 0U; }

    /* head ≥ tail → 可用字节在 [tail, head) */
    if (rb->head >= rb->tail) { return rb->head - rb->tail; }

    /* head < tail → 发生了回绕，可用字节 = 尾部剩余 + 头部已用 */
    return (uint16_t)(rb->capacity - rb->tail + rb->head);
}

void RingBuffer_Flush(RingBuffer *rb)
{
    if (rb == 0) { return; }
    rb->head = 0U;
    rb->tail = 0U;
}
