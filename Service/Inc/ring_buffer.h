#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

/* ── ISR-safe ring buffer (single producer, single consumer) ──
 *
 * Usage pattern:
 *   ISR:  RingBuffer_PushFromIsr(&rb, byte);
 *   Task: RingBuffer_Pop(&rb, &byte);
 *
 * No locks needed when one side writes and the other reads.
 * head  is written only by ISR  (producer)
 * tail  is written only by Task (consumer)
 */

typedef struct
{
    uint8_t *buffer;            /* externally provided storage */
    uint16_t capacity;          /* total buffer size in bytes */
    volatile uint16_t head;     /* ISR writes */
    uint16_t tail;              /* Task writes */
} RingBuffer;

/* ── lifecycle ── */

void  RingBuffer_Init(RingBuffer *rb, uint8_t *storage, uint16_t capacity);

/* ── ISR side (never called from task) ── */

bool  RingBuffer_PushFromIsr(RingBuffer *rb, uint8_t byte);

/* ── task side (never called from ISR) ── */

bool  RingBuffer_Pop(RingBuffer *rb, uint8_t *byte);
uint16_t RingBuffer_Available(const RingBuffer *rb);
void  RingBuffer_Flush(RingBuffer *rb);

#endif /* RING_BUFFER_H */
