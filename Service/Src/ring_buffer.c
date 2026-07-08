#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer *rb, uint8_t *storage, uint16_t capacity)
{
    if ((rb == 0) || (storage == 0) || (capacity < 2U))
    {
        return;
    }

    rb->buffer = storage;
    rb->capacity = capacity;
    rb->head = 0U;
    rb->tail = 0U;
}

bool RingBuffer_PushFromIsr(RingBuffer *rb, uint8_t byte)
{
    if (rb == 0)
    {
        return false;
    }

    uint16_t next = rb->head + 1U;
    if (next >= rb->capacity)
    {
        next = 0U;
    }

    /* full */
    if (next == rb->tail)
    {
        return false;
    }

    rb->buffer[rb->head] = byte;
    rb->head = next;
    return true;
}

bool RingBuffer_Pop(RingBuffer *rb, uint8_t *byte)
{
    if ((rb == 0) || (byte == 0))
    {
        return false;
    }

    /* empty */
    if (rb->head == rb->tail)
    {
        return false;
    }

    *byte = rb->buffer[rb->tail];

    uint16_t next = rb->tail + 1U;
    if (next >= rb->capacity)
    {
        next = 0U;
    }

    rb->tail = next;
    return true;
}

uint16_t RingBuffer_Available(const RingBuffer *rb)
{
    if (rb == 0)
    {
        return 0U;
    }

    if (rb->head >= rb->tail)
    {
        return rb->head - rb->tail;
    }

    return (uint16_t)(rb->capacity - rb->tail + rb->head);
}

void RingBuffer_Flush(RingBuffer *rb)
{
    if (rb == 0)
    {
        return;
    }

    rb->head = 0U;
    rb->tail = 0U;
}
