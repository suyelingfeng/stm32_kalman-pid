#include "ring_buffer.h"

#include <stddef.h>

static uint16_t RingBuffer_NextIndex(uint16_t index)
{
    ++index;
    if (index == (uint16_t)RING_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

void RingBuffer_Init(RingBuffer *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->head = 0U;
    rb->tail = 0U;
    rb->count = 0U;
}

uint8_t RingBuffer_IsEmpty(const RingBuffer *rb)
{
    return (uint8_t)((rb == NULL) || (rb->count == 0U));
}

uint8_t RingBuffer_IsFull(const RingBuffer *rb)
{
    return (uint8_t)((rb != NULL) &&
                     (rb->count == (uint16_t)RING_BUFFER_SIZE));
}

uint8_t RingBuffer_Put(RingBuffer *rb, uint8_t data)
{
    if ((rb == NULL) || RingBuffer_IsFull(rb)) {
        return 0U;
    }
    rb->buffer[rb->head] = data;
    rb->head = RingBuffer_NextIndex(rb->head);
    ++rb->count;
    return 1U;
}

uint8_t RingBuffer_Get(RingBuffer *rb, uint8_t *data)
{
    if ((rb == NULL) || (data == NULL) || RingBuffer_IsEmpty(rb)) {
        return 0U;
    }
    *data = rb->buffer[rb->tail];
    rb->tail = RingBuffer_NextIndex(rb->tail);
    --rb->count;
    return 1U;
}

void RingBuffer_Flush(RingBuffer *rb)
{
    RingBuffer_Init(rb);
}

uint16_t RingBuffer_GetFreeSpace(const RingBuffer *rb)
{
    if (rb == NULL) {
        return 0U;
    }
    return (uint16_t)RING_BUFFER_SIZE - rb->count;
}

uint16_t RingBuffer_GetUsedSpace(const RingBuffer *rb)
{
    return (rb == NULL) ? 0U : rb->count;
}

uint16_t RingBuffer_Write(RingBuffer *rb, const uint8_t *data, uint16_t len)
{
    uint16_t written = 0U;
    if ((rb == NULL) || (data == NULL)) {
        return 0U;
    }
    while ((written < len) && (rb->count < (uint16_t)RING_BUFFER_SIZE)) {
        rb->buffer[rb->head] = data[written];
        rb->head = RingBuffer_NextIndex(rb->head);
        ++rb->count;
        ++written;
    }
    return written;
}

uint16_t RingBuffer_Read(RingBuffer *rb, uint8_t *data, uint16_t len)
{
    uint16_t read = 0U;
    if ((rb == NULL) || (data == NULL)) {
        return 0U;
    }
    while ((read < len) && (rb->count > 0U)) {
        data[read] = rb->buffer[rb->tail];
        rb->tail = RingBuffer_NextIndex(rb->tail);
        --rb->count;
        ++read;
    }
    return read;
}