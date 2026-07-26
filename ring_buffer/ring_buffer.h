#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RING_BUFFER_SIZE
#define RING_BUFFER_SIZE 256U
#endif

#if (RING_BUFFER_SIZE == 0U) || (RING_BUFFER_SIZE > 65535U)
#error "RING_BUFFER_SIZE must be in the range 1..65535"
#endif

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} RingBuffer;

void RingBuffer_Init(RingBuffer *rb);
uint8_t RingBuffer_IsEmpty(const RingBuffer *rb);
uint8_t RingBuffer_IsFull(const RingBuffer *rb);
uint8_t RingBuffer_Put(RingBuffer *rb, uint8_t data);
uint8_t RingBuffer_Get(RingBuffer *rb, uint8_t *data);
void RingBuffer_Flush(RingBuffer *rb);
uint16_t RingBuffer_GetFreeSpace(const RingBuffer *rb);
uint16_t RingBuffer_GetUsedSpace(const RingBuffer *rb);
uint16_t RingBuffer_Write(RingBuffer *rb, const uint8_t *data, uint16_t len);
uint16_t RingBuffer_Read(RingBuffer *rb, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */