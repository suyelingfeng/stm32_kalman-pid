#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include "stm32f1xx_hal.h"

#define RING_BUFFER_SIZE 256  // 可根据需要调整大小

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} RingBuffer;

// 基础操作接口
void RingBuffer_Init(RingBuffer *rb);
uint8_t RingBuffer_IsEmpty(RingBuffer *rb);
uint8_t RingBuffer_IsFull(RingBuffer *rb);
uint8_t RingBuffer_Put(RingBuffer *rb, uint8_t data);
uint8_t RingBuffer_Get(RingBuffer *rb, uint8_t *data);
void RingBuffer_Flush(RingBuffer *rb);

// 高级封装接口
uint16_t RingBuffer_GetFreeSpace(RingBuffer *rb);
uint16_t RingBuffer_GetUsedSpace(RingBuffer *rb);
uint16_t RingBuffer_Write(RingBuffer *rb, uint8_t *data, uint16_t len);
uint16_t RingBuffer_Read(RingBuffer *rb, uint8_t *data, uint16_t len);

#endif /* __RING_BUFFER_H */
