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

/** 固定容量字节环形缓冲区；并发同步由调用方负责。 */
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE]; /**< 静态数据存储区。 */
    uint16_t head;                    /**< 下一个写入位置。 */
    uint16_t tail;                    /**< 下一个读取位置。 */
    uint16_t count;                   /**< 当前已存字节数。 */
} RingBuffer;

/** @brief 将读写位置和计数器清零。 */
void RingBuffer_Init(RingBuffer *rb);

/** @return 缓冲区为空或 rb 为 NULL 时返回 1，否则返回 0。 */
uint8_t RingBuffer_IsEmpty(const RingBuffer *rb);

/** @return 缓冲区已满时返回 1，否则返回 0。 */
uint8_t RingBuffer_IsFull(const RingBuffer *rb);

/**
 * @brief 写入一个字节。
 * @return 成功返回 1；缓冲区已满或 rb 为 NULL 时返回 0。
 */
uint8_t RingBuffer_Put(RingBuffer *rb, uint8_t data);

/**
 * @brief 读取一个字节。
 * @param data 接收读出字节的地址。
 * @return 成功返回 1；缓冲区为空或参数无效时返回 0。
 */
uint8_t RingBuffer_Get(RingBuffer *rb, uint8_t *data);

/** @brief 丢弃全部未读数据并复位读写位置。 */
void RingBuffer_Flush(RingBuffer *rb);

/** @return 剩余可写字节数；rb 为 NULL 时返回 0。 */
uint16_t RingBuffer_GetFreeSpace(const RingBuffer *rb);

/** @return 当前已用字节数；rb 为 NULL 时返回 0。 */
uint16_t RingBuffer_GetUsedSpace(const RingBuffer *rb);

/**
 * @brief 尽可能批量写入字节。
 * @return 实际写入数量；空间不足时允许短写。
 */
uint16_t RingBuffer_Write(RingBuffer *rb, const uint8_t *data, uint16_t len);

/**
 * @brief 尽可能批量读取字节。
 * @return 实际读取数量；数据不足时允许短读。
 */
uint16_t RingBuffer_Read(RingBuffer *rb, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */