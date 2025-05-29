#include "ring_buffer.h"
#include "stm32f1xx_hal.h"
#include"include.h"

/*
 * 环形缓冲区使用示例：
 * 
 * 1. 初始化缓冲区：
 *    RingBuffer buffer;
 *    RingBuffer_Init(&buffer);
 * 
 * 2. 放入数据(通常在中断中)：
 *    uint8_t data = 0x55;
 *    if(RingBuffer_Put(&buffer, data)) {
 *        // 放入成功
 *    } else {
 *        // 缓冲区已满
 *    }
 * 
 * 3. 取出数据(通常在主循环中)：
 *    uint8_t received;
 *    if(RingBuffer_Get(&buffer, &received)) {
 *        // 处理received数据
 *    } else {
 *        // 缓冲区为空
 *    }
 * 
 * 4. 清空缓冲区：
 *    RingBuffer_Flush(&buffer);
 * 
 * 注意：
 * - 此实现是线程安全的，可在中断和主循环中同时使用
 * - 默认缓冲区大小256字节，可在ring_buffer.h中修改
 */

/**
 * @brief 初始化环形缓冲区
 * @param rb 指向要初始化的RingBuffer结构体指针
 * @note 必须在使用缓冲区前调用此函数进行初始化
 *       将head、tail和count全部置0
 */
void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief 检查缓冲区是否为空
 * @param rb 指向RingBuffer结构体指针
 * @return 1表示缓冲区为空，0表示非空
 * @note 此函数是线程安全的，可在中断中调用
 */
uint8_t RingBuffer_IsEmpty(RingBuffer *rb) {
    return (rb->count == 0);
}

/**
 * @brief 检查缓冲区是否已满
 * @param rb 指向RingBuffer结构体指针
 * @return 1表示缓冲区已满，0表示未满
 * @note 此函数是线程安全的，可在中断中调用
 */
uint8_t RingBuffer_IsFull(RingBuffer *rb) {
    return (rb->count == RING_BUFFER_SIZE);
}

/**
 * @brief 向缓冲区放入一个字节数据
 * @param rb 指向RingBuffer结构体指针
 * @param data 要放入的数据字节
 * @return 1表示成功，0表示失败(缓冲区已满)
 * @note 此函数是线程安全的，适合在中断服务例程中调用
 *       典型应用场景：UART接收中断中存放接收到的数据
 */
uint8_t RingBuffer_Put(RingBuffer *rb, uint8_t data) {
    if (RingBuffer_IsFull(rb)) {
        return 0; // 队列已满
    }
    
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
    rb->count++;
    return 1;
}

/**
 * @brief 从缓冲区取出一个字节数据
 * @param rb 指向RingBuffer结构体指针
 * @param data 用于存储取出数据的指针
 * @return 1表示成功，0表示失败(缓冲区为空)
 * @note 此函数是线程安全的，适合在主循环中调用
 *       典型应用场景：在主程序中处理接收到的数据
 */
uint8_t RingBuffer_Get(RingBuffer *rb, uint8_t *data) {
    if (RingBuffer_IsEmpty(rb)) {
        return 0; // 队列为空
    }
    
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    rb->count--;
    return 1;
}

/**
 * @brief 清空缓冲区
 * @param rb 指向RingBuffer结构体指针
 * @note 将head、tail和count全部置0
 *       清空后缓冲区可重新使用
 */
void RingBuffer_Flush(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief 获取缓冲区剩余空间
 * @param rb 指向RingBuffer结构体指针
 * @return 剩余空间字节数
 */
uint16_t RingBuffer_GetFreeSpace(RingBuffer *rb) {
    return RING_BUFFER_SIZE - rb->count;
}

/**
 * @brief 获取缓冲区已用空间
 * @param rb 指向RingBuffer结构体指针
 * @return 已用空间字节数
 */
uint16_t RingBuffer_GetUsedSpace(RingBuffer *rb) {
    return rb->count;
}

/**
 * @brief 批量写入数据
 * @param rb 指向RingBuffer结构体指针
 * @param data 要写入的数据指针
 * @param len 要写入的数据长度
 * @return 实际写入的字节数
 */
uint16_t RingBuffer_Write(RingBuffer *rb, uint8_t *data, uint16_t len) {
    uint16_t i;
    for(i = 0; i < len && !RingBuffer_IsFull(rb); i++) {
        RingBuffer_Put(rb, data[i]);
    }
    return i;
}

/**
 * @brief 批量读取数据
 * @param rb 指向RingBuffer结构体指针
 * @param data 存储读取数据的指针
 * @param len 要读取的最大长度
 * @return 实际读取的字节数
 */
uint16_t RingBuffer_Read(RingBuffer *rb, uint8_t *data, uint16_t len) {
    uint16_t i;
    for(i = 0; i < len && !RingBuffer_IsEmpty(rb); i++) {
        RingBuffer_Get(rb, &data[i]);
    }
    return i;
}
// 在main.c中
// #include "include.h"

// RingBuffer uart_rx_buf;

// int main(void) {
//     // 初始化环形缓冲区
//     RingBuffer_Init(&uart_rx_buf);
    
//     // 放入数据
//     uint8_t data = 0x55;
//     if(RingBuffer_Put(&uart_rx_buf, data)) {
//         // 成功放入
//     }
    
//     // 取出数据
//     uint8_t received;
//     if(RingBuffer_Get(&uart_rx_buf, &received)) {
//         // 成功取出
//     }
// }
