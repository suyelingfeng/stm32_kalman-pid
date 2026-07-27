#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOVING_AVERAGE_MAX_WINDOW
#define MOVING_AVERAGE_MAX_WINDOW 16U
#endif

#if (MOVING_AVERAGE_MAX_WINDOW == 0U) || \
    (MOVING_AVERAGE_MAX_WINDOW > 65535U)
#error "MOVING_AVERAGE_MAX_WINDOW must be in the range 1..65535"
#endif

/** 指数移动平均滤波器：y += alpha * (x - y)。 */
typedef struct {
    float alpha;        /**< 平滑系数，范围 [0, 1]。 */
    float output;       /**< 最近一次滤波输出。 */
    uint8_t initialized; /**< 非零表示已有有效初值。 */
} EmaFilter;

/** 使用循环数组和累计和实现的 O(1) 滑动平均滤波器。 */
typedef struct {
    float buffer[MOVING_AVERAGE_MAX_WINDOW]; /**< 固定容量样本窗口。 */
    float sum;                               /**< 当前有效样本之和。 */
    uint16_t window_size;                    /**< 实际窗口长度。 */
    uint16_t index;                          /**< 下一个写入位置。 */
    uint16_t count;                          /**< 当前有效样本数。 */
} MovingAverageFilter;

/**
 * @brief 初始化 EMA 滤波器。
 * @param alpha 新样本权重；超出 [0,1] 时自动限幅。
 * @param initial_value 初始输出，可设置为首个传感器读数以减小启动跳变。
 */
void EmaFilter_Init(EmaFilter *filter, float alpha, float initial_value);

/** @brief 输入一个样本并返回新的 EMA 输出。 */
float EmaFilter_Update(EmaFilter *filter, float input);

/** @brief 重置 EMA 输出但保留 alpha。 */
void EmaFilter_Reset(EmaFilter *filter, float value);

/**
 * @brief 初始化滑动平均滤波器。
 * @param window_size 窗口长度；自动限制到 1..MOVING_AVERAGE_MAX_WINDOW。
 */
void MovingAverage_Init(MovingAverageFilter *filter, uint16_t window_size);

/**
 * @brief 输入一个样本并返回当前有效窗口平均值。
 * @note 窗口尚未填满时，只对已经收到的样本求平均。
 */
float MovingAverage_Update(MovingAverageFilter *filter, float input);

/** @brief 清空滑动窗口但保留 window_size。 */
void MovingAverage_Reset(MovingAverageFilter *filter);

/** @brief 返回三个浮点样本的中值，用于抑制孤立尖峰。 */
float Median3_Filter(float first, float second, float third);

#ifdef __cplusplus
}
#endif

#endif /* FILTERS_H */