#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/** 融合积分速率与长期参考角的一阶互补滤波器。 */
typedef struct {
    float alpha; /**< 预测角权重，范围 [0,1]。 */
    float angle; /**< 最近一次融合角度，单位由调用方统一。 */
} ComplementaryFilter;

/**
 * @brief 初始化互补滤波器。
 * @param alpha 预测角权重；超出 [0,1] 时自动限幅。
 * @param initial_angle 初始角度。
 */
void ComplementaryFilter_Init(ComplementaryFilter *filter,
                              float alpha,
                              float initial_angle);

/**
 * @brief 融合角速度积分结果与参考角。
 * @param gyro_rate 角速度，单位应为角度单位/秒。
 * @param reference_angle 加速度计等低频参考角。
 * @param dt_seconds 固定采样周期，单位秒；负值按零处理。
 * @return 更新后的融合角度。
 */
float ComplementaryFilter_Update(ComplementaryFilter *filter,
                                 float gyro_rate,
                                 float reference_angle,
                                 float dt_seconds);

/** @brief 重置融合角度但保留 alpha。 */
void ComplementaryFilter_Reset(ComplementaryFilter *filter, float angle);

#ifdef __cplusplus
}
#endif

#endif /* COMPLEMENTARY_FILTER_H */