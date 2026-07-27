#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/** 一维离散线性卡尔曼滤波器。 */
typedef struct {
    float Q; /**< 过程噪声方差，必须非负。 */
    float R; /**< 测量噪声方差，必须非负。 */
    float x; /**< 当前状态估计。 */
    float P; /**< 当前估计误差协方差。 */
    float K; /**< 最近一次卡尔曼增益。 */
    float A; /**< 状态转移系数。 */
    float H; /**< 观测系数。 */
} KalmanFilter;

/**
 * @brief 初始化一维卡尔曼滤波器。
 * @param Q 过程噪声方差；负值按零处理。
 * @param R 测量噪声方差；负值按零处理。
 * @param A 状态转移系数。
 * @param H 观测系数。
 * @param init_x 初始状态估计。
 * @param init_P 初始协方差；负值按零处理。
 */
void Kalman_Init(KalmanFilter *kf, float Q, float R, float A, float H,
                 float init_x, float init_P);

/**
 * @brief 完成一次预测和测量更新。
 * @param z 当前测量值。
 * @return 更新后的状态估计。
 * @note 创新协方差退化为零时跳过测量更新，避免除零。
 */
float Kalman_Filter(KalmanFilter *kf, float z);

/** @brief 重置状态、协方差和增益，但保留模型及噪声参数。 */
void Kalman_Reset(KalmanFilter *kf, float init_x, float init_P);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H */