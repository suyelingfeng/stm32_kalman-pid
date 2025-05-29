/**
 * @file kalman.c
 * @brief JSON库实现
 * @author suye
 * @date 2025/5/29
 */

#include "kalman.h"
#include "stm32f1xx_hal.h"
#include"include.h"
/**
 * @brief 初始化卡尔曼滤波器
 * @param kf 卡尔曼滤波器指针
 * @param Q 过程噪声协方差
 * @param R 测量噪声协方差
 * @param A 状态转移系数
 * @param H 观测系数
 * @param init_x 初始状态值
 * @param init_P 初始估计误差协方差
 */
void Kalman_Init(KalmanFilter *kf, float Q, float R, float A, float H, 
                float init_x, float init_P) {
    kf->Q = Q;
    kf->R = R;
    kf->A = A;
    kf->H = H;
    kf->x = init_x;
    kf->P = init_P;
    kf->K = 0;
}

/**
 * @brief 卡尔曼滤波计算
 * @param kf 卡尔曼滤波器指针
 * @param z 测量值
 * @return 滤波后的估计值
 */
float Kalman_Filter(KalmanFilter *kf, float z) {
    // 预测步骤
    kf->x = kf->A * kf->x;          // 状态预测
    kf->P = kf->A * kf->P * kf->A + kf->Q; // 协方差预测

    // 更新步骤
    kf->K = kf->P * kf->H / (kf->H * kf->P * kf->H + kf->R); // 计算卡尔曼增益
    kf->x = kf->x + kf->K * (z - kf->H * kf->x); // 状态更新
    kf->P = (1 - kf->K * kf->H) * kf->P; // 协方差更新

    return kf->x;
}

/**
 * @brief 重置卡尔曼滤波器
 * @param kf 卡尔曼滤波器指针
 * @param init_x 初始状态值
 * @param init_P 初始估计误差协方差
 */
void Kalman_Reset(KalmanFilter *kf, float init_x, float init_P) {
    kf->x = init_x;
    kf->P = init_P;
    kf->K = 0;
}
// // 初始化卡尔曼滤波器
// KalmanFilter kf;
// Kalman_Init(&kf, 0.01, 0.1, 1, 1, 0, 1);

// // 使用示例
// float measurement = 25.3f; // 测量值
// float filtered_value = Kalman_Filter(&kf, measurement);

// // 重置滤波器
// Kalman_Reset(&kf, 0, 1);
