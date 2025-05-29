/**
 * @file kalman.h
 * @brief JSON库实现
 * @author suye
 * @date 2025/5/29
 */
#ifndef __KALMAN_H
#define __KALMAN_H

#include "stm32f1xx_hal.h"

typedef struct {
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float x;        // 估计值
    float P;        // 估计误差协方差
    float K;        // 卡尔曼增益
    float A;        // 状态转移系数
    float H;        // 观测系数
} KalmanFilter;

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
                float init_x, float init_P);

/**
 * @brief 卡尔曼滤波计算
 * @param kf 卡尔曼滤波器指针
 * @param z 测量值
 * @return 滤波后的估计值
 */
float Kalman_Filter(KalmanFilter *kf, float z);

/**
 * @brief 重置卡尔曼滤波器
 * @param kf 卡尔曼滤波器指针
 * @param init_x 初始状态值
 * @param init_P 初始估计误差协方差
 */
void Kalman_Reset(KalmanFilter *kf, float init_x, float init_P);

#endif /* __KALMAN_H */
