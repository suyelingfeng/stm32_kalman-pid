#include "kalman.h"

#include <float.h>
#include <stddef.h>

static float Kalman_NonNegative(float value)
{
    return (value < 0.0f) ? 0.0f : value;
}

void Kalman_Init(KalmanFilter *kf, float Q, float R, float A, float H,
                 float init_x, float init_P)
{
    if (kf == NULL) {
        return;
    }
    kf->Q = Kalman_NonNegative(Q);
    kf->R = Kalman_NonNegative(R);
    kf->A = A;
    kf->H = H;
    Kalman_Reset(kf, init_x, init_P);
}

float Kalman_Filter(KalmanFilter *kf, float z)
{
    float predicted_p;
    float innovation_covariance;
    float correction;
    if (kf == NULL) {
        return 0.0f;
    }
    kf->x = kf->A * kf->x;
    predicted_p = kf->A * kf->P * kf->A + kf->Q;
    predicted_p = Kalman_NonNegative(predicted_p);
    innovation_covariance =
        kf->H * predicted_p * kf->H + kf->R;
    if (innovation_covariance <= FLT_EPSILON) {
        kf->K = 0.0f;
        kf->P = predicted_p;
        return kf->x;
    }
    kf->K = predicted_p * kf->H / innovation_covariance;
    kf->x += kf->K * (z - kf->H * kf->x);
    /* Joseph form keeps covariance non-negative under float rounding. */
    correction = 1.0f - kf->K * kf->H;
    kf->P = correction * correction * predicted_p +
            kf->K * kf->K * kf->R;
    kf->P = Kalman_NonNegative(kf->P);
    return kf->x;
}

void Kalman_Reset(KalmanFilter *kf, float init_x, float init_P)
{
    if (kf == NULL) {
        return;
    }
    kf->x = init_x;
    kf->P = Kalman_NonNegative(init_P);
    kf->K = 0.0f;
}