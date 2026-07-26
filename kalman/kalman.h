#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Q;
    float R;
    float x;
    float P;
    float K;
    float A;
    float H;
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf, float Q, float R, float A, float H,
                 float init_x, float init_P);
float Kalman_Filter(KalmanFilter *kf, float z);
void Kalman_Reset(KalmanFilter *kf, float init_x, float init_P);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H */