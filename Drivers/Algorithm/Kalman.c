#include "Kalman.h"
#include "stdio.h"

void Kalman_Init(KalmanFilter* kf, float Q_angle, float Q_bias, float R_measure)
{
    kf->Q_angle = Q_angle;          // 过程噪声协方差（角度）
    kf->Q_bias = Q_bias;            // 过程噪声协方差（偏差）
    kf->R_measure = R_measure;      // 测量噪声协方差
    kf->angle = 0.0f;               // 初始角度估计
    kf->bias = 0.0f;                // 初始偏差估计
    kf->P[0][0] = 1.0f;             // 初始协方差矩阵
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}


float Kalman_Update(KalmanFilter* kf, float measurement, float rate, float dt)
{
    kf->angle += dt * (rate - kf->bias);
    
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    float S = kf->P[0][0] + kf->R_measure;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    
    float y = measurement - kf->angle;
    kf->angle += K0 * y;
    kf->bias += K1 * y;
    
    kf->P[0][0] -= K0 * kf->P[0][0];
    kf->P[0][1] -= K0 * kf->P[0][1];
    kf->P[1][0] -= K1 * kf->P[0][0];
    kf->P[1][1] -= K1 * kf->P[0][1];

    // printf("%f\n", kf->angle);
    
    return kf->angle;
}





