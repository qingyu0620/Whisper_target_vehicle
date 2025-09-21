#ifndef	__KALMAN_H__
#define	__KALMAN_H__

#include "sys.h"
#include "User_Define.h"

typedef struct 
{
    float Q_angle;          // 过程噪声协方差（角度）
    float Q_bias;           // 过程噪声协方差（偏差）
    float R_measure;        // 测量噪声协方差
    float angle;            // 角度估计
    float bias;             // 偏差估计
    float P[2][2];          // 误差协方差矩阵
} KalmanFilter;


typedef struct
{
    KalmanFilter ax;
    KalmanFilter ay;
    KalmanFilter az;
    KalmanFilter gx;
    KalmanFilter gy;
    KalmanFilter gz;
}Kalman_updata;


typedef struct
{
	float ax, ay, az; // 加速度计数据 (m/s2)
    float gx, gy, gz; // 陀螺仪数据 (rad/s)
    float temp;
}Kalman_Data;



void Kalman_Init(KalmanFilter* kf, float Q_angle, float Q_bias, float R_measure);
float Kalman_Update(KalmanFilter* kf, float measurement, float rate, float dt);


#endif
