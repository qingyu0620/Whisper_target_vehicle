#ifndef	__IMU_DATA_H__
#define	__IMU_DATA_H__

#include "sys.h"
#include "User_Define.h"

// Beta参数：调整滤波器的收敛速率（推荐0.1-0.5）
#define BETA 			0.2f
#define RAD_TO_DEG 		57.29577951308232f


typedef struct 
{
    float q0;
    float q1;
    float q2;
    float q3;
}Quaternion;


typedef struct 
{
    float roll;   // 滚转角 (度) - 稳定
    float pitch;  // 俯仰角 (度) - 稳定
    float yaw;    // 偏航角 (度) - 会漂移
}EulerAngles;


void IMU_Update_Data(IMU_DATA* imu, float delta_time);
EulerAngles Quaternion_to_Euler(const Quaternion* q);

#endif
