#ifndef	__BMI088_H__
#define	__BMI088_H__

#include "sys.h"
#include "User_Define.h"
#include "BMI088_Reg.h"
#include "Kalman.h"

typedef struct 
{
    // 四元数
    float q0, q1, q2, q3;
    
    // 翻滚角 旋转角 偏航角
    float roll, pitch, yaw;
    
    // 滤波器效率 0.01 - 0.1
    float filter_coefficient;
    
    // 温度
    float temperate;

    // 检测时间
    float dt;

    //积分器
    float exInt, eyInt, ezInt;
} AttitudeEstimator;


void Attitude_Estimator_Init(AttitudeEstimator* est, float dt, float filter_coef);
void Attitude_Estimator_Update(AttitudeEstimator* est, Kalman_Data* kalman_data);
void Kalman_Attitude_Estimator_Update(BMI088_DATA* imu_data);
void TempCtrl_BMI088(void);


#endif
