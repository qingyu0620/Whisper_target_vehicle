#include "BMI088.h"
#include "FreeRTOS.h"
#include "task.h"

#include "TIM.h"
#include "PID.h"
#include "BMI088_Driver.h"

#include "math.h"
#include "stdio.h"

#define INT_LIMIT 0.1f

PIDController tempctrl;
Kalman_updata kalman_after_filter;
BMI088_DATA imu_data;
AttitudeEstimator estimator;
Kalman_Data kalman_data;


/**
* @brief    Attitude_Estimator_Init(AttitudeEstimator* est, float dt, float filter_coef)
* @param    est：姿态解算器结构体
* @param    dt：检测时间（ms）
* @param    filter_coef：互补滤波系数
* @retval   void
* @details  初始化一个姿态解算器实例
**/
void Attitude_Estimator_Init(AttitudeEstimator* est, float dt, float filter_coef)
{
    // 初始四元数 (单位四元数，表示无旋转)
    est->q0 = 1.0f;
    est->q1 = 0.0f;
    est->q2 = 0.0f;
    est->q3 = 0.0f;
    
    // 初始欧拉角
    est->roll = 0.0f;
    est->pitch = 0.0f;
    est->yaw = 0.0f;
    
    // 设置参数
    est->dt = dt;
    est->filter_coefficient = filter_coef;
    est->exInt = 0;
    est->eyInt = 0;
    est->ezInt = 0;
}


/**
* @brief    Normalize_Vector(float* x, float* y, float* z)
* @param    x、y、z：3轴向量
* @retval   void
* @details  将3轴向量转换成单位向量
**/
void Normalize_Vector(float* x, float* y, float* z)
{
    float norm = sqrtf(*x * *x + *y * *y + *z * *z);
    if (norm > 0.0f)
	{
        *x /= norm;
        *y /= norm;
        *z /= norm;
    }
}


/**
* @brief    Quaternion_to_Euler(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw)
* @param    q：单位四元数的四个分量
* @param    roll：翻滚角
* @param    pitch：俯仰角
* @param    yaw：偏航角
* @retval   void
* @details  将四元数转换成欧拉角
**/
void Quaternion_to_Euler(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw)
{
    // 滚转 (x轴旋转)
    float roll_rad = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    *roll = roll_rad * 180.0f / PI;
    
    // 俯仰 (y轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    float pitch_rad;
	
    if (fabsf(sinp) >= 1.0f) 
	{
        pitch_rad = copysignf(PI / 2.0f, sinp); // 使用90度，防止asin溢出
    }
	else
	{
        pitch_rad = asinf(sinp);
    }
    *pitch = pitch_rad * 180.0f / PI;
    
    // 偏航 (z轴旋转)
    float yaw_rad = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    *yaw = yaw_rad * 180.0f / PI;
}


/**
* @brief    Attitude_Estimator_Update(AttitudeEstimator* est, Kalman_Data* kalman_data)
* @param    est：姿态解算器结构体
* @param    kalman_data：卡尔曼滤波数据
* @retval   void
* @details  将滤波数据转换成四元数
**/
void Attitude_Estimator_Update(AttitudeEstimator* est, Kalman_Data* kalman_data)
{
    float norm;
    float halfT = est->dt * 0.5f;
    
    // 1. 归一化滤波后的加速度计数据
    float ax_norm = kalman_data->ax;
    float ay_norm = kalman_data->ay;
    float az_norm = kalman_data->az;
    Normalize_Vector(&ax_norm, &ay_norm, &az_norm);
    
    // 2. 从四元数中计算当前方向的重力矢量
    // 用当前四元数旋转世界坐标系的重力矢量 [0, 0, 1] 到机体坐标系
    float vx = 2.0f * (est->q1 * est->q3 - est->q0 * est->q2);
    float vy = 2.0f * (est->q0 * est->q1 + est->q2 * est->q3);
    float vz = est->q0 * est->q0 - est->q1 * est->q1 - est->q2 * est->q2 + est->q3 * est->q3;
    
    // 3. 计算加速度计测量值与重力矢量估计值之间的误差（向量叉积）
    float ex = (ay_norm * vz - az_norm * vy);
    float ey = (az_norm * vx - ax_norm * vz);
    float ez = (ax_norm * vy - ay_norm * vx);
    
    // 4. 积分误差，用于陀螺仪偏差校正（PI控制器中的积分项）
    // 使用静态变量保持积分状态
    est->exInt += ex * est->filter_coefficient;
    est->eyInt += ey * est->filter_coefficient;
    est->ezInt += ez * est->filter_coefficient;
    
    // 积分限幅
    if(est->exInt > INT_LIMIT) est->exInt = INT_LIMIT;
    if(est->exInt < -INT_LIMIT) est->exInt = -INT_LIMIT;
    if(est->eyInt > INT_LIMIT) est->eyInt = INT_LIMIT;
    if(est->eyInt < -INT_LIMIT) est->eyInt = -INT_LIMIT;
    if(est->ezInt > INT_LIMIT) est->ezInt = INT_LIMIT;
    if(est->ezInt < -INT_LIMIT) est->ezInt = -INT_LIMIT;
    
    // 5. 使用滤波后的陀螺仪数据，并用误差进行校正
    // 注意：这里使用临时变量，不再修改输入的k_data
    float gx_corrected = kalman_data->gx + est->filter_coefficient * ex + est->exInt;
    float gy_corrected = kalman_data->gy + est->filter_coefficient * ey + est->eyInt;
    float gz_corrected = kalman_data->gz + est->filter_coefficient * ez + est->ezInt;
    
    // 6. 使用一阶龙格-库塔法更新四元数
    float q0 = est->q0;
    float q1 = est->q1;
    float q2 = est->q2;
    float q3 = est->q3;
    
    // 四元数微分方程（使用校正后的角速度）
    float qDot0 = 0.5f * (-q1 * gx_corrected - q2 * gy_corrected - q3 * gz_corrected);
    float qDot1 = 0.5f * ( q0 * gx_corrected + q2 * gz_corrected - q3 * gy_corrected);
    float qDot2 = 0.5f * ( q0 * gy_corrected - q1 * gz_corrected + q3 * gx_corrected);
    float qDot3 = 0.5f * ( q0 * gz_corrected + q1 * gy_corrected - q2 * gx_corrected);
    
    // 积分得到新的四元数
    q0 += qDot0 * est->dt;
    q1 += qDot1 * est->dt;
    q2 += qDot2 * est->dt;
    q3 += qDot3 * est->dt;
    
    // 7. 归一化四元数
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    norm = (norm > 0.0f) ? (1.0f / norm) : 1.0f;  // 防止除以零
    est->q0 = q0 * norm;
    est->q1 = q1 * norm;
    est->q2 = q2 * norm;
    est->q3 = q3 * norm;
    
    // 8. 存储温度数据（如果需要）
    est->temperate = kalman_data->temp;
    
    // 9. 将四元数转换为欧拉角
    Quaternion_to_Euler(est->q0, est->q1, est->q2, est->q3, &est->roll, &est->pitch, &est->yaw);
}


/**
* @brief    Kalman_Attitude_Estimator_Update(BMI088_DATA* imu_data)
* @param    imu_data：BMI088数据数组
* @retval   void
* @details  将IMU检测的数据解算成欧拉角
**/
void Kalman_Attitude_Estimator_Update(BMI088_DATA* imu_data)
{
    // kalman_data.ax = Kalman_Update(&kalman_after_filter.ax, imu_data->ax, 0, BMI088_Delay);
    // kalman_data.ay = Kalman_Update(&kalman_after_filter.ay, imu_data->ay, 0, BMI088_Delay);
    // kalman_data.az = Kalman_Update(&kalman_after_filter.az, imu_data->az, 0, BMI088_Delay);
    kalman_data.gx = Kalman_Update(&kalman_after_filter.gx, imu_data->gx, 0, BMI088_Delay);
    kalman_data.gy = Kalman_Update(&kalman_after_filter.gy, imu_data->gy, 0, BMI088_Delay);
    kalman_data.gz = Kalman_Update(&kalman_after_filter.gz, imu_data->gz, 0, BMI088_Delay);
    kalman_data.temp = imu_data->temp;

    Attitude_Estimator_Update(&estimator, &kalman_data);

    printf("%d,%d,%d\n", (int16_t)estimator.roll, (int16_t)estimator.pitch, (int16_t)estimator.yaw);
}


/**
* @brief    TempCtrl_BMI088()
* @param    void
* @retval   void
* @details  BMI088温度控制
**/
void TempCtrl_BMI088()
{
    int16_t pwmx = PID_Calculate(&tempctrl, estimator.temperate, BMI088_Delay);
    if(pwmx < 0)
        pwmx = 0;
    TIM3_PWM_Set(pwmx);
}

