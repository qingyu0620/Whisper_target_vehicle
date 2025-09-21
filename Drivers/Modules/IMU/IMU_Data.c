#include "IMU_Data.h"
#include "IMU.h"
#include "Kalman.h"


#include "math.h"
#include "stdio.h"

Quaternion attitude = {1.0f, 0.0f, 0.0f, 0.0f};
IMU_DATA imu_data;


void Quaternion_normalize(Quaternion* q)
{
	float norm = sqrtf(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
	if (norm > 0.0f) 
	{
        q->q0 /= norm;
        q->q1 /= norm;
        q->q2 /= norm;
        q->q3 /= norm;
    }
}

#define GYRO_SCALE_FACTOR (30.f / 11.f)

void IMU_Update_Data(IMU_DATA* imu, float delta_time)
{
    float q0 = attitude.q0, q1 = attitude.q1, q2 = attitude.q2, q3 = attitude.q3;
    float ax = imu->ax, ay = imu->ay, az = imu->az;
    float gx = imu->gx, gy = imu->gy, gz = imu->gz;
	

    // 1. 归一化加速度计数据
    float norm_a = sqrtf(ax*ax + ay*ay + az*az);
    if (norm_a > 0.0f) 
	{
        ax /= norm_a;
        ay /= norm_a;
        az /= norm_a;
	}

    // 2. 计算估计的重力方向（从当前姿态推算）
    // 在机体坐标系中，重力方向应该是 [0, 0, 1] 在导航系中的表示
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3. 计算加速度计测量值与估计值之间的误差
    // 误差是这两个向量的叉积，代表了姿态误差
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // 4. 将误差积分并反馈到陀螺仪数据中，修正漂移
    gx += BETA * ex;
    gy += BETA * ey;
    gz += BETA * ez; // 注意：没有磁力计，Yaw的修正较弱
	
	
    // 5. 用修正后的陀螺仪数据积分四元数
    float q0_dot = 0.5f * (-gx*q1 - gy*q2 - gz*q3);
    float q1_dot = 0.5f * ( gx*q0 + gz*q2 - gy*q3);
    float q2_dot = 0.5f * ( gy*q0 - gz*q1 + gx*q3);
    float q3_dot = 0.5f * ( gz*q0 + gy*q1 - gx*q2);

    // 6. 积分更新四元数
    attitude.q0 += q0_dot * delta_time;
    attitude.q1 += q1_dot * delta_time;
    attitude.q2 += q2_dot * delta_time;
    attitude.q3 += q3_dot * delta_time;

    // 7. 归一化四元数（必须步骤）
    Quaternion_normalize(&attitude);
}


EulerAngles Quaternion_to_Euler(const Quaternion* q)
{
	EulerAngles euler;
	
    // 计算滚转角 (x-axis rotation)
    euler.roll = atan2f(2.0f * (q->q0 * q->q1 + q->q2 * q->q3), 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2));
    
    // 计算俯仰角 (y-axis rotation)
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
	
    if (fabsf(sinp) >= 1.0f)
	{
        euler.pitch = copysignf(PI / 2.0f, sinp); // 使用90度如果超出范围
    } 
	else 
	{
        euler.pitch = asinf(sinp);
    }
    
    // 计算偏航角 (z-axis rotation)
    euler.yaw = atan2f(2.0f * (q->q0 * q->q3 + q->q1 * q->q2), 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3));
	
	euler.roll *= RAD_TO_DEG;
    euler.pitch *= RAD_TO_DEG;
    euler.yaw *= RAD_TO_DEG;
	
	if(euler.yaw < 0)
		euler.yaw = 180 + euler.yaw;
	
	return euler;
}



