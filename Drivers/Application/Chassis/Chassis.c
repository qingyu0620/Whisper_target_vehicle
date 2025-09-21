#include "Chassis.h"
#include "Motor.h"
#include "BMI088.h"
#include "IBUS.h"
#include "PID.h"

#include "stdio.h"

extern Remote_a8x remote;
extern PIDController pid_Lf, pid_Rf, pid_Lb, pid_Rb;
extern AttitudeEstimator estimator;

DJMotor_Current motor_current;
static float chassis_vx, chassis_vy;


/**
* @brief	Chassis_Init()
* @param	void
* @retval	void
* @details	初始化底盘
**/
void Chassis_Init()
{
	//大疆电机初始化
    DJMotor_Init();
	//遥控器初始化
    Remote_Init();
}


/**
* @brief	Rotation_Matrix()
* @param	void
* @retval	void
* @details	旋转矩阵映射Vx, Vy
**/
void Rotation_Matrix()
{
	if(fabs(remote.X) < 0.05) remote.X = 0;
	if(fabs(remote.Y) < 0.05) remote.Y = 0;
	if(fabs(remote.R) < 0.05) remote.R = 0;
	
	static float cos_theta, sin_theta;
	
	cos_theta = cosf((estimator.roll) * PI / 180.0f);
	sin_theta = sinf((estimator.roll) * PI / 180.0f);
	
	chassis_vx = remote.X * cos_theta - remote.Y * sin_theta;
	chassis_vy = remote.X * sin_theta + remote.Y * cos_theta;
}


/**
* @brief	MecanumCalculate()
* @param	void
* @retval	void
* @details	解算四个电机的目标电流值, 发送给pid.target_data
**/
void MecanumCalculate()
{
	motor_current.lf = (chassis_vx + chassis_vy) * DJMotor_MaxCurrent + remote.R * DJMotor_MaxRotate;
	motor_current.rf = (chassis_vx - chassis_vy) * DJMotor_MaxCurrent - remote.R * DJMotor_MaxRotate;
	motor_current.lb = (chassis_vx - chassis_vy) * DJMotor_MaxCurrent + remote.R * DJMotor_MaxRotate;
	motor_current.rb = (chassis_vx + chassis_vy) * DJMotor_MaxCurrent - remote.R * DJMotor_MaxRotate;
	
	pid_Lf.target_data = motor_current.lf;
	pid_Rf.target_data = motor_current.rf;
	pid_Lb.target_data = motor_current.lb;
	pid_Rb.target_data = motor_current.rb;
}


/**
* @brief	Chassis_movement()
* @param	void
* @retval	void
* @details	底盘移动控制, 当remote.swa为Remote_On时开启, 为Remote_Off时停止
**/
void Chassis_movement()
{
    if(remote.swa == Remote_On)
    {
		Rotation_Matrix();
		MecanumCalculate();
		DJMotor_Control();
    }
    else if(remote.swa == Remote_Off)
    {
        DJMotor_Stop();
    }
}

