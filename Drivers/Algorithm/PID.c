#include "PID.h"

/**
************************************************************************
* @brief:       PIDController Creat_PID()
* @param:       void
* @retval:      PIDController: PID结构体
* @details:    	创造一个PID结构体并返回它
************************************************************************
**/
PIDController Creat_PID()
{
	PIDController target_pid;
	
	target_pid.Kp = 0;
	target_pid.Ki = 0;
	target_pid.Kd = 0;
	target_pid.integral = 0;
	target_pid.last_error = 0;
	target_pid.target_data = 0;
	target_pid.max_data = 0;
	
	return target_pid;
}

/**
************************************************************************
* @brief:       Modify_PID_K(PIDController *target_pid, float Kp, float Ki, float Kd)
* @param:       target_pid: 所调整的目标PID结构体
*				Kp: 比例项
*				Ki: 积分项
*				Kd: 微分项
* @retval:      void
* @details:    	修改PID结构体的K
************************************************************************
**/
void Modify_PID_K(PIDController *target_pid, float Kp, float Ki, float Kd)
{
	target_pid -> Kp = Kp;
	target_pid -> Ki = Ki;
	target_pid -> Kd = Kd;
}


/**
************************************************************************
* @brief:       PID_Calculate(PIDController *target_pid, float detected_data, float dt)
* @param:       target_pid: 所计算的目标PID结构体
*				detected_data: 传入检测的数据
*				dt: 检测时间, 单位为 s
* @retval:      int32_t: 返回PID计算的实际数据
* @details:    	void
************************************************************************
**/
int32_t PID_Calculate(PIDController *target_pid, float detected_data, float dt)
{
	float error = target_pid->target_data - detected_data;
	
	float P = target_pid->Kp * error;
	
	target_pid->integral += error * dt;
	float I = target_pid->Ki * target_pid->integral;
	
	float error_size = error - target_pid->last_error;
	float D = target_pid->Kd * error_size / dt;
	
	target_pid->last_error = error;
	
	float pid = P + I + D;
	
	if(pid > target_pid->max_data)
		pid = target_pid->max_data;
	
	if(pid < -target_pid->max_data)
		pid = -target_pid->max_data;
	
	return pid;
}





