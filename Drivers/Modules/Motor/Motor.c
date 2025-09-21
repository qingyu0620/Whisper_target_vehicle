#include "Motor.h"
#include "CAN.h"
#include "TIM.h"
#include "PID.h"

#include "stdio.h"

DJMotor_Feedback Lf_data, Rf_data, Lb_data, Rb_data;
PIDController pid_Lf, pid_Rf, pid_Lb, pid_Rb;

/**
* @brief:       DJMotor_Init()
* @param:       void
* @retval:      void
* @details:    	初始化电机, 包含FDCAN初始化, TIM2初始化, 以及电机PID初始化
**/
void DJMotor_Init()
{
	FDCAN_Init();

	//配置PID, 选择最大值为PID允许的max_data
	pid_Lf = Creat_PID();
	pid_Lf.max_data = DJMotor_MaxCurrent > DJMotor_MaxRotate ? DJMotor_MaxCurrent : DJMotor_MaxRotate;
	
	pid_Rf = Creat_PID();
	pid_Rf.max_data = DJMotor_MaxCurrent > DJMotor_MaxRotate ? DJMotor_MaxCurrent : DJMotor_MaxRotate;
	
	pid_Lb = Creat_PID();
	pid_Lb.max_data = DJMotor_MaxCurrent > DJMotor_MaxRotate ? DJMotor_MaxCurrent : DJMotor_MaxRotate;
	
	pid_Rb = Creat_PID();
	pid_Rb.max_data = DJMotor_MaxCurrent > DJMotor_MaxRotate ? DJMotor_MaxCurrent : DJMotor_MaxRotate;
	
	//配置PID参数
	Modify_PID_K(&pid_Lf, 1.85, 0.057, 0.0015);
	Modify_PID_K(&pid_Rf, 1.85, 0.057, 0.0015);
	Modify_PID_K(&pid_Lb, 1.85, 0.057, 0.0015);
	Modify_PID_K(&pid_Rb, 1.85, 0.057, 0.0015);
}


/**
* @brief:       DJMotor_Control()
* @param:       void
* @retval:      void
* @details:    	控制电机转速, 转速rpm为反馈值, 目标电流target_data为目标, 解算实际电流actual_current
**/
void DJMotor_Control()
{
	int32_t actual_current1, actual_current2, actual_current3, actual_current4;

	actual_current1 = PID_Calculate(&pid_Lf,  Lf_data.rpm, TIM2_Detect_Time);
	actual_current2 = PID_Calculate(&pid_Rf, -Rf_data.rpm, TIM2_Detect_Time);
	actual_current3 = PID_Calculate(&pid_Lb,  Lb_data.rpm, TIM2_Detect_Time);
	actual_current4 = PID_Calculate(&pid_Rb, -Rb_data.rpm, TIM2_Detect_Time);

	FDCAN_Motor_SendCurrent(actual_current1, -actual_current2, actual_current3, -actual_current4);

	// printf("%d,%d,%d,%d\n", actual_current1, actual_current2, actual_current3, actual_current4);
}


/**
* @brief:       DJMotor_Stop()
* @param:       void
* @retval:      void
* @details:    	控制电机停止
**/
void DJMotor_Stop()
{
	FDCAN_Motor_SendCurrent(0, 0, 0, 0);
}


