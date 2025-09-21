#ifndef	__MOTOR_H__
#define	__MOTOR_H__

#include "sys.h"
#include "User_Define.h"


//电机最大速度电流
#define DJMotor_MaxCurrent	2500
//电机最大旋转电流
#define DJMotor_MaxRotate	3000


//四电机电流结构体
typedef struct
{
	float lf;
	float rf;
	float lb;
	float rb;
}DJMotor_Current;

//电机反馈值结构体
typedef struct
{
	int16_t angle;
	int16_t rpm;
}DJMotor_Feedback;


void DJMotor_Init(void);
void DJMotor_Control(void);
void DJMotor_Stop(void);


#endif
