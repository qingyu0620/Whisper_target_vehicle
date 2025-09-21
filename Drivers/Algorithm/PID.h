#ifndef	__PID_H__
#define	__PID_H__

#include "sys.h"
#include "User_Define.h"


typedef struct
{
	float Kp, Ki, Kd;
	float last_error;
	float integral;
	
	int32_t target_data;
	int32_t max_data;
}PIDController;


PIDController Creat_PID(void);
void Modify_PID_K(PIDController *target_pid, float Kp, float Ki, float Kd);
int32_t PID_Calculate(PIDController *target_pid, float detected_data, float dt);


#endif
