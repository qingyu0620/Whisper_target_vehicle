#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "sys.h"
#include "User_Define.h"

#define Chassis_Delay			0.005				    //s
#define Chassis_Task_Delay	    Chassis_Delay * 1000    //ms

void Chassis_Init(void);
void Chassis_movement(void);

#endif
