#include "User_Define.h"
#include "sys.h"
#include "Delay.h"

#include "Chassis.h"
#include "BMI088.h"
#include "BMI088_Driver.h"

#include "stdio.h"


int main(void)
{
	MPU_Config();
	HAL_Init();
	SystemClock_Config();

	Chassis_Init();
	BMI088_init();
	
	while (1)
	{
		
	}
}




