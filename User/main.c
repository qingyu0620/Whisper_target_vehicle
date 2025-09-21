#include "User_Define.h"
#include "sys.h"
#include "Delay.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "FreeRTOS_Text.h"
#include "stdio.h"


int main(void)
{
	MPU_Config();
	HAL_Init();
	SystemClock_Config();
	Delay_init();

	FreeRTOS_Text();
	vTaskStartScheduler();
}




