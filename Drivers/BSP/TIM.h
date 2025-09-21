#ifndef	__TIM_H__
#define	__TIM_H__

#include "sys.h"
#include "User_Define.h"

//调整TIM2检测时间, 单位 s
#define TIM2_Detect_Time    0.005f

void TIM2_Init();
void TIM3_PWM_Set(uint16_t pwmx);


#endif
