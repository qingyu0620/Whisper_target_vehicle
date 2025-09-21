#include "TIM.h"
#include "IBUS.h"
#include "BMI088.h"
#include "BMI088_Driver.h"

#include "Chassis.h"
#include "stdio.h"

extern BMI088_DATA imu_data;
extern AttitudeEstimator estimator;
extern Kalman_Data kalman_data;

TIM_HandleTypeDef tim2_handle;
TIM_HandleTypeDef tim3_handle;


void TIM2_Init()
{
	tim2_handle.Instance = TIM2;
	tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	tim2_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2_handle.Init.Prescaler = 200 - 1;
	tim2_handle.Init.Period = 1000000 * TIM2_Detect_Time - 1;
	tim2_handle.Init.RepetitionCounter = 0;
	
	HAL_TIM_Base_Init(&tim2_handle);
	
	HAL_TIM_Base_Start_IT(&tim2_handle);
}


void TIM3_Init()
{
	tim3_handle.Instance = TIM3;
	tim3_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	tim3_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	tim3_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim3_handle.Init.Prescaler = 200 - 1;
	tim3_handle.Init.Period = 1000 - 1;
	tim3_handle.Init.RepetitionCounter = 0;

	HAL_TIM_PWM_Init(&tim3_handle);

	TIM_OC_InitTypeDef tim3_config;
	tim3_config.OCFastMode = TIM_OCFAST_DISABLE;
	tim3_config.OCMode = TIM_OCMODE_PWM1;
	tim3_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	tim3_config.Pulse = 0;

	HAL_TIM_PWM_ConfigChannel(&tim3_handle, &tim3_config, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&tim3_handle, TIM_CHANNEL_4);
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		__HAL_RCC_TIM2_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM2_IRQn, 2, 2);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();
		GPIOB_CLK();

		GPIO_InitTypeDef gpio_initstruct;

		gpio_initstruct.Alternate = GPIO_AF2_TIM3;
		gpio_initstruct.Mode = AF_PP;
		gpio_initstruct.Pin = P1;
		gpio_initstruct.Pull = PULLUP;
		gpio_initstruct.Speed = HIGH;
		HAL_GPIO_Init(GPIOB, &gpio_initstruct);
	}
}


void TIM2_IRQHandler()
{
	HAL_TIM_IRQHandler(&tim2_handle);
}


/**
* @brief:       HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
* @param:       htim: 触发溢出中断的定时器对应句柄
* @retval:      void
* @details:    	每TIM2_Detect_Time触发一次
*				读取陀螺仪roll pitch yaw 原始数据
*				解算欧拉角
*				控制底盘移动
**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		//读取原始数据
		BMI088_read_(&imu_data);
		//卡尔曼滤波、互补滤波、四元数解算欧拉角
		Kalman_Attitude_Estimator_Update(&imu_data);
		//温度控制
		TempCtrl_BMI088();
		//底盘控制
		Chassis_movement();
	}
}


/**
* @brief:       IMU_PWM_Set(uint16_t pwmx)
* @param:       pwmx: pwmx占空比
* @retval:      void
* @details:    	控制IMU加热
**/
void TIM3_PWM_Set(uint16_t pwmx)
{
   __HAL_TIM_SET_COMPARE(&tim3_handle, TIM_CHANNEL_4, pwmx);
}




