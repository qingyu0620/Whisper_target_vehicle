#include "TIM.h"


TIM_HandleTypeDef tim3_handle;

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


/**
* @brief	IMU_PWM_Set(uint16_t pwmx)
* @param	pwmx: pwmx占空比
* @retval	void
* @details	控制IMU加热
**/
void TIM3_PWM_Set(uint16_t pwmx)
{
   __HAL_TIM_SET_COMPARE(&tim3_handle, TIM_CHANNEL_4, pwmx);
}







