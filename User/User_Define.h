#ifndef	__User_Define_H__
#define	__User_Define_H__


//GPIO端口时钟
#define	GPIOA_CLK()  				__HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIOA_CLK_DISABLE()			__HAL_RCC_GPIOA_CLK_DISABLE()
#define	GPIOB_CLK()  				__HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIOB_CLK_DISABLE()			__HAL_RCC_GPIOB_CLK_DISABLE()
#define	GPIOC_CLK()  				__HAL_RCC_GPIOC_CLK_ENABLE()
#define GPIOC_CLK_DISABLE()			__HAL_RCC_GPIOC_CLK_DISABLE()
#define	GPIOD_CLK()  				__HAL_RCC_GPIOD_CLK_ENABLE()
#define GPIOD_CLK_DISABLE()			__HAL_RCC_GPIOD_CLK_DISABLE()
#define	GPIOE_CLK()  				__HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIOE_CLK_DISABLE()			__HAL_RCC_GPIOE_CLK_DISABLE()
//定时器时钟
#define TIM2_CLK()					__HAL_RCC_TIM2_CLK_ENABLE()
#define TIM2_CLK_DISABLE()			__HAL_RCC_TIM2_CLK_DISABLE()
#define TIM3_CLK()					__HAL_RCC_TIM3_CLK_ENABLE()
#define TIM3_CLK_DISABLE()			__HAL_RCC_TIM3_CLK_DISABLE()
#define TIM4_CLK()					__HAL_RCC_TIM4_CLK_ENABLE()
#define TIM4_CLK_DISABLE()			__HAL_RCC_TIM4_CLK_DISABLE()
//看门狗时钟
#define WWDG_CLK()					__HAL_RCC_WWDG_CLK_ENABLE()
#define WWDG_CLK_DISABLE()			__HAL_RCC_WWDG_CLK_DISABLE()
//串口时钟
#define UART1_CLK()					__HAL_RCC_USART1_CLK_ENABLE()
#define UART1_CLK_DISABLE()			__HAL_RCC_USART1_CLK_DISABLE()
#define UART2_CLK()					__HAL_RCC_USART2_CLK_ENABLE()
#define UART2_CLK_DISABLE()			__HAL_RCC_USART2_CLK_DISABLE()
#define UART3_CLK()					__HAL_RCC_USART3_CLK_ENABLE()
#define UART3_CLK_DISABLE()			__HAL_RCC_USART3_CLK_DISABLE()
#define UART4_CLK()					__HAL_RCC_USART4_CLK_ENABLE()
#define UART4_CLK_DISABLE()			__HAL_RCC_USART4_CLK_DISABLE()
#define UART5_CLK()					__HAL_RCC_UART5_CLK_ENABLE()
#define UART5_CLK_DISABLE()			__HAL_RCC_UART5_CLK_DISABLE()
#define UART10_CLK()				__HAL_RCC_USART10_CLK_ENABLE()
#define UART10_CLK_DISABLE()		__HAL_RCC_USART10_CLK_DISABLE()
//DMA时钟
#define DMA1_CLK()					__HAL_RCC_DMA1_CLK_ENABLE()
#define DMA1_CLK_DISABLE()			__HAL_RCC_DMA1_CLK_DISABLE()
#define DMA2_CLK()					__HAL_RCC_DMA2_CLK_ENABLE()
#define DMA2_CLK_DISABLE()			__HAL_RCC_DMA2_CLK_DISABLE()
//ADC时钟
#define ADC1_CLK()					__HAL_RCC_ADC1_CLK_ENABLE()
#define ADC1_CLK_DISABLE()			__HAL_RCC_ADC1_CLK_DISABLE()
//SPI时钟
#define SPI1_CLK()					__HAL_RCC_SPI1_CLK_ENABLE()
#define SPI1_CLK_DISABLE()			__HAL_RCC_SPI1_CLK_DISABLE()
//RCC时钟
#define PWR_CLK()					__HAL_RCC_PWR_CLK_ENABLE()
#define PWR_CLK_DISABLE()			__HAL_RCC_PWR_CLK_DISABLE()
#define BKP_CLK()					__HAL_RCC_BKP_CLK_ENABLE()
#define BKP_CLK_DISABLE()			__HAL_RCC_BKP_CLK_DISABLE()
//FDCAN时钟
#define FDCAN_CLK()					__HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCAN_CLK_DISABLE()			__HAL_RCC_FDCAN_CLK_DISABLE()




//端口引脚
#define P0 					GPIO_PIN_0 
#define P1					GPIO_PIN_1 	  
#define P2					GPIO_PIN_2 
#define P3					GPIO_PIN_3 
#define P4					GPIO_PIN_4 
#define P5					GPIO_PIN_5 
#define P6					GPIO_PIN_6 
#define P7					GPIO_PIN_7 
#define P8					GPIO_PIN_8 
#define P9					GPIO_PIN_9 
#define P10					GPIO_PIN_10
#define P11					GPIO_PIN_11
#define P12					GPIO_PIN_12
#define P13					GPIO_PIN_13
#define P14					GPIO_PIN_14
#define P15					GPIO_PIN_15


//引脚模式
#define INPUT				GPIO_MODE_INPUT    
#define PP 					GPIO_MODE_OUTPUT_PP		   
#define OD 					GPIO_MODE_OUTPUT_OD		   
#define AF_PP 				GPIO_MODE_AF_PP    		   
#define AF_OD 				GPIO_MODE_AF_OD    		   
#define AF_INPUT 			GPIO_MODE_AF_INPUT
#define ANALOG				GPIO_MODE_ANALOG            
#define IT_RISING			GPIO_MODE_IT_RISING         
#define IT_FALLING			GPIO_MODE_IT_FALLING        
#define IT_RF				GPIO_MODE_IT_RISING_FALLING 		
#define EVT_RISING			GPIO_MODE_EVT_RISING        
#define EVT_FALLING			GPIO_MODE_EVT_FALLING       
#define EVT_RF 				GPIO_MODE_EVT_RISING_FALLING
#define ANALOG				GPIO_MODE_ANALOG


//速度
#define LOW 	 			GPIO_SPEED_FREQ_LOW   
#define MEDIUM 				GPIO_SPEED_FREQ_MEDIUM
#define HIGH 				GPIO_SPEED_FREQ_HIGH  


//输出模式
#define NOPULL				GPIO_NOPULL  
#define PULLUP 				GPIO_PULLUP  
#define PULLDOWN 			GPIO_PULLDOWN


//外部中断线
#define	EXTI0				EXTI0_IRQn
#define	EXTI1				EXTI1_IRQn
#define	EXTI2				EXTI2_IRQn
#define	EXTI3				EXTI3_IRQn
#define	EXTI4				EXTI4_IRQn
#define	EXTI9_5				EXTI9_5_IRQn
#define	EXTI15_10			EXTI15_10_IRQn

#define PI					3.1415926f
#define	ON					1
#define OFF					0




































































































































































































































#endif
