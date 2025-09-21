#include "UART1.h"
#include "IBUS.h"
#include "stdio.h"
#include "string.h"

UART_HandleTypeDef uart10_handle;
DMA_HandleTypeDef dma10rx_handle;

uint8_t uart10_buffer1[RX_BUF_SIZE] = {0};
uint8_t uart10_buffer2[RX_BUF_SIZE] = {0};
uint8_t *uart10_current_buf = uart10_buffer1;
uint8_t *uart10_ready_buf = NULL;


/**
************************************************************************
* @brief:       UART10_Init()
* @param:       void
* @retval:      void
* @details:    	初始化UART10
************************************************************************
**/
void UART10_Init()
{
	uart10_handle.Instance = USART10;
	uart10_handle.Init.BaudRate = 115200;
	uart10_handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	uart10_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart10_handle.Init.Mode = UART_MODE_TX_RX;
	uart10_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart10_handle.Init.OverSampling = UART_OVERSAMPLING_16;
	uart10_handle.Init.Parity = UART_PARITY_NONE;
	uart10_handle.Init.StopBits = UART_STOPBITS_1;
	uart10_handle.Init.WordLength = UART_WORDLENGTH_8B;
	HAL_UART_Init(&uart10_handle);
	
	
	dma10rx_handle.Instance = DMA2_Stream3;
    dma10rx_handle.Init.Request = DMA_REQUEST_USART10_RX;
    dma10rx_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma10rx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    dma10rx_handle.Init.MemInc = DMA_MINC_ENABLE;
    dma10rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma10rx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma10rx_handle.Init.Mode = DMA_CIRCULAR;
    dma10rx_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
    dma10rx_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&dma10rx_handle);
	
	__HAL_LINKDMA(&uart10_handle, hdmarx, dma10rx_handle);
	
	HAL_UART_Receive_DMA(&uart10_handle, uart10_current_buf, RX_BUF_SIZE);
}



void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef gpio_initstruct;
	
	if(huart->Instance == USART10)
	{
		DMA2_CLK();
		UART10_CLK();
		GPIOE_CLK();
		
		gpio_initstruct.Pull = PULLUP;
		gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		gpio_initstruct.Mode = AF_PP;
		
		gpio_initstruct.Alternate = GPIO_AF4_USART10;
		gpio_initstruct.Pin = P2;
		HAL_GPIO_Init(GPIOE, &gpio_initstruct);
		
		gpio_initstruct.Alternate = GPIO_AF11_USART10;
		gpio_initstruct.Pin = P3;
		HAL_GPIO_Init(GPIOE, &gpio_initstruct);
		
		HAL_NVIC_SetPriority(USART10_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART10_IRQn);

		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	}
}



void USART10_IRQHandler()
{
	if(__HAL_UART_GET_FLAG(&uart10_handle, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&uart10_handle);
		
		HAL_UART_DMAStop(&uart10_handle);
		if(uart10_current_buf == uart10_buffer1)
		{
			uart10_current_buf = uart10_buffer2;
			uart10_ready_buf = uart10_buffer1;
		}
		else
		{
			uart10_current_buf = uart10_buffer1;
			uart10_ready_buf = uart10_buffer2;
		}
		
		Remote_IBUS_to_RC(uart10_ready_buf);
		
		memset(uart10_ready_buf, 0, RX_BUF_SIZE);
		
		HAL_UART_Receive_DMA(&uart10_handle, uart10_current_buf, RX_BUF_SIZE);
	}
}



#if 1         
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;


void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{ 	
	while((USART10->ISR & 0X40) == 0);
	USART10->TDR = (uint8_t)ch;      
	return ch;
}


#endif 
