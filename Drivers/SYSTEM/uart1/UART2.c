#include "UART2.h"

UART_HandleTypeDef uart2_handle;

void UART2_Init(uint32_t baudrata)
{
	uart2_handle.Instance = USART2;
	uart2_handle.Init.BaudRate = baudrata;
	uart2_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart2_handle.Init.Mode = UART_MODE_TX_RX;
	uart2_handle.Init.OverSampling = UART_OVERSAMPLING_16;
	uart2_handle.Init.Parity = UART_PARITY_NONE;
	uart2_handle.Init.StopBits = UART_STOPBITS_1;
	uart2_handle.Init.WordLength = UART_WORDLENGTH_8B;
	HAL_UART_Init(&uart2_handle);
}

