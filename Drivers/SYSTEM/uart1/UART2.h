#ifndef	__UART2_H__
#define	__UART2_H__

#include "sys.h"
#include "User_Define.h"

#define UART_TXD2_PIN				P2
#define UART_TXD2_PORT				GPIOA
#define UART_TXD2_CLK()				GPIOA_CLK()

#define UART_RXD2_PIN				P3
#define UART_RXD2_PORT				GPIOA
#define UART_RXD2_CLK()				GPIOA_CLK()

#endif
