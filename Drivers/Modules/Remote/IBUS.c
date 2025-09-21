#include "IBUS.h"
#include "UART1.h"
#include "stdio.h"

Remote_a8x remote;


/**
* @brief	Remote_Init()
* @param	void
* @retval	void
* @details	初始化串口10
**/
void Remote_Init()
{
	UART10_Init();
}


/**
* @brief	Remote_IBUS_to_RC(uint8_t* array)
* @param	array: 传入IBUS发送的数据
* @retval	void
* @details	解析IBUS数据
**/
void Remote_IBUS_to_RC(uint8_t* array)
{
	uint16_t channel[8];

	channel[0] = ((array[3] << 8) | array[2]) & 0x07FF;
	channel[1] = ((array[5] << 8) | array[4]) & 0x07FF;
	channel[2] = ((array[7] << 8) | array[6]) & 0x07FF;
	channel[3] = ((array[9] << 8) | array[8]) & 0x07FF;
	//SWA
	channel[4] = ((array[11] << 8) | array[10]) & 0x07FF;
//	channel[5] = ((array[13] << 8) | array[12]) & 0x07FF;
	//VRA
	channel[6] = ((array[15] << 8) | array[14]) & 0x07FF;
	//SWC
	channel[7] = ((array[17] << 8) | array[16]) & 0x07FF;

	//将X, Y, R通道值归一化
	remote.X = 0.002 * channel[1] - 3;
	remote.Y = 0.002 * channel[3] - 3;
	remote.R = 0.002 * channel[0] - 3;

	//左摇杆上下通道待定
//	remote.ch3 = 0.002 * channel[2] - 3;
	
	remote.swa = channel[4];
	remote.swc = channel[7];
	remote.vra = channel[6];
}


