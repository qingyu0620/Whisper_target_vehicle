#include "CAN.h"
#include "Motor.h"
#include "stdio.h"

FDCAN_HandleTypeDef fdcan_handle;

DJMotor_Feedback* motor_this;
extern DJMotor_Feedback Lf_data, Rf_data, Lb_data, Rb_data;

void FDCAN_Init()
{
	fdcan_handle.Instance = FDCAN1;
	
	fdcan_handle.Init.Mode = FDCAN_MODE_NORMAL;
	fdcan_handle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	fdcan_handle.Init.AutoRetransmission = ENABLE;
	fdcan_handle.Init.TransmitPause = DISABLE;
	
//	fdcan_handle.Init.DataPrescaler = ;
//	fdcan_handle.Init.DataSyncJumpWidth = ;
//	fdcan_handle.Init.DataTimeSeg1 = ;
//	fdcan_handle.Init.DataTimeSeg2 = ;
	
	fdcan_handle.Init.ExtFiltersNbr = 0;
	fdcan_handle.Init.StdFiltersNbr = 1;
	
	fdcan_handle.Init.MessageRAMOffset = 0;
	fdcan_handle.Init.ProtocolException = DISABLE;
	
	fdcan_handle.Init.RxBufferSize = 0;
	fdcan_handle.Init.RxBuffersNbr = 0;
	fdcan_handle.Init.TxBuffersNbr = 0;
	
	fdcan_handle.Init.NominalPrescaler = 4;
	fdcan_handle.Init.NominalSyncJumpWidth = 1;
	fdcan_handle.Init.NominalTimeSeg1 = 36;
	fdcan_handle.Init.NominalTimeSeg2 = 13;
	
	fdcan_handle.Init.RxFifo0ElmtsNbr = 5;
	fdcan_handle.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	fdcan_handle.Init.RxFifo1ElmtsNbr = 0;
	fdcan_handle.Init.RxFifo1ElmtSize = 0;
	
	fdcan_handle.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	fdcan_handle.Init.TxEventsNbr = 0;
	fdcan_handle.Init.TxFifoQueueElmtsNbr = 10;
	fdcan_handle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	
	HAL_FDCAN_Init(&fdcan_handle);
	
	FDCAN_FilterTypeDef can_filter;
	
	can_filter.FilterIndex = 0;
	can_filter.FilterType = FDCAN_FILTER_MASK;
	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter.FilterID1 = 0x200;
	can_filter.FilterID2 = 0x700;
	
	HAL_FDCAN_ConfigFilter(&fdcan_handle, &can_filter);
	
	HAL_FDCAN_ConfigFifoWatermark(&fdcan_handle, FDCAN_CFG_RX_FIFO0, 0);
	
	HAL_FDCAN_ActivateNotification(&fdcan_handle, FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE, 0);
	
	HAL_FDCAN_Start(&fdcan_handle);
}



void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	GPIO_InitTypeDef gpio_initstruct;
	
	if(hfdcan->Instance == FDCAN1)
	{
		__HAL_RCC_FDCAN_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
		PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
		
		HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
		
		gpio_initstruct.Pin = P0 | P1;
		gpio_initstruct.Mode = AF_PP;
		gpio_initstruct.Pull = NOPULL;
		gpio_initstruct.Speed = HIGH;
		gpio_initstruct.Alternate = GPIO_AF9_FDCAN1;
		
		HAL_GPIO_Init(GPIOD, &gpio_initstruct);
		
		HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 5);
		HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
		
		__HAL_FDCAN_ENABLE_IT(&fdcan_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
	}
}


void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&fdcan_handle);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint8_t rx_data[8];
	FDCAN_RxHeaderTypeDef rx_header;
	
	HAL_FDCAN_GetRxMessage(&fdcan_handle, FDCAN_RX_FIFO0, &rx_header, rx_data);
	
	switch(rx_header.Identifier)
	{
		case 0x201:
			motor_this = &Lf_data;
			break;
		case 0x202:
			motor_this = &Rf_data;
			break;
		case 0x203:
			motor_this = &Lb_data;
			break;
		case 0x204:
			motor_this = &Rb_data;
			break;
	}
	
	motor_this -> angle = ((rx_data[0] << 8) | rx_data[1]) * 360.0f / 8191.0f;
	motor_this -> rpm = (rx_data[2] << 8) | rx_data[3];
}


void FDCAN_Motor_SendCurrent(int32_t motor1, int32_t motor2, int32_t motor3, int32_t motor4)
{
	uint8_t tx_data[8];
	
    tx_data[0] = motor1 >> 8;
    tx_data[1] = motor1;
    tx_data[2] = motor2 >> 8;
    tx_data[3] = motor2;
	tx_data[4] = motor3 >> 8;
    tx_data[5] = motor3;
    tx_data[6] = motor4 >> 8;
    tx_data[7] = motor4;
	
	FDCAN_TxHeaderTypeDef tx_header;
	
	tx_header.Identifier = 0x200;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = FDCAN_DLC_BYTES_8;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.MessageMarker = 0;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&fdcan_handle, &tx_header, tx_data);
}
