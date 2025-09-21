#ifndef	__CAN_H__
#define	__CAN_H__

#include "sys.h"
#include "User_Define.h"

void FDCAN_Init(void);
void FDCAN_Motor_SendCurrent(int32_t motor1, int32_t motor2, int32_t motor3, int32_t motor4);
void FDCAN_Motor_Stop(void);

#endif
