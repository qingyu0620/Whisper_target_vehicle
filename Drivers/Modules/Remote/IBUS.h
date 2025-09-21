#ifndef	__IBUS_H__
#define	__IBUS_H__

#include "sys.h"
#include "User_Define.h"

#define Remote_On	2000
#define Remote_Off	1000


typedef struct
{
	float X;
	float Y;
	float R;

	uint16_t swa;
	uint16_t swc;
	uint16_t vra;
}Remote_a8x;


void Remote_Init(void);
void Remote_IBUS_to_RC(uint8_t* array);

#endif
