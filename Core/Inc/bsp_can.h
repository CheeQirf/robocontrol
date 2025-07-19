#ifndef __BSP_CAN 
#define __BSP_CAN
#include "main.h"

void CAN_Init(void);
uint8_t CAN_Transmit(const void* buf, uint32_t len, uint8_t number);



#endif

