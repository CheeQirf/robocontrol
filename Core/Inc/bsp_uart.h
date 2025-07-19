#ifndef __BSP_UART 
#define __BSP_UART
#include "main.h"


void USART4_Init(void);
void USART4_SendData(const uint8_t *p_data, uint16_t len);
#endif


