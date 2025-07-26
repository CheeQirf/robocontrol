#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include <stdint.h>
#include "stm32f4xx_hal.h" 
#include "gpio.h" 

// RS485_1 (USART1) 的方向控制引脚宏定义
#define RS485_1_DIR_PORT        GPIOC
#define RS485_1_DIR_PIN         GPIO_PIN_15

// RS485_1 方向控制函数
#define RS485_1_TRANSMIT_MODE() HAL_GPIO_WritePin(RS485_1_DIR_PORT, RS485_1_DIR_PIN, GPIO_PIN_SET)
#define RS485_1_RECEIVE_MODE()  HAL_GPIO_WritePin(RS485_1_DIR_PORT, RS485_1_DIR_PIN, GPIO_PIN_RESET)

// RS485_1 接收缓冲区大小和接收帧大小
#define RS485_1_RX_BUFFER_SIZE  16 // 宇树电机反馈帧是 16 字节，直接用这个大小

// 外部声明 UART 句柄 (由 CubeMX 生成并在 usart.h 中声明)
extern UART_HandleTypeDef huart1;

// 函数原型
HAL_StatusTypeDef BSP_RS485_1_Transmit_IT(uint8_t *pData, uint16_t Size);
void BSP_RS485_Init(void); // 只初始化 RS485_1

#endif 