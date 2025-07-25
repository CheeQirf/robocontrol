#ifndef BSP_RS485_H
#define BSP_RS485_H

#include "stm32f4xx_hal.h" 

// RS485_1 (对应 USART1，方向控制引脚 PC15)
#define RS485_1_DIR_PORT      GPIOC
#define RS485_1_DIR_PIN       GPIO_PIN_15

// RS485_2 (对应 USART2，方向控制引脚 PB3)
#define RS485_2_DIR_PORT      GPIOB
#define RS485_2_DIR_PIN       GPIO_PIN_3

// RS485_1 方向控制宏
// PC15 高电平开启发送
#define RS485_1_TRANSMIT_MODE()    HAL_GPIO_WritePin(RS485_1_DIR_PORT, RS485_1_DIR_PIN, GPIO_PIN_SET)
// PC15 低电平开启接收
#define RS485_1_RECEIVE_MODE()     HAL_GPIO_WritePin(RS485_1_DIR_PORT, RS485_1_DIR_PIN, GPIO_PIN_RESET)

// RS485_2 方向控制宏
// PB3 高电平开启发送
#define RS485_2_TRANSMIT_MODE()    HAL_GPIO_WritePin(RS485_2_DIR_PORT, RS485_2_DIR_PIN, GPIO_PIN_SET)
// PB3 低电平开启接收
#define RS485_2_RECEIVE_MODE()     HAL_GPIO_WritePin(RS485_2_DIR_PORT, RS485_2_DIR_PIN, GPIO_PIN_RESET)

// 函数声明
// 注意：非阻塞发送函数，不再有 Timeout 参数，因为是立即返回
HAL_StatusTypeDef RS485_1_Transmit_IT(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef RS485_2_Transmit_IT(uint8_t *pData, uint16_t Size);

void RS485_Init(void); // 用于初始化所有RS485端口为默认模式并启动接收

#endif // BSP_RS485_H