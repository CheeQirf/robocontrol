#ifndef __APP_485_H
#define __APP_485_H

#include <stdint.h>
#include <stdbool.h> 
#include "app_motor.h" 
#include "FreeRTOS.h"
#include "queue.h"

#define UNITREE_TX_FRAME_SIZE       17      // 控制帧大小为 17 字节
#define UNITREE_RX_FRAME_SIZE       16      // 反馈帧大小为 16 字节

extern UnitreeMotor_Measure_t motor_arm[2]; // 对应两个宇树电机

// FreeRTOS 队列用于 ISR 到任务的数据传递 (在 app_485.c 中定义)
extern xQueueHandle xUnitreeRxQueue;

// 函数原型
void app_485_Init(void); // 初始化 RS485 任务和队列
void Unitree_RS485_Rx_Task(void *pvParameters); // 宇树电机接收处理任务
void Unitree_RS485_Send_Command(uint8_t motor_id, int16_t tau_set, int16_t omega_set, int32_t theta_set, int16_t Kp_pos, int16_t Kp_spd, uint8_t motor_mode);
UnitreeMotor_Measure_t* get_unitree_motor_measure_ptr(uint8_t i); // 获取宇树电机测量数据的指针

uint16_t unitree_crc16(const uint8_t *data, uint32_t len);

#endif // __APP_485_H