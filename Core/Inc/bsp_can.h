#ifndef __BSP_CAN
#define __BSP_CAN
#include "main.h"
#include "stdbool.h"

typedef struct 
{
    /* data */
    bool ide;        // 是否为扩展帧 true为扩展帧
    uint8_t data[8]; // 存放can数据
    uint8_t dlc;
    uint32_t id;     // 存放id号
    uint8_t can_num; // 是哪个CAN发过来的 can1 还是can2 里面就是数字1 2
}CanMessage_t;

void CAN_Init(void);
HAL_StatusTypeDef CAN_Transmit(CanMessage_t *msg);
#endif
