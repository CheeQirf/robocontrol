#ifndef __APP_CAN
#define __APP_CAN
#include "app_motor.h"
#include "bsp_can.h"

#include "main.h"
void CAN_Rcv_DataFromISR(CanMessage_t *RxMsg);


/*
获取电机测量值的 指针 为什么要这么做？ 防止初始化问题 让测量结构体先初始化 这样can接收会空指针的错误
*/

motorMeasure_t* get_motor_arm_measure_ptr(uint8_t i);

motorMeasure_t* get_motor_lift_measure_ptr(uint8_t i);

motorMeasure_t* get_motor_stretch_measure_ptr(void);

motorMeasure_t* get_motor_wrist_measure_ptr(uint8_t i);

OID_Encoder_t * get_encoder_measuer_ptr(uint8_t i);



#endif
