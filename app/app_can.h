#ifndef __APP_CAN
#define __APP_CAN
#include "bsp_can.h"

#include "main.h"
void CAN_Rcv_DateFromISR(CanMessage_t *RxMsg);
typedef struct {
    uint16_t rotor_mech_angle; // 转子机械角度 (DATA[0]高8位 | DATA[1]低8位)
    int16_t rotor_speed;       // 转子转速 (DATA[2]高8位 | DATA[3]低8位)
    int16_t actual_torque_current; // 实际转矩电流 (DATA[4]高8位 | DATA[5]低8位)
    int8_t motor_temperature;  // 电机温度 (DATA[6]是8位，所以用int8_t或uint8_t)
    uint8_t motor_id;          // 电机ID (1-5)
} M3508_Motor_t;
typedef struct {
    uint16_t rotor_mech_angle; 
    int16_t rotor_speed;       
    int16_t actual_torque_current; 
    uint8_t motor_id;     //   电机ID (1-6)  
} M2006_Motor_t;

typedef struct {
    uint16_t position;         // 角度或位置
    int16_t velocity;          // 速度
    int16_t torque;            // 力矩/电流
    uint8_t error_code;        // 错误码
    uint8_t motor_id;          // 电机ID (1-2)

} Go_M8010_6_Motor_t;

#define MAX_M3508_MOTORS 5
#define MAX_M2006_MOTORS 6
#define MAX_GO8010_6_MOTORS 2

extern M3508_Motor_t g_m3508_motors[MAX_M3508_MOTORS];
extern M2006_Motor_t g_m2006_motors[MAX_M2006_MOTORS];
extern Go_M8010_6_Motor_t g_go8010_6_motors[MAX_GO8010_6_MOTORS];

#endif
