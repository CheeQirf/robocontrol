#ifndef __APP_MOTOR
#define __APP_MOTOR
//#include "stm32f411xe.h"
#include <stdint.h>
typedef struct 
{
    uint16_t angle;  //转子角度 当前 编码器测量的
    int16_t rpm;     //当前测量速度
    int16_t current; //测量电流
    uint16_t last_angle;  //上一次的角度 用来积分
    int pos;    //积分的角度
    uint8_t temperture;  //温度
}motorMeasure_t; //can接收到的数据


typedef struct {
    const motorMeasure_t* measure; //用于保存测量值
    float speed_set;   //设定速度
    int pos_set;     //设定角度
    int16_t current_set;   //设定电流



} M3508_Motor_t;
typedef struct {
    const motorMeasure_t* measure;  //保存测量值
    float speed_set;   //设定速度
    int pos_set;    //设定角度
    int16_t current_set;   //设定电流

} M2006_Motor_t;

typedef struct {
    uint16_t position;         // 角度或位置
    int16_t velocity;          // 速度
    int16_t torque;            // 力矩/电流
    uint8_t error_code;        // 错误码
   // uint8_t motor_id;          // 电机ID (1-2)

} Go_M8010_6_Motor_t;

typedef struct{

	uint32_t encoder_count;


}OAIDI_Encoder_t;




#endif