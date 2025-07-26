#ifndef __APP_MOTOR
#define __APP_MOTOR


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

//typedef struct {
//		const motorMeasure_t * measure;
//    uint16_t position;         // 角度或位置
//    int16_t velocity;          // 速度
//    int16_t torque;            // 力矩/电流
//    uint8_t error_code;        // 错误码
//		int8_t temperature;          // 电机ID (1-2)
//		float speed_set;  
//    int pos_set; 
//} Go_M8010_6_Motor_t;

typedef struct
{
    uint8_t  id;            // 目标电机 ID
    uint8_t  status;        // 电机工作模式
    int16_t  tau_fbk;       // 实际输出转矩 (原始值)
    int16_t  omega_fbk_raw; // 实际输出速度 (原始值)
    int32_t  theta_fbk_raw; // 实际输出位置 (原始值)
    uint8_t  temp;          // 电机温度
    uint8_t  merror;        // 电机错误标识
    uint16_t force_raw;     // 足端力 (原始数据，12bit)

    // 转换为实际物理量的值
    float    tau_N_m;       // 实际输出转矩 (N.m) = tau_fbk / 256
    float    omega_rad_s;   // 实际输出速度 (rad/s) = omega_fbk_raw / 256
    float    theta_rad;     // 实际输出位置 (rad) = theta_fbk_raw / 32768 * 2 * PI

} UnitreeMotor_Measure_t;

typedef struct{

	uint32_t encoder_count;
	int32_t encoder_rads_count;

}OID_Encoder_t;




#endif