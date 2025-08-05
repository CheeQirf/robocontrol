#ifndef __APP_MOTOR
#define __APP_MOTOR

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint16_t angle;      // 转子角度 实际是编码器的值
    int16_t rpm;         // 当前测量速度
    int16_t current;     // 测量电流
    uint16_t last_angle; // 上一次的角度 用来积分
    int pos;             // 积分的角度
    uint8_t temperture;  // 温度
} motorMeasure_t;        // can接收到的数据

typedef struct
{
    const motorMeasure_t *measure; // 用于保存测量值
    float speed_set;               // 设定速度
    int pos_set;                   // 设定角度
    int16_t current_set;           // 设定电流

} M3508_Motor_t;

typedef struct
{
    const motorMeasure_t *measure; // 保存测量值
    float speed_set;               // 设定速度
    int pos_set;                   // 设定角度
    int16_t current_set;           // 设定电流

} M2006_Motor_t;

// typedef struct
// {
//     uint8_t id;            // 目标电机 ID
//     uint8_t status;        // 电机工作模式
//     int16_t tau_fbk;       // 实际输出转矩 (原始值)
//     int16_t omega_fbk_raw; // 实际输出速度 (原始值)
//     int32_t theta_fbk_raw; // 实际输出位置 (原始值)
//     uint8_t temp;          // 电机温度
//     uint8_t merror;        // 电机错误标识
//     uint16_t force_raw;    // 足端力 (原始数据，12bit)

//     // 转换为实际物理量的值
//     float tau_N_m;     // 实际输出转矩 (N.m) = tau_fbk / 256
//     float omega_rad_s; // 实际输出速度 (rad/s) = omega_fbk_raw / 256
//     float theta_rad;   // 实际输出位置 (rad) = theta_fbk_raw / 32768 * 2 * PI

// } UnitreeMotor_Measure_t;

typedef struct
{
    float theta_rad;   // 实际输出位置 (rad), 逆时针为正
    float omega_rad_s; // 实际输出速度 (rad/s), 逆时针为正
    float tau_N_m;     // 实际输出转矩 (N.m)
    uint8_t temp;      // 电机温度 (°C)
    uint8_t merror;    // 电机错误标识, 0为正常

    // -- 时序和有效性 --
    uint32_t last_update_time; // 上次成功接收到数据的时间戳
    bool data_is_valid;        // 当前数据是否有效（通信成功且CRC正确）

} UnitreeMotor_Measure_t;

typedef struct
{
    uint8_t id;                            // 电机ID
    const UnitreeMotor_Measure_t *measure; // 指向对应的测量数据结构体

    // --- 控制指令 (Setpoints) ---
    uint8_t mode_set; // 目标工作模式 (0:失能, 10:位置模式)
    float pos_set;    // 目标位置 (rad)
    float speed_set;  // 目标速度 (rad/s) (在位置模式下作为速度限制)
    float kp_set;     // 位置环增益 Kp (0.0-25.5)
    float kw_set;     // 速度环增益 Kw (0.0-25.5)
    float torque_set; // 目标力矩 (N.m) (前馈)

} UnitreeMotor_t;

typedef struct
{

    uint32_t encoder_count;
    int32_t encoder_rads_count;

} OID_Encoder_t;

#endif