#ifndef __APP_LIFT
#define __APP_LIFT
#include "app_motor.h"
#include "stdbool.h"
#include "pid.h"
#include "hrtimer.h"

typedef enum
{
    LIFT_OK = 0,
    LIFT_INIT,
    LIFT_CALIBRATING,      // 校准中
    LIFT_POS_TOO_HIGH,     // 位置过高
    LIFT_POS_TOO_LOW,      // 位置过低
    LIFT_TEMPERTURE_ERROR, // 温度错误
    LIFT_COLLAPSE,         // 电机卡死
    LIFT_LOCK,
    LIFT_DEBUG,
    LIFT_CONTROL
} LiftStatusCode_t;

// const int LIFT_LEFT_SIDE = 0;
// const int LIFT_RIGHT_SIDE =1;

typedef struct
{

    M3508_Motor_t motors[4];
    OID_Encoder_t *encoders[2];
    float height;     // 测量到的height当前高度 获取于编码器欧姆特
    float height_set; // 设定的抬升高度
    float height_offset;
    LiftStatusCode_t status; // 用于指示状态
    bool calibrated;

    float velocity;     // 运动速度
    float set_velocity; // 设定运动速度

    float avg_speed[2];
    float avg_current[2];
    float avg_pos[2];

    PID_t current_pid[2];
    PID_t speed_pid[2];
    PID_t pos_pid;

    int16_t motor_cmd_current[4];

    hrt_abstime last_update_t;
    float dt;
} Lift_t;

int lift_init(Lift_t *lift); // 程序初始化

int lift_calibrate(Lift_t *lift); // 校准

int lift_check(Lift_t *lift); // lift 错误检查

int lift_update_data(Lift_t *lift); // 更新lift数据

int lift_control(Lift_t *lift);

void lift_debug(Lift_t *lift);

#endif