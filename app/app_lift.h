#ifndef __APP_LIFT
#define __APP_LEFT
#include "app_motor.h"
#include "stdbool.h"
typedef enum{
    LIFT_OK = 0,
    LIFT_INIT,
    LIFT_CALIBRATING, //校准中
    LIFT_POS_TOO_HIGH, //位置过高
    LIFT_POS_TOO_LOW, //位置过低
    LIFT_TEMPERTURE_ERROR,  //温度错误
    LIFT_COLLAPSE, //电机卡死
}LiftStatusCode_t;


typedef struct{

    M3508_Motor_t motors[4];
    OAIDI_Encoder_t* encoders[2];
    float height;  // 测量到的height当前高度 获取于编码器欧姆特
    float height_set; //设定的抬升高度
    LiftStatusCode_t status; //用于指示状态
    bool calibrated;

    float velocity ; //运动速度
    float set_velocity; //设定运动速度

}Lift_t;

int lift_init(Lift_t *lift);//程序初始化

int lift_calibrate(Lift_t *lift);//校准

int lift_check(Lift_t *lift); //lift 错误检查

int lift_update_data(Lift_t* lift); //更新lift数据


#endif