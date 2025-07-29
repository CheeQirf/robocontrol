#ifndef __APP_ARM
#define __APP_ARM
#include "hrtimer.h"


#include "app_motor.h"
#include "app_lift.h"
#include "app_joint.h"
#include "stdbool.h"
typedef enum 
{

    ARM_OK = 0 ,
    ARM_INIT,
    ARM_CALIBRATING,
    ARM_ERROR,
    ARM_COLLAPSE,
    ARM_LOCK,
    ARM_CONTROL,

}ArmStatus_t;

typedef enum 
{

    CLOSE_ANGLE=0x01,
    STRETCH_SPEED=0x02,
    STRETCH_POS=0x04

}ArmControlMode_t;


typedef struct 
{

    //电机定义
    M2006_Motor_t close_motor[4];
    M2006_Motor_t wrist_motor[2]; //用于手臂旋转电机，控制效果只需要锁住即可
    M3508_Motor_t stretch_motor;  //伸缩机械臂电机


    //arm定义
    bool calibrated;
    ArmStatus_t status;
    ArmControlMode_t mode;
    hrt_abstime last_update_t;
    float dt;


    //下面是抓取部分定义
    bool closed;
    bool close_calibrated[2];
    float close_angle_offset[2];
    float avg_close_angle[2]; //测量量 两组 分别是左右 一组有两个电机平均值
    float avg_close_speed[2];
    float avg_close_current[2];
    PID_t close_current_pid[2];
    PID_t close_speed_pid[2];
    PID_t close_angle_pid[2];
    float angle_closed_set; //设置量
		int16_t	close_control_current[2];
        /*
                | 爪子的中轴线
                |
                |
                |     
            |   |    |
        |       |       |
        |       |        |
        |       |        |
                |
                |------------> 爪子闭合的程度 爪子与中轴线夹角就是这个变量
                |
    closed_quality = 0 即为 爪子完全闭合
    closed quality的量 就是爪子张开的角度
    初始化校准把 这个量找到零
    */

    //手腕部分定义 直接锁住
    PID_t wrist_speed_pid[2];

    PID_t wrist_current_pid[2]; //电流环 + 位置环 不要速度环了


    //伸缩部分
    float stretch_set;
    float stretch; //meter unit
    float stretch_speed;
    float stretch_speed_set;
    PID_t stretch_pos_pid;
    PID_t stretch_speed_pid;
    PID_t stretch_current_pid;
    //电流环 + 速度环 + 位置环

    
}Arm_t;

// ! 详细函数说明查看C文件

int arm_init(Arm_t *arm);//初始化 

int arm_update_data(Arm_t *arm);//根据测量数据更新单独arm数据


int arm_check(Arm_t *arm); //机械臂check 就是看有没有异常情况 C文件里有个static方法对应异常处理

int arm_calibrate(Arm_t *arm); //机械臂校准 上电归位

int arm_control(Arm_t *arm);

void arm_debug(Arm_t *arm);

#endif