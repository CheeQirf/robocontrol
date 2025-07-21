#ifndef __APP_ARM
#define __APP_ARM
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
    ARM_COLLAPSE

}ArmStatus_t;
typedef enum 
{
    ARM_FREE = 0,
    ARM_LOCK,
    

}ArmControlMode_t;



typedef struct 
{
    M2006_Motor_t close_motor[4];
    M2006_Motor_t wrist_motor[2]; //用于手臂旋转电机，控制效果只需要锁住即可
    M3508_Motor_t stretch_motor;  //伸缩机械臂电机
    float stretch; // 机械臂伸缩量 m
    /*
           ***--------------------------------------
        |       |
    |               |
    |               |
    竖线代表爪子， 上面的横线就是伸缩量
    */
    float angle_wrist; //手臂翻腕角度 弧度制
    //同样上面那个图 爪子 现在是朝向地面的 但是可以朝向其他地方 就是这个
    float angle_closed;; //手的闭合量
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
  //爪子闭合的状态指示位
    bool closed;
    bool calibrated;
  
    //下面就是相应的控制量
    float angle_wrist_set;
    float angle_closed_set;
    float stretch_set;
    ArmStatus_t status;
    ArmControlMode_t mode;
    int encoder_offset; //偏移量of角度


}Arm_t;

// ! 详细函数说明查看C文件

int arm_init(Arm_t *arm);//初始化 

int arm_update_data(Arm_t *arm);//根据测量数据更新单独arm数据


int arm_check(Arm_t *arm); //机械臂check 就是看有没有异常情况 C文件里有个static方法对应异常处理

int arm_calibrate(Arm_t *arm); //机械臂校准 上电归位



#endif