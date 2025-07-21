#include "app_arm.h"
#include "app_motor.h"
#include "app_can.h"
//全局变量：
motorMeasure_t arm_m2006_measure[6]; //6个电机测量值
motorMeasure_t arm_m3508_measure;    //单个3508 测量值 在手臂部分的


Arm_t Arm;
/*
    机械臂初始化 ， 结构体变量的初始化等等
    这个初始化针对于程序的初始化 不是现实世界初始化 现实世界的是calibrate

*/
int arm_init(Arm_t *arm)
{

    if (arm == NULL)
    {
        /* code */
        return 1;
    }
    
    for(int i = 0; i < 4 ; ++i)
    {
        arm->close_motor[i].measure = get_motor_arm_measure_ptr(i);
    }
    for(int i = 0 ;i <2 ; ++i)
    {
        arm->wrist_motor[i].measure = get_motor_wrist_measure_ptr(i);
    }
    arm->stretch_motor.measure = get_motor_stretch_measure_ptr(i);

    //PID init

    arm->status = ARM_INIT;
    arm->mode   = ARM_LOCK;

    return arm_update_data(arm);
}
/*
    校准 ： 上电的时候 回到正确位置

*/
int arm_calibrate(Arm_t *arm)
{



}
/*
        根据M3508 和M2006测量数据 更新Arm的数据
        这里只是更新Arm的数据 比如说抓握的角度等等
        
*/
int arm_update_data(Arm_t *arm)
{
    

}

/*
    根据其他连杆结构更新Arm在3维空间的位置
    什么意思呢？
    就是下面抬升机构动会影响 arm的位置， arm也随之上升
    宇树电机动 arm的旋转角度会改变 arm的位置也会有个弧度变化
    要根据所有连杆机构的数据 更新Arm的数据

    我先在这里列一个简单公式
    

*/


int arm_check(Arm_t *arm)
{



} 




static int arm_error_solve(Arm_t *arm)
{
    


}