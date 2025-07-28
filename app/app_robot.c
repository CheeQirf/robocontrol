#include "app_robot.h"
#define LOG_TAG "Robot"
#include "elog.h"


Robot_t Robot;



int robot_init(Robot_t *robot)
{
    robot->arm = robot_get_arm();
    robot->joint = robot_get_joint();
    robot->lift = robot_get_joint();
    int ret = 0;
    ret = arm_init(robot->arm);
    while(ret){
        log_e("error while init arm");
    }
    ret = lift_init(robot->lift);
    while(ret){
        log_e("error while init lift");
    }
    ret = joint_init(robot->joint);
    while(ret){
        log_e("error wihle init joint");
    }



    log_i("robot init succesffuly");

    robot->status = ROBOT_INITED;

    return ret;

}

int robot_recv_command(Robot_t * robot)
{
    //接收到calibrated
    if(   && robot->calibrated==false)
    {
        robot->status == ROBOT_CALIBRATE;
    }

}

int robot_check(Robot_t* robot)
{
    int ret =0 ;
    ret = arm_check(robot->arm);
    ret = lift_check(robot->lift);
    ret = joint_check(robot->joint);
}

int robot_control(Robot_t *robot)
{
    if(robot->status==ROBOT_DEBUG){
        arm_debug(robot->arm);
        lift_debug(robot->lift);
        joint_debug(robot->joint);

    }
    if(robot->status ==ROBOT_INITED){
        return 0; 
    }
    if(robot->status == ROBOT_CALIBRATE)//校准模式
    {
        if(robot->calibrated == true)
        {
            robot->status == ROBOT_LOCK;
            log_w("robot has calibreated once!");
        }else{
            int ret=0;
            //这里是校准顺序： 我这里假设是先lift 后 joint 最后arm
            //蜂鸣器响 and delay
            //TODO buzz
            
            
            if(robot->lift->calibrated == false){
                robot->lift->status=LIFT_CALIBRATING;
                ret = lift_calibrate(robot->lift);
            }
            if(robot->lift->calibrated == true && robot->joint->calibrated ==false){
                robot->lift->status=LIFT_LOCK;
                robot->joint->status=JOINT_CALIBRATING;
                ret =  joint_calibrate(robot->joint);
            }
            if(robot->arm->calibrated==false && robot->lift->calibrated==true && robot->joint->calibrated==true){
                robot->joint->status=JOINT_HOLD;
                robot->arm->status=ARM_CALIBRATING;
                ret =  arm_calibrate(robot->arm);
            }

            if(robot->arm->calibrated==true && robot->lift->calibrated==true && robot->joint->calibrated==true)
            {
              
                log_i("robot calibrated");
                robot->status=ROBOT_LOCK;
                return 0;
            }
            robot->last_status = ROBOT_CALIBRATE;
            return ret;
        }
    }

    if(robot->status != robot->last_status){
        //reset pid interg 归零积分 防止有问题
    }

    if (robot->status == ROBOT_LOCK) //这是闭环控制锁住模式 就是校准完所有电机保证原位不动了
    {
        /* code */
        robot->arm->status == ARM_LOCK;
        robot->joint->status == JOINT_HOLD;
        robot->lift->status == LIFT_LOCK;

        arm_control(robot->arm);
        lift_control(robot->lift);
        joint_control(robot->joint);

        robot->last_status == ROBOT_LOCK;
    }
    if(robot->status == ROBOT_CONTROL) //真正的控制模式
    {
    

    
        


    }
}

void robot_loop(void* param)
{
    Robot_t* robot = &Robot;
    
    robot_init(robot);
    for(;;)
    {
        int ret = 0;
        ret = robot_check(robot);
        
        ret = robot_recv_command(robot);

        ret = robot_control(robot);


    }




}