#include "app_robot.h"
#define LOG_TAG "Robot"
#include "elog.h"
#include "cmsis_os.h"

Robot_t Robot;
extern Arm_t Arm;
extern Lift_t Lift;
extern Joint_t Joint;

Arm_t *robot_get_arm()
{
    return &Arm;
}
Joint_t *robot_get_joint()
{
    return &Joint;
}
Lift_t *robot_get_lift()
{
    return &Lift;
}

int robot_init(Robot_t *robot)
{
    robot->arm = robot_get_arm(); // 获取全局变量 结构体Arm
    robot->joint = robot_get_joint();
    robot->lift = robot_get_lift();

    int ret = 0;
    // pid初始化 获取回馈can帧的指针 ARM的状态设置为ARM_INIT 设置没有校准 然后把一些ARM结构体成员变量初始化0
    ret = arm_init(robot->arm);
    while (ret)
    {
        log_e("error while init arm");
    }
    log_i("arm init succesffuly\n");
    ret = lift_init(robot->lift);
    while (ret)
    {
        log_e("error while init lift");
    }
    ret = joint_init(robot->joint);
    while (ret)
    {
        log_e("error wihle init joint");
    }

    log_i("robot init succesffuly");

    robot->status = ROBOT_INITED; // 最后会设置为ROBOT_INITED

    return 0;
}

int robot_recv_command(Robot_t *robot)
{
    // 接收到calibrated
    // 例如：if (robot->command_received.type == COMMAND_CALIBRATE && robot->calibrated == false)
    if (1 && robot->calibrated == false) // ？？？？？
    {
        robot->status = ROBOT_CALIBRATE;
    }
}

int robot_check(Robot_t *robot)
{

    // 1. 把mavlink接收到的控制值赋给相应的量
    //
    // uint8_t flag; // 作用是控制模式 他有最高的优先级 1.校准 2.伸长手臂 3.运球 4.投球
    // float stretch_speed;
    // float stretch_pos;
    // float arm_close_angle;
    // float lift_speed;
    // float lift_pos;
    // float joint_angle;
    // float throw_distance;

    // set_arm_stretch_distance
    // set_arm_close_angle
    // set_arm_stretch_speed

    int ret_arm = arm_check(robot->arm);
    if (ret_arm != 0)
    {
        log_e("Robot check found issues in claw\n");
        return -1;
    }

    int ret_lift = lift_check(robot->lift);
    if (ret_lift != 0)
    {
        log_e("Robot check found issues in lift\n");
        return -1;
    }
    int ret_joint = joint_check(robot->joint);
    if (ret_arm != 0 || ret_lift != 0 || ret_joint != 0)
    {
        log_e("Robot check found issues in subsystems.");
        return -1;
    }
    return 0;
}

int robot_control(Robot_t *robot)
{
    if (robot->status == ROBOT_DEBUG)
    {
        robot->arm->status = ARM_DEBUG;
        robot->lift->status = LIFT_DEBUG;
        robot->joint->status = JOINT_DEBUG;
    }
    else if (robot->status == ROBOT_INITED)
    {
        robot->status = ROBOT_DEBUG;
    }
    else if (robot->status == ROBOT_CALIBRATE) // 校准模式
    {
        if (robot->calibrated == true)
        {
            robot->status = ROBOT_LOCK; // 校准完就会锁住
            log_w("robot has calibreated once!");
        }
        else
        {
            int ret = 0;
            // 这里是校准顺序： 我这里假设是先lift 后 joint 最后arm
            // 蜂鸣器响 and delay
            // TODO buzz

            if (robot->lift->calibrated == false)
            {
                robot->lift->status = LIFT_CALIBRATING;
                // ret = lift_calibrate(robot->lift);
            }
            else if (robot->lift->calibrated == true && robot->joint->calibrated == false)
            {
                robot->lift->status = LIFT_LOCK;
                robot->joint->status = JOINT_CALIBRATING;
                // ret = joint_calibrate(robot->joint);
            }
            else if (robot->arm->calibrated == false && robot->lift->calibrated == true && robot->joint->calibrated == true)
            {
                robot->joint->status = JOINT_HOLD;
                robot->arm->status = ARM_CALIBRATING;
                // ret = arm_calibrate(robot->arm);
            }

            if (robot->arm->calibrated == true && robot->lift->calibrated == true && robot->joint->calibrated == true)
            {
                robot->calibrated = true;
                robot->status = ROBOT_LOCK;
                log_i("robot calibrated successfully");
                return 0;
            }
            // robot->last_status = ROBOT_CALIBRATE;
            //  return ret;
        }
    }
    else if (robot->status == ROBOT_LOCK) // 这是闭环控制锁住模式 就是校准完所有电机保证原位不动了
    {
        /* code */
        robot->arm->status = ARM_LOCK;
        robot->joint->status = JOINT_HOLD;
        robot->lift->status = LIFT_LOCK;

        robot->last_status = ROBOT_LOCK;
    }
    else if (robot->status == ROBOT_CONTROL) // 真正的控制模式
    {
    }

    // if (robot->status != robot->last_status) // 可能说明切换状态了
    // {
    //     // reset pid interg 归零积分 防止有问题
    // }
    arm_control(robot->arm);
    lift_control(robot->lift);
    joint_control(robot->joint);

    robot->last_status = robot->status;
    return 0;
}

void robot_loop(void *param)
{

    Robot_t *robot = &Robot;

    robot_init(robot);
    for (;;)
    {
        robot_check(robot); // 获取来自mavlink的控制值

        // robot_recv_command(robot); // 命令的切换，比如是要校准还是控制，应该也是来自mavlink的消息

        robot_control(robot); // 最终的实际控制函数

        osDelay(1);
    }
}
