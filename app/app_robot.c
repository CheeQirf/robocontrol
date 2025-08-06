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
    robot->arm = robot_get_arm();
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

int robot_check(Robot_t *robot)
{

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
static void robot_update_subsystem_states(Robot_t *robot)
{
    // 默认情况下，所有子系统都处于锁定状态
    ArmStatus_t target_arm_status = ARM_LOCK;
    LiftStatusCode_t target_lift_status = LIFT_LOCK;
    JointStatus_t target_joint_status = JOINT_HOLD;

    // 根据主状态进行分发
    switch (robot->status)
    {
    case ROBOT_CONTROL:
        target_arm_status = ARM_CONTROL;
        target_lift_status = LIFT_CONTROL;
        target_joint_status = JOINT_CONTROL;
        break;

    case ROBOT_DEBUG:
        target_arm_status = ARM_DEBUG;
        target_lift_status = LIFT_DEBUG;
        target_joint_status = JOINT_DEBUG;
        break;

    case ROBOT_LOCK:
        // 使用默认的锁定状态
        break;
    }

    robot->arm->status = target_arm_status;
    robot->lift->status = target_lift_status;
    robot->joint->status = target_joint_status;
}
int robot_control(Robot_t *robot)
{
    robot_update_subsystem_states(robot);
    if (robot->status != robot->last_status)
    {
        for (int i = 0; i < 2; i++)
        {
            pid_reset_integral(&robot->arm->close_angle_pid[i]);
            pid_reset_integral(&robot->arm->close_speed_pid[i]);
            pid_reset_integral(&robot->arm->close_current_pid[i]);

            pid_reset_integral(&robot->arm->wrist_angle_pid[i]);
            pid_reset_integral(&robot->arm->wrist_speed_pid[i]);
            pid_reset_integral(&robot->arm->wrist_current_pid[i]);
        }

        pid_reset_integral(&robot->arm->stretch_pos_pid);
        pid_reset_integral(&robot->arm->stretch_speed_pid);
        pid_reset_integral(&robot->arm->stretch_current_pid);

        pid_reset_integral(&robot->lift->pos_pid);

        for (int i = 0; i < 2; i++)
        {
            pid_reset_integral(&robot->lift->speed_pid[i]);
            pid_reset_integral(&robot->lift->current_pid[i]);
        }
    }
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

        robot_control(robot); // 最终的实际控制函数

        osDelay(1);
    }
}
