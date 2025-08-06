#include "app_joint.h"
#include "bsp_unitree.h"

Joint_t Joint;
static uint8_t joint_motor_poll_index = 0;
int joint_init(Joint_t *joint)
{
    joint->status = JOINT_INIT;

    for (int i = 0; i < 2; ++i)
    {
        unitree_motor_init(&joint->motor[i], i, get_unitree_motor_measure_ptr(i));
        joint->motor[i].pos_set = 0;
        joint->motor[i].speed_set = 0;
        joint->motor[i].torque_set = 0;
    }
    return 0;
}

int joint_update_data(Joint_t *joint)
{
    if (joint->motor[0].measure->data_is_valid && joint->motor[1].measure->data_is_valid)
    {
        joint->current_angle = (joint->motor[0].measure->theta_rad - joint->motor[1].measure->theta_rad) / 2.0f;
        joint->angular_velocity = (joint->motor[0].measure->omega_rad_s - joint->motor[1].measure->omega_rad_s) / 2.0f;
    }
    return 0;
}
int joint_check(Joint_t *joint)
{
    joint_update_data(joint);
}
void joint_debug(Joint_t *joint)
{
    // 设置电机0的控制参数
    joint->motor[0].mode_set = 10;
    joint->motor[0].speed_set = 0.1;
    joint->motor[0].kp_set = 0.1;
    joint->motor[0].kw_set = 0.1;
    joint->motor[0].pos_set = 0.0f;    // 在速度模式下，位置应设为0
    joint->motor[0].torque_set = 0.0f; // 前馈力矩为0
}
int joint_control(Joint_t *joint)
{
    if (joint->status == JOINT_DEBUG)
    {
        joint_debug(joint);
    }
    else if (joint->status == JOINT_HOLD)
    {
        }
    else if (joint->status == JOINT_CONTROL)
    {
    }
    // 阻塞发送
    // unitree_motor_control_send(&joint->motor[0]);
    // unitree_motor_control_send(&joint->motor[1]);
    // 非阻塞发送
    if (unitree_motor_control_start(&joint->motor[joint_motor_poll_index]))
    {
        joint_motor_poll_index = (joint_motor_poll_index + 1) % 2;
    }
    return 0;
}