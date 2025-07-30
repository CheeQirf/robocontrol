#include "app_arm.h"
#include "app_motor.h"
#include "app_can.h"
#include "hrtimer.h"
#include <math.h>
#define LOG_TAG "app_arm"

#include "elog.h"
#include "usart.h"

#define MOTOR_POS_TO_STRETCH 0.0f

#define CALIBRATE_CLOSE_SPEED 0.0f
#define CALIBRATE_CURRENT 0.0f
#define ENCODER_TO_ARM_CLOSE_ANGLE 0.0f
#define RPM_TO_ARM_STRETCH_SPEED 0.0f

static int g_debug_motor_group = 0;   // -1:关闭调试, 0:左爪, 1:右爪, 2:腕0, 3:腕1, 4:伸缩
static int g_debug_mode = 0;          // 0:直接电流, 1:速度环, 2:位置环
static float g_debug_target = 500.0f; // 你的目标值 (e.g., 500mA, 100rpm, 30deg)

const float GRAVITY_COMPENSATION_CURRENT = 1600.0f;

// 全局变量：
// motorMeasure_t arm_m2006_measure[6]; // 6个电机测量值
// motorMeasure_t arm_m3508_measure;    // 单个3508 测量值 在手臂部分的

Arm_t Arm;
/*
    机械臂初始化 ， 结构体变量的初始化等等
    这个初始化针对于程序的初始化 不是现实世界初始化 现实世界的是calibrate

*/

// 4. 设置伸缩电机的位置控制
static void arm_set_stretch_position_control(Arm_t *arm, float target_position);
// 5. 设置伸缩电机的速度控制
static float arm_set_stretch_speed_control(Arm_t *arm, float target_speed);
// 6. 设置伸缩电机的电流控制
static float arm_set_stretch_current_control(Arm_t *arm, float target_current);

// 7. 设置腕部电机的位置控制
static void arm_set_wrist_position_control(Arm_t *arm, int index, float target_angle);
// 8. 设置腕部电机的电流控制
static float arm_set_wrist_current_control(Arm_t *arm, int index, float target_current);

static float arm_calculate_current_from_current(Arm_t *arm, int index, float target_current)
{
    return pid_calculate(&arm->close_current_pid[index], target_current, arm->avg_close_current[index], 0, arm->dt);
}

static float arm_calculate_current_from_speed(Arm_t *arm, int index, float target_speed)
{
    float target_current = pid_calculate(&arm->close_speed_pid[index], target_speed, arm->avg_close_speed[index], 0, arm->dt);
    return arm_calculate_current_from_current(arm, index, target_current);
}

static float arm_calculate_current_from_angle(Arm_t *arm, int index, float target_angle)
{
    float target_speed = pid_calculate(&arm->close_angle_pid[index], target_angle, arm->avg_close_angle[index], 0, arm->dt);
    return arm_calculate_current_from_speed(arm, index, target_speed);
}

void arm_set_close_current(Arm_t *arm, int index, float target_current)
{
    float final_cmd = arm_calculate_current_from_current(arm, index, target_current);
    final_cmd = fmaxf(-M2006_CURRENT_LIMIT, fminf(M2006_CURRENT_LIMIT, final_cmd));
    arm->motor_cmd_current[index] = (int16_t)final_cmd;
    arm->motor_cmd_current[index + 2] = (int16_t)(-final_cmd);
}

void arm_set_close_speed(Arm_t *arm, int index, float target_speed)
{
    float final_cmd = arm_calculate_current_from_speed(arm, index, target_speed);
    // 进行机械臂重力补偿
    final_cmd = final_cmd + GRAVITY_COMPENSATION_CURRENT;

    final_cmd = fmaxf(-M2006_CURRENT_LIMIT, fminf(M2006_CURRENT_LIMIT, final_cmd));
    arm->motor_cmd_current[index] = (int16_t)final_cmd;
    arm->motor_cmd_current[index + 2] = (int16_t)(-final_cmd);
}

void arm_set_close_angle(Arm_t *arm, int index, float target_angle)
{
    float final_cmd = arm_calculate_current_from_angle(arm, index, target_angle);
    final_cmd = fmaxf(-M2006_CURRENT_LIMIT, fminf(M2006_CURRENT_LIMIT, final_cmd));
    arm->motor_cmd_current[index] = (int16_t)final_cmd;
    arm->motor_cmd_current[index + 2] = (int16_t)(-final_cmd);
}

// 4. 设置伸缩电机的位置控制
static void arm_set_stretch_position_control(Arm_t *arm, float target_position)
{
    if (arm == NULL)
    {
        return;
    }
    float actual_position = arm->stretch; // 需要从电机数据计算得出
    // 位置PID计算速度设定值
    float target_speed = pid_calculate(&arm->stretch_pos_pid, target_position, actual_position, 0.0f, arm->dt);
    // 传递给速度环
    float target_current = arm_set_stretch_speed_control(arm, target_speed);
    target_current = fmaxf(-M3508_CURRENT_LIMIT, fminf(M3508_CURRENT_LIMIT, target_current));
    arm->motor_cmd_current[6] = (int16_t)target_current;
}

// 5. 设置伸缩电机的速度控制
static float arm_set_stretch_speed_control(Arm_t *arm, float target_speed)
{
    if (arm == NULL)
    {
        return 0.0f;
    }

    float actual_speed = arm->stretch_speed; // 或其他速度单位
    // 速度PID计算电流设定值
    float target_current = pid_calculate(&arm->stretch_speed_pid, target_speed, actual_speed, 0.0f, arm->dt);
    // 传递给电流环
    float motor_current_cmd = arm_set_stretch_current_control(arm, target_current);
    motor_current_cmd = fmaxf(-M3508_CURRENT_LIMIT, fminf(M3508_CURRENT_LIMIT, motor_current_cmd));
    arm->motor_cmd_current[6] = (int16_t)motor_current_cmd;
    return motor_current_cmd;
}

// 6. 设置伸缩电机的电流控制
static float arm_set_stretch_current_control(Arm_t *arm, float target_current)
{
    if (arm == NULL)
    {
        return 0.0f;
    }
    float actual_current = arm->stretch_motor.measure->current;

    float motor_cmd = pid_calculate(&arm->stretch_current_pid, target_current, actual_current, 0.0f, arm->dt);
    return motor_cmd;
}

// 7. 设置腕部电机的位置控制
static void arm_set_wrist_position_control(Arm_t *arm, int index, float target_angle)
{
    if (arm == NULL || index < 0 || index >= 2)
    {
        return;
    }
    // 假设 wrist_motor.measure 包含角度信息
    float actual_angle = arm->wrist_motor[index].measure->angle; // 需要确认数据结构
    // 位置PID计算电流设定值 (假设腕部只用位置和电流环)
    float target_current = pid_calculate(&arm->wrist_speed_pid[index], target_angle, actual_angle, 0.0f, arm->dt);
    // 传递给电流环
    float motor_current_cmd = arm_set_wrist_current_control(arm, index, target_current);
    motor_current_cmd = fmaxf(-M2006_CURRENT_LIMIT, fminf(M2006_CURRENT_LIMIT, motor_current_cmd));
    arm->motor_cmd_current[index + 4] = (int16_t)motor_current_cmd;
}

// 8. 设置腕部电机的电流控制
static float arm_set_wrist_current_control(Arm_t *arm, int index, float target_current)
{
    if (arm == NULL || index < 0 || index >= 2)
    {
        return 0.0f;
    }
    float actual_current = arm->wrist_motor[index].measure->current;
    // 电流PID计算最终指令
    float motor_cmd = pid_calculate(&arm->wrist_current_pid[index], target_current, actual_current, 0.0f, arm->dt);
    return motor_cmd;
}

int arm_init(Arm_t *arm)
{

    if (arm == NULL)
    {
        /* code */
        log_w("ptr arm created failed\n");
        return 1;
    }

    for (int i = 0; i < 4; ++i)
    {
        arm->close_motor[i].measure = get_motor_arm_measure_ptr(i);
    }
    for (int i = 0; i < 2; ++i)
    {
        arm->wrist_motor[i].measure = get_motor_wrist_measure_ptr(i);
    }
    // to be fixed
    arm->stretch_motor.measure = get_motor_stretch_measure_ptr();

    // PID init

    for (int i = 0; i < 2; ++i)
    {
        pid_init(&arm->close_angle_pid[i], PID_MODE_DERIVATIV_NONE, 0.001);
        pid_init(&arm->close_current_pid[i], PID_MODE_DERIVATIV_NONE, 0.001);
        pid_init(&arm->close_speed_pid[i], PID_MODE_DERIVATIV_NONE, 0.001);
        pid_init(&arm->wrist_current_pid[i], PID_MODE_DERIVATIV_NONE, 0.001);
        pid_init(&arm->wrist_speed_pid[i], PID_MODE_DERIVATIV_NONE, 0.001);
    }
    // set params
    pid_set_parameters(&arm->close_angle_pid[0], 0, 0, 0, 0, 0);
    pid_set_parameters(&arm->close_angle_pid[1], 0, 0, 0, 0, 0);
    pid_set_parameters(&arm->close_current_pid[0], 0.6, 1, 0, 3000, 8000);
    pid_set_parameters(&arm->close_current_pid[1], 0.6, 1, 0, 3000, 8000);
    pid_set_parameters(&arm->close_speed_pid[0], 9, 1, 0, 0, 0);
    pid_set_parameters(&arm->close_speed_pid[1], 0, 0, 0, 0, 0);
    pid_set_parameters(&arm->wrist_current_pid[0], 0, 0, 0, 0, 0);
    pid_set_parameters(&arm->wrist_current_pid[1], 0, 0, 0, 0, 0);
    pid_set_parameters(&arm->wrist_speed_pid[0], 0, 0, 0, 0, 0);
    pid_set_parameters(&arm->wrist_speed_pid[1], 0, 0, 0, 0, 0);

    arm->status = ARM_INIT;
    arm->calibrated = false;
    for (int i = 0; i < 2; i++)
    {
        arm->close_calibrated[i] = false;
        arm->close_angle_offset[i] = 0.0f;
        arm->avg_close_angle[i] = 0.0f;
        arm->avg_close_speed[i] = 0.0f;
        arm->avg_close_current[i] = 0.0f;
    }
    arm->stretch = 0.0f;
    arm->stretch_set = 0.0f;
    arm->last_update_t = 0;
    arm->dt = 0.001f; // 初始值

    return 0;
}
/*
    校准 ： 上电的时候 回到正确位置

*/
int arm_calibrate(Arm_t *arm)
{

    const float close_cali_speed = CALIBRATE_CLOSE_SPEED; // 校准想要的闭合速度，需更改
    float close_cmd[2];
    float close_current[2];
    for (int i = 0; i < 2; ++i)
    {
        if (arm->avg_close_current[i] > CALIBRATE_CURRENT &&
            arm->avg_close_speed[i] < CALIBRATE_CLOSE_SPEED * 0.5)
        {
            if (arm->close_calibrated[i] == false)
            {
                arm->close_calibrated[i] = true;
                pid_reset_integral(&arm->close_current_pid[i]);
                pid_reset_integral(&arm->close_speed_pid[i]);
                arm->close_angle_offset[i] = arm->avg_close_angle[i];
            }
            else
            {
                // arm->motor_cmd_current[i] = arm->motor_cmd_current[i + 2] = (int16_t)arm_set_close_speed_control(arm, i, 0);
            }
        }
        else
        {
            close_current[i] = pid_calculate(&arm->close_speed_pid[i], close_cali_speed, arm->avg_close_speed[i], 0, arm->dt);
            close_cmd[i] = pid_calculate(&arm->close_current_pid[i], close_current[i], arm->avg_close_current[i], 0, arm->dt);
            float final_current = fmaxf(-M2006_CURRENT_LIMIT, fminf(M2006_CURRENT_LIMIT, close_cmd[i]));

            arm->motor_cmd_current[i] = (int16_t)final_current;
            arm->motor_cmd_current[i + 2] = (int16_t)final_current;
        }
    }
    return 0;
}
/*
        根据M3508 和M2006测量数据 更新Arm的数据
        这里只是更新Arm的数据 比如说抓握的角度等等

*/
int arm_update_data(Arm_t *arm)
{

    hrt_abstime now = hrt_absolute_time();
    arm->dt = (float)(now - arm->last_update_t) / 1000000.0f;
    if (arm->last_update_t == 0 || arm->last_update_t >= now)
    {
        arm->dt = 0.001;
    }
    // 通过电机的测量值 更新arm的状态值
    // 抓取部分
    for (int i = 0; i < 2; ++i)
    {
        arm->avg_close_current[i] = 0.5f * (arm->close_motor[i].measure->current - arm->close_motor[i + 2].measure->current);
        arm->avg_close_speed[i] = 0.5f * (arm->close_motor[i].measure->rpm - arm->close_motor[i + 2].measure->rpm);
        // arm->avg_close_angle[i] = ENCODER_TO_ARM_CLOSE_ANGLE * 0.5f *
        //                               (arm->close_motor[i].measure->pos + arm->close_motor[i + 2].measure->pos) -
        //                           arm->close_angle_offset[i];
    }

    // arm->stretch_speed = RPM_TO_ARM_STRETCH_SPEED * arm->stretch_motor.measure->rpm;

    arm->last_update_t = now;
    return 0;
}

int arm_check(Arm_t *arm)
{
    arm_update_data(arm);
    if (arm->status == ARM_CALIBRATING)
        return 1; // 因为校准的时候会有碰撞
    // TODO 检测电流过高报警
    return 0;
}

static int arm_error_solve(Arm_t *arm)
{
    // TODO 如果报警 修正
}

// in app_arm.c
void arm_debug(Arm_t *arm)
{
    // --- 虚拟遥控器 ---
    int g_debug_motor_group = 1;   // 0 = 左爪
    int g_debug_mode = 0;          // 0 = 直接电流控制
    float g_debug_target = 700.0f; // 目标
    // ---------------------

    // float final_current_cmd = 0.0f;

    if (g_debug_motor_group == 0) // 调试左爪
    {
        if (g_debug_mode == 0)
        {
            arm_set_close_current(arm, 0, g_debug_target);
            myprintf("%f,%f\n", g_debug_target, (float)arm->avg_close_current[0]);
        }
        else if (g_debug_mode == 1)
        {
            // 【核心】进入速度环调试

            // 1. 调用速度环的 Caller 函数
            arm_set_close_speed(arm, 0, g_debug_target);

            // 2. 打印速度的目标值和实际值，用于Vofa+绘图
            myprintf("%f,%f\n", g_debug_target, (float)arm->avg_close_speed[0]);
        }
    }
    else if (g_debug_motor_group == 1) // 调试右爪
    {
        if (g_debug_mode == 0)
        {
            arm_set_close_current(arm, 1, g_debug_target);
            myprintf("%f,%f\n", g_debug_target, (float)arm->avg_close_current[1]);
        }
    }
}
int arm_control(Arm_t *arm)
{
    if (arm->status == ARM_DEBUG)
    {
        arm_debug(arm);
    }
    else
    {
        if (arm->status == ARM_CALIBRATING) // robot初始化完之后就会进入校准模式,然后对应各个部分的calibrating
        {
            arm_calibrate(arm);
            arm_set_wrist_position_control(arm, 0, 0);
            arm_set_wrist_position_control(arm, 1, 0);
            arm_set_stretch_speed_control(arm, 0);
        }
        else if (arm->status == ARM_LOCK)
        {
            // arm_set_close_speed_control(arm, 0, 0);
            // arm_set_close_speed_control(arm, 1, 0);
            arm_set_wrist_position_control(arm, 0, 0);
            arm_set_wrist_position_control(arm, 1, 0);
            arm_set_stretch_speed_control(arm, 0);
        }
        else if (arm->status == ARM_CONTROL)
        {
            if (arm->mode & CLOSE_ANGLE)
            {
                // arm_set_close_angle_control(arm, 0, arm->angle_closed_set);
                // arm_set_close_angle_control(arm, 1, arm->angle_closed_set);
            }
            else
            {
                // arm_set_close_speed_control(arm, 0, 0);
                // arm_set_close_speed_control(arm, 1, 0);
            }
            if (arm->mode & STRETCH_POS)
            {
                arm_set_stretch_position_control(arm, arm->stretch_set);
            }
            else if (arm->mode & STRETCH_SPEED)
            {
                arm_set_stretch_speed_control(arm, arm->stretch_speed_set);
            }
            else
            {
                arm_set_stretch_speed_control(arm, 0);
            }
        }
    }

    CAN_send_motor_currents(2, 0x200,
                            arm->motor_cmd_current[0],
                            arm->motor_cmd_current[1],
                            arm->motor_cmd_current[2],
                            arm->motor_cmd_current[3]);

    CAN_send_motor_currents(2, 0x1FF,
                            arm->motor_cmd_current[4], // wrist 0
                            arm->motor_cmd_current[5], // wrist 1
                            arm->motor_cmd_current[6], // stretch
                            0);                        // ID 8未使用
    return 0;
}