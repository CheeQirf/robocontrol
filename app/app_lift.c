#include "app_lift.h"

#include "app_can.h"
#include "math.h"
#include "usart.h"
Lift_t Lift;

#define PI 3.14159265358979323846
#define M3508_CURRENT_LIMIT 10000

#define TICKS_TO_LIFT_HEIGHT (4096.0f / (37.05f * PI)) // 单位mm

#define ENCODER_TOTAL_TICKS (16.0f * 4096.0f)

static float lift_calculate_current_from_current(Lift_t *lift, int index, float target_current)
{
    float final_cmd = pid_calculate(&lift->current_pid[index], target_current, lift->avg_current[index], 0, lift->dt);
    return final_cmd;
}

static float lift_calculate_current_from_speed(Lift_t *lift, int index, float target_speed)
{
    float target_current = pid_calculate(&lift->speed_pid[index], target_speed, lift->avg_speed[index], 0, lift->dt);
    return lift_calculate_current_from_current(lift, index, target_current);
}

// 外环：位置控制
static void lift_set_position_control(Lift_t *lift, float target_height)
{
    // 三环控制
    // float target_rpm = pid_calculate(&lift->pos_pid, target_height, lift->height, 0, lift->dt);

    // for (int i = 0; i < 2; ++i)
    // {
    //     float final_cmd = lift_calculate_current_from_speed(lift, i, target_rpm);
    //     final_cmd = fmaxf(-M3508_CURRENT_LIMIT, fminf(M3508_CURRENT_LIMIT, final_cmd));

    //     lift->motor_cmd_current[i] = (int16_t)final_cmd;
    //     lift->motor_cmd_current[i + 2] = (int16_t)final_cmd;
    // }
    // 两环控制
    float target_current = pid_calculate(&lift->pos_pid, target_height, lift->height, 0, lift->dt);

    for (int i = 0; i < 2; ++i)
    {
        float final_cmd = lift_calculate_current_from_current(lift, i, target_current);
        final_cmd = fmaxf(-M3508_CURRENT_LIMIT, fminf(M3508_CURRENT_LIMIT, final_cmd));

        lift->motor_cmd_current[i] = (int16_t)final_cmd;
        lift->motor_cmd_current[i + 2] = (int16_t)final_cmd;
    }
}

// <-- 新增：一个独立的速度控制函数，用于校准和调试
static void lift_set_speed_control(Lift_t *lift, int index, float target_speed)
{
    float final_cmd = lift_calculate_current_from_speed(lift, index, target_speed);
    final_cmd = fmaxf(-M3508_CURRENT_LIMIT, fminf(M3508_CURRENT_LIMIT, final_cmd));
    lift->motor_cmd_current[index] = (int16_t)final_cmd;
    lift->motor_cmd_current[index + 2] = (int16_t)final_cmd;
}
int lift_init(Lift_t *lift)
{

    for (int i = 0; i < 4; ++i)
    {
        lift->motors[i].measure = get_motor_lift_measure_ptr(i);
    }
    for (int i = 0; i < 2; ++i)
    {
        lift->encoders[i] = get_encoder_measure_ptr(i);
    }
    // 初始化PID控制器
    pid_init(&lift->pos_pid, PID_MODE_DERIVATIV_NONE, 0.001);
    pid_init(&lift->speed_pid[0], PID_MODE_DERIVATIV_NONE, 0.001);
    pid_init(&lift->speed_pid[1], PID_MODE_DERIVATIV_NONE, 0.001);
    pid_init(&lift->current_pid[0], PID_MODE_DERIVATIV_NONE, 0.001);
    pid_init(&lift->current_pid[1], PID_MODE_DERIVATIV_NONE, 0.001);

    // 设置PID参数

    pid_set_parameters(&lift->pos_pid, 3, 0.2, 0, 0, 0);
    pid_set_parameters(&lift->speed_pid[0], 4, 0.5, 0, 0, 0);
    pid_set_parameters(&lift->speed_pid[1], 4, 0.5, 0, 0, 0);
    pid_set_parameters(&lift->current_pid[0], 3, 1, 0, 0, 0);
    pid_set_parameters(&lift->current_pid[1], 3, 1, 0, 0, 0);

    lift->calibrated = false;
    lift->height = 0.0f;
    lift->height_set = 0.0f;
    lift->height_offset = 0.0f;

    for (int i = 0; i < 2; ++i)
    {
        lift->avg_speed[i] = 0.0f;
        lift->avg_current[i] = 0.0f;
        lift->avg_pos[i] = 0.0f;
    }
    for (int i = 0; i < 4; ++i)
    {
        lift->motor_cmd_current[i] = 0;
    }

    lift->last_update_t = 0;
    lift->dt = 0.001f;

    lift->status = LIFT_INIT;
    return 0;
}

int lift_calibrate(Lift_t *lift)
{
    // 定义校准时使用的常量
    const float cali_speed = -500.0f;             // 向下运动的速度 (rpm)
    const float cali_current_threshold = 4000.0f; // 堵转电流阈值 (mA)
    const float cali_speed_threshold = 50.0f;     // 堵转速度阈值 (rpm)

    // 如果已经校准完成，则应该保持不动并退出
    if (lift->calibrated)
    {
        for (int i = 0; i < 4; ++i)
            lift->motor_cmd_current[i] = 0;
        return 0;
    }

    // 分别控制两侧以校准速度向下运动
    for (int i = 0; i < 2; ++i)
    {
        lift_set_speed_control(lift, i, cali_speed);
    }

    // 当两侧都满足堵转条件时，认为到达底部
    bool left_stalled = (fabs(lift->avg_current[0]) > cali_current_threshold) && (fabs(lift->avg_speed[0]) < cali_speed_threshold);
    bool right_stalled = (fabs(lift->avg_current[1]) > cali_current_threshold) && (fabs(lift->avg_speed[1]) < cali_speed_threshold);

    if (left_stalled && right_stalled)
    {
        // 关键一步：读取当前时刻的绝对编码器平均ticks，作为零点偏移
        int32_t left_ticks = (int32_t)lift->encoders[0]->encoder_count;
        int32_t right_ticks = (int32_t)lift->encoders[1]->encoder_count;

        lift->height_offset = 0.5f * (float)(left_ticks + right_ticks);

        lift->calibrated = true;

        // 复位PID积分项，防止积分饱和影响后续控制
        pid_reset_integral(&lift->pos_pid);
        for (int i = 0; i < 2; ++i)
        {
            pid_reset_integral(&lift->speed_pid[i]);
            pid_reset_integral(&lift->current_pid[i]);
        }

        lift->status = LIFT_LOCK; // 校准完成，切换到锁定状态
        myprintf("Lift calibration complete! Offset is: %f ticks\n", lift->height_offset);
    }

    return 0;
}

int lift_check(Lift_t *lift)
{
    lift_update_data(lift);
    return 0;
}

int lift_update_data(Lift_t *lift)
{
    // TODO 2 用编码器数据更新 lift位置
    hrt_abstime now = hrt_absolute_time();
    lift->dt = (float)(now - lift->last_update_t) / 1000000.0f;
    if (lift->last_update_t == 0 || lift->last_update_t >= now || lift->dt <= 0)
    {
        lift->dt = 0.001f; // 避免dt为0或负数
    }
    int32_t left_total_ticks = (int32_t)lift->encoders[0]->encoder_count;
    int32_t right_total_ticks = (int32_t)lift->encoders[1]->encoder_count;

    // float avg_total_ticks = 0.5f * (float)(left_total_ticks + right_total_ticks);
    float avg_total_ticks = right_total_ticks;

    // lift->height = (avg_total_ticks - lift->height_offset) * TICKS_TO_LIFT_HEIGHT;
    lift->height = avg_total_ticks * TICKS_TO_LIFT_HEIGHT;

    for (int i = 0; i < 2; ++i) // 对应左右的抬升电机
    {
        lift->avg_current[i] = 0.5f * (lift->motors[i].measure->current + lift->motors[i + 2].measure->current);
        lift->avg_speed[i] = 0.5f * (lift->motors[i].measure->rpm + lift->motors[i + 2].measure->rpm);
    }

    lift->last_update_t = now;
    return 0;
}

int lift_control(Lift_t *lift)
{
    if (lift->status == LIFT_DEBUG)
    {
        lift_debug(lift);
    }
    else if (lift->status == LIFT_CALIBRATING)
    {
        lift_calibrate(lift); // 假设 lift_calibrate 会设置 motor_cmd_current
    }
    else if (lift->status == LIFT_CONTROL)
    {
        lift_set_position_control(lift, lift->height_set);
    }
    else // 包括 LIFT_INIT, LIFT_LOCK, LIFT_OK 等所有其他状态
    {
        // 明确停止所有电机
        for (int i = 0; i < 4; ++i)
        {
            lift->motor_cmd_current[i] = 0;
        }
    }

    CAN_send_motor_currents(1, 0x200,
                            lift->motor_cmd_current[0],
                            lift->motor_cmd_current[1],
                            lift->motor_cmd_current[2],
                            lift->motor_cmd_current[3]);

    return 0;
}

void lift_debug(Lift_t *lift)
{
    // --- 虚拟遥控器 (设置为 static, 以便在调试器中修改后能保持值) ---
    // 您可以在调试模式下，通过IDE修改这些变量的值，实时改变调试目标，无需重新编译
    static int g_debug_motor_group = 1;  // 调试对象: 0 = 左侧电机组(1&3), 1 = 右侧电机组(2&4)
    static int g_debug_mode = 2;         // 调试模式: 0 = 电流环, 1 = 速度环, 2 = 位置环
    static float g_debug_target = 40.0f; // 调试目标值 (单位: mA, rpm, 或 mm)
    // ----------------------------------------------------------------

    if (g_debug_mode == 0)
    {
        // 目标：验证 avg_current 能否精确跟随 target_current
        // 调用最内层的PID计算
        float final_cmd = lift_calculate_current_from_current(lift, g_debug_motor_group, g_debug_target);
        final_cmd = fmaxf(-M3508_CURRENT_LIMIT, fminf(M3508_CURRENT_LIMIT, final_cmd));

        // 将指令赋给指定侧的两个电机
        lift->motor_cmd_current[g_debug_motor_group] = (int16_t)final_cmd;
        lift->motor_cmd_current[g_debug_motor_group + 2] = (int16_t)final_cmd;

        myprintf("%f,%f\n", g_debug_target, lift->avg_current[g_debug_motor_group]);
    }
    else if (g_debug_mode == 1)
    {
        lift_set_speed_control(lift, g_debug_motor_group, g_debug_target);

        myprintf("%f,%f\n", g_debug_target, lift->avg_speed[g_debug_motor_group]);
    }
    else if (g_debug_mode == 2)
    {
        lift->height_set = g_debug_target;

        // 调用最外层的位置控制函数
        lift_set_position_control(lift, lift->height_set);

        myprintf("%f,%f\n", g_debug_target, lift->height);
    }
}