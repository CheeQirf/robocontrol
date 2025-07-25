#include "pid.h"
#include <math.h> // for fabsf, fmaxf, isnan, isinf

// 定义一个很小的浮点数，用于比较接近零的值
#define SIGMA 0.000001f

// 初始化 PID 控制器
void pid_init(PID_t *pid, pid_mode_t mode, float dt_min)
{
    pid->mode = mode;
    pid->dt_min = dt_min;
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->integral = 0.0f;
    pid->integral_limit = 0.0f;
    pid->output_limit = 0.0f;
    pid->error_previous = 0.0f;
    pid->last_output = 0.0f;
}

// 设置 PID 控制器参数
int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    int ret = 0; // 0 表示成功，1 表示有无效参数

    if (CUSTOM_ISFINITE(kp))
    {
        pid->kp = kp;
    }
    else
    {
        ret = 1;
    }

    if (CUSTOM_ISFINITE(ki))
    {
        pid->ki = ki;
    }
    else
    {
        ret = 1;
    }

    if (CUSTOM_ISFINITE(kd))
    {
        pid->kd = kd;
    }
    else
    {
        ret = 1;
    }

    if (CUSTOM_ISFINITE(integral_limit))
    {
        pid->integral_limit = fabsf(integral_limit); // 确保积分限制为正
    }
    else
    {
        ret = 1;
    }

    if (CUSTOM_ISFINITE(output_limit))
    {
        pid->output_limit = fabsf(output_limit); // 确保输出限制为正
    }
    else
    {
        ret = 1;
    }

    return ret;
}

// 计算 PID 控制器输出
float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
    // 检查输入参数是否有效，如果无效则返回上一次的输出
    if (!CUSTOM_ISFINITE(sp) || !CUSTOM_ISFINITE(val) || !CUSTOM_ISFINITE(val_dot) || !CUSTOM_ISFINITE(dt))
    {
        return pid->last_output;
    }

    float i_term, d_term; // 积分项和微分项

    /* 当前误差值 */
    float error = sp - val;

    /* 当前误差的微分项 */
    if (pid->mode == PID_MODE_DERIVATIV_CALC)
    {
        // 导数通过当前误差与上一误差的差值计算
        d_term = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
        pid->error_previous = error; // 更新上一误差
    }
    else if (pid->mode == PID_MODE_DERIVATIV_CALC_NO_SP)
    {
        // 导数通过当前值与上一值的差值计算（例如，速度控制中直接对速度进行微分）
        d_term = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
        pid->error_previous = -val; // 更新上一值
    }
    else if (pid->mode == PID_MODE_DERIVATIV_SET)
    {
        // 导数直接由外部提供
        d_term = -val_dot; // 注意这里的负号，通常表示“抵抗”变化
    }
    else
    {
        // 未知模式，微分项为 0
        d_term = 0.0f;
    }

    // 检查微分项是否有效
    if (!CUSTOM_ISFINITE(d_term))
    {
        d_term = 0.0f;
    }

    /* 计算 P 和 D 部分的输出 */
    float output = (error * pid->kp) + (d_term * pid->kd);

    /* 积分项计算 */
    if (pid->ki > SIGMA)
    { // 只有当积分增益不为零时才计算积分
        // 计算新的积分项累积值
        i_term = pid->integral + (error * dt);

        /* 检查积分饱和 */
        if (CUSTOM_ISFINITE(i_term))
        {
            // 条件：
            // 1. 如果没有设置输出限制 (pid->output_limit < SIGMA)
            // 2. 或者 (P + D + 新的积分项) 不会超过输出限制
            // 3. 并且新的积分项本身不超过积分限制
            if ((pid->output_limit < SIGMA || (fabsf(output + (i_term * pid->ki)) <= pid->output_limit)) &&
                fabsf(i_term) <= pid->integral_limit)
            {
                /* 未饱和，使用新的积分值 */
                pid->integral = i_term;
            }
        }

        /* 将积分项添加到总输出 */
        output += pid->integral * pid->ki;
    }

    /* 限制总输出 */
    if (CUSTOM_ISFINITE(output))
    {
        if (pid->output_limit > SIGMA)
        { // 如果设置了输出限制
            if (output > pid->output_limit)
            {
                output = pid->output_limit; // 超出上限，钳位到上限
            }
            else if (output < -pid->output_limit)
            {
                output = -pid->output_limit; // 超出下限，钳位到下限
            }
        }
        pid->last_output = output; // 保存当前输出，以备下次使用或作为错误返回值
    }

    return pid->last_output;
}

// 重置 PID 积分项
void pid_reset_integral(PID_t *pid)
{
    pid->integral = 0.0f;
}