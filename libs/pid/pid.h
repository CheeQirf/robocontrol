#ifndef PID_H
#define PID_H

#include <stdbool.h> // 用于 bool 类型
#include <stdint.h>  // 用于 int 类型，虽然这里主要用 float

// 为了兼容性，定义一个简单的浮点数有限性检查宏
// 在大多数C标准库中，可以使用 isnan() 和 isinf() 来实现
#ifdef __cplusplus
#include <cmath>
#define CUSTOM_ISFINITE(x) (std::isfinite(x))
#else
#include <math.h> // for fabsf, isnan, isinf
#define CUSTOM_ISFINITE(x) (!isnan(x) && !isinf(x))
#endif

/**
 * @brief PID 控制器的工作模式。
 */
typedef enum
{
    PID_MODE_DERIVATIV_CALC,       ///< 导数通过 (error - error_previous) / dt 计算
    PID_MODE_DERIVATIV_CALC_NO_SP, ///< 导数通过 (-val - error_previous) / dt 计算 (例如，仅对输入值的变化率进行微分)
    PID_MODE_DERIVATIV_SET,        ///< 导数直接由 val_dot 参数提供
    PID_MODE_LAST_VALUE            ///< 未知模式，导数设为 0
} pid_mode_t;

/**
 * @brief PID 控制器结构体。
 */
typedef struct
{
    pid_mode_t mode;      ///< PID 控制器的工作模式
    float dt_min;         ///< 最小时间步长，用于避免除以零或过小的值
    float kp;             ///< 比例增益
    float ki;             ///< 积分增益
    float kd;             ///< 微分增益
    float integral;       ///< 积分项累积值
    float integral_limit; ///< 积分项的绝对值限制 (防止积分饱和)
    float output_limit;   ///< 输出的绝对值限制
    float error_previous; ///< 上一次的误差值，用于计算微分项
    float last_output;    ///< 上一次的输出值
} PID_t;

/**
 * @brief 初始化 PID 控制器结构体。
 *
 * @param pid 指向 PID_t 结构体的指针。
 * @param mode PID 控制器的工作模式。
 * @param dt_min 最小时间步长，用于计算微分项时防止除以零或过小的值。
 */
void pid_init(PID_t *pid, pid_mode_t mode, float dt_min);

/**
 * @brief 设置 PID 控制器的参数。
 *
 * @param pid 指向 PID_t 结构体的指针。
 * @param kp 比例增益。
 * @param ki 积分增益。
 * @param kd 微分增益。
 * @param integral_limit 积分项的绝对值限制。
 * @param output_limit 输出的绝对值限制。
 * @return int 0表示成功，1表示有无效参数。
 */
int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);

/**
 * @brief 计算 PID 控制器的输出。
 *
 * @param pid 指向 PID_t 结构体的指针。
 * @param sp 设定点 (Set Point)。
 * @param val 当前测量值 (Process Variable)。
 * @param val_dot 当前测量值的变化率（仅在 PID_MODE_DERIVATIV_SET 模式下使用）。
 * @param dt 自上次计算以来的时间步长。
 * @return float PID 控制器的输出值。
 */
float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt);

/**
 * @brief 重置 PID 控制器的积分项。
 *
 * @param pid 指向 PID_t 结构体的指针。
 */
void pid_reset_integral(PID_t *pid);

#endif // PID_H