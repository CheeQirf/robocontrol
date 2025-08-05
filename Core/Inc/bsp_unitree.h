#ifndef __BSP_UNITREE_H
#define __BSP_UNITREE_H

#include "app_motor.h"
typedef enum
{
    UART_STATE_IDLE,
    UART_STATE_TX,
    UART_STATE_RX,
} UartCommState_e;
/**
 * @brief 初始化宇树电机控制对象，并将其与一个外部的测量数据区关联
 * @param motor 指向要初始化的电机控制对象
 * @param id 电机的物理ID
 * @param measure_p 指向为该电机分配的测量数据存储区
 */
void unitree_motor_init(UnitreeMotor_t *motor, uint8_t id, const UnitreeMotor_Measure_t *measure_p);

/**
 * @brief 执行一次“发送指令-接收反馈”的完整通信周期
 * @brief 这个函数会从 motor 对象中读取控制指令，调用底层函数发送，
 * @brief 然后将接收到的反馈数据更新到 motor->measure 指向的内存区域。
 * @param motor 指向要通信的电机对象
 * @return HAL_StatusTypeDef 通信结果
 */
int unitree_motor_control_send(UnitreeMotor_t *motor);
bool unitree_motor_control_start(UnitreeMotor_t *motor);
UnitreeMotor_Measure_t *get_unitree_motor_measure_ptr(uint8_t index);
#endif