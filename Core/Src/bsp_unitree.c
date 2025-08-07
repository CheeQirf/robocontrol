#include "bsp_unitree.h"
#include "motor_control.h" // 引入官方底层头文件
#include "hrtimer.h"
#include "usart.h"
static UnitreeMotor_Measure_t joint_motor[2];
static volatile UartCommState_e g_uart_comm_state = UART_STATE_IDLE;

static MOTOR_recv g_motor_recv_buffer;

static UnitreeMotor_t *g_current_motor = NULL;
void unitree_motor_init(UnitreeMotor_t *motor, uint8_t id, const UnitreeMotor_Measure_t *measure_p)
{
    motor->id = id;
    motor->measure = measure_p; // 关联指针

    // 初始化控制指令
    motor->mode_set = 0; // 默认失能
    motor->pos_set = 0.0f;
    motor->speed_set = 0.0f;
    motor->kp_set = 0.0f;
    motor->kw_set = 0.0f;
    motor->torque_set = 0.0f;
}

int unitree_motor_control_send(UnitreeMotor_t *motor)
{
    if (!motor || !motor->measure)
    {
        return -1;
    }

    // 1. 准备发送和接收的底层结构体
    MOTOR_send cmd_to_send;
    MOTOR_recv data_received;

    // 2. 将上层应用的控制指令(rad, N.m)转换为底层结构体需要的格式
    //    这一步由官方底层函数 modify_data() 完成
    cmd_to_send.id = motor->id;
    cmd_to_send.mode = motor->mode_set;
    cmd_to_send.Pos = motor->pos_set;
    cmd_to_send.W = motor->speed_set;
    cmd_to_send.K_P = motor->kp_set;
    cmd_to_send.K_W = motor->kw_set;
    cmd_to_send.T = motor->torque_set;

    // 3. 调用官方底层函数，执行通信。这是个阻塞操作。
    HAL_StatusTypeDef ret = SERVO_Send_recv(&cmd_to_send, &data_received);

    // 4. 将收到的数据更新到 motor->measure 指向的全局数据区
    //    需要一个可写的指针来操作 const 指针指向的内容
    UnitreeMotor_Measure_t *writable_measure = (UnitreeMotor_Measure_t *)motor->measure;

    if (ret == HAL_OK && data_received.correct)
    {
        // 通信成功，更新数据快照
        writable_measure->theta_rad = data_received.Pos;
        writable_measure->omega_rad_s = data_received.W;
        writable_measure->tau_N_m = data_received.T;
        writable_measure->temp = data_received.Temp;
        writable_measure->merror = data_received.MError;
        writable_measure->data_is_valid = true;
        writable_measure->last_update_time = hrt_absolute_time();
    }
    else
    {
        // 通信失败，将数据标记为无效
        writable_measure->data_is_valid = false;
    }

    return ret;
}
bool unitree_motor_control_start(UnitreeMotor_t *motor)
{
    // 如果串口不处于空闲状态，则返回false，表示正忙
    if (g_uart_comm_state != UART_STATE_IDLE)
    {
        return false;
    }

    // 记录当前是哪个电机在通信
    g_current_motor = motor;

    // 1. 进入发送状态
    g_uart_comm_state = UART_STATE_TX;

    // 2. 准备发送数据 (这一步不变)
    static MOTOR_send cmd_to_send; // 使用静态变量，防止栈溢出
    cmd_to_send.id = motor->id;
    cmd_to_send.mode = motor->mode_set;
    cmd_to_send.Pos = motor->pos_set;
    cmd_to_send.W = motor->speed_set;
    cmd_to_send.K_P = motor->kp_set;
    cmd_to_send.K_W = motor->kw_set;
    cmd_to_send.T = motor->torque_set;
    // ... 其他参数赋值 ...
    modify_data(&cmd_to_send); // 计算CRC等

    // 3. 设置RS485为发送模式
    SET_485_UP();

    // 4. 【核心】启动中断发送，这个函数会立即返回
    HAL_UART_Transmit_IT(&huart1, (uint8_t *)&cmd_to_send.motor_send_data, sizeof(cmd_to_send.motor_send_data));

    return true; // 成功启动
}

/**
 * @brief 【中断回调】发送完成时被HAL库自动调用
 * @param huart 触发中断的串口句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 确保是我们的目标串口 (huart1)
    if (huart->Instance == USART1)
    {
        // 确保我们当前处于发送状态
        if (g_uart_comm_state == UART_STATE_TX)
        {
            // 1. 设置RS485为接收模式
            SET_485_DOWN();

            // 2. 进入接收状态
            g_uart_comm_state = UART_STATE_RX;

            // 3. 【核心】启动中断接收(使用空闲中断模式)
            HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)&g_motor_recv_buffer.motor_recv_data, sizeof(g_motor_recv_buffer.motor_recv_data));
        }
    }
}

/**
 * @brief 【中断回调】接收到数据(并检测到总线空闲)时被HAL库自动调用
 * @param huart 触发中断的串口句柄
 * @param Size 接收到的数据长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 确保是我们的目标串口
    if (huart->Instance == USART1)
    {
        // 确保我们当前处于接收状态
        if (g_uart_comm_state == UART_STATE_RX)
        {
            // 1. 检查接收长度和包头是否正确 (可选但推荐)
            if (Size == sizeof(g_motor_recv_buffer.motor_recv_data) &&
                g_motor_recv_buffer.motor_recv_data.head[0] == 0xFE &&
                g_motor_recv_buffer.motor_recv_data.head[1] == 0xEE)
            {
                // 2. 提取数据并做CRC校验 (extract_data内部会设置 .correct 标志)
                extract_data(&g_motor_recv_buffer);

                // 3. 如果数据有效，更新到对应的全局测量区
                if (g_motor_recv_buffer.correct && g_current_motor != NULL)
                {
                    UnitreeMotor_Measure_t *writable_measure = (UnitreeMotor_Measure_t *)g_current_motor->measure;
                    writable_measure->theta_rad = g_motor_recv_buffer.Pos;
                    writable_measure->omega_rad_s = g_motor_recv_buffer.W;
                    // ... 更新其他测量值 ...
                    writable_measure->data_is_valid = true;
                    writable_measure->last_update_time = hrt_absolute_time();
                }
            }

            // 4. 无论接收是否成功，通信流程都已结束，回到空闲状态
            g_uart_comm_state = UART_STATE_IDLE;
        }
    }
}

inline UnitreeMotor_Measure_t *get_unitree_motor_measure_ptr(uint8_t index)
{
    return &joint_motor[index];
}