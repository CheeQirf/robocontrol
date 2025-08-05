#include "app_mavlink.h"
#include "usart.h"
#include "mavlink_types.h"
#include "ardupilotmega/mavlink.h"

#define MAVLINK_SERIAL huart5
#define MAVLINK_RECV_LEN 280
static uint8_t mavlink_recv_buff[MAVLINK_RECV_LEN];

// extern Robot_t Robot;

void mavlink_dispatch(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
        //	case MAVLINK_MSG_ID_GIMBAL_TRACK :
        //		mavlink_msg_gimbal_track_decode(msg,&gimbal_command);
        break;

    default:
        break;
    }
}

/**
 * @brief Mavlink的接收回调函数，在接收到一帧数据后被HAL库调用
 */
void mavlink_callback(UART_HandleTypeDef *huart, uint16_t size)
{
    static mavlink_message_t msg;
    static mavlink_status_t status;

    // 1. 解析刚刚收到的数据 (长度为 size)
    for (int i = 0; i < size; ++i)
    {
        if (MAVLINK_FRAMING_OK == mavlink_parse_char(MAVLINK_COMM_0, mavlink_recv_buff[i], &msg, &status))
        {
            mavlink_dispatch(&msg);
        }
    }

    // 2. 【关键】处理完本次数据后，立即重新启动下一次中断接收，准备接收下一个数据包
    HAL_UARTEx_ReceiveToIdle_IT(&MAVLINK_SERIAL, (uint8_t *)mavlink_recv_buff, MAVLINK_RECV_LEN);
}

/**
 * @brief 初始化函数，用于注册回调并启动第一次接收
 */
void mavlink_uart_init(void) // 改个更合适的名字
{
    // 注册回调函数
    HAL_UART_RegisterRxEventCallback(&MAVLINK_SERIAL, mavlink_callback);

    // 启动第一次中断接收
    HAL_UARTEx_ReceiveToIdle_IT(&MAVLINK_SERIAL, (uint8_t *)mavlink_recv_buff, MAVLINK_RECV_LEN);
}