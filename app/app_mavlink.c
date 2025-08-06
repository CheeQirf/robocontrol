#include "app_mavlink.h"
#include "usart.h"
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "app_robot.h"
#include "elog.h"
#include "param.h"

#define MAVLINK_SERIAL huart5
#define MAVLINK_RECV_LEN 280
static uint8_t mavlink_recv_buff[MAVLINK_RECV_LEN];

extern Robot_t Robot;

#define RC_CHANNEL_HIGH_THRESHOLD 1800
#define RC_CHANNEL_LOW_THRESHOLD 1200

static mavlink_rc_channels_t rc_channels;
void mavlink_dispatch(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_RC_CHANNELS:

        // 解码消息
        mavlink_msg_rc_channels_decode(msg, &rc_channels);

        if (rc_channels.chan10_raw > RC_CHANNEL_HIGH_THRESHOLD)
        {
            Robot.arm->angle_closed_set = CLAW_ANGLE_OPEN;
        }
        else
        {
            Robot.arm->angle_closed_set = CLAW_ANGLE_CLOSE;
        }

        // --- 升降控制 (通道8) ---
        if (rc_channels.chan8_raw > RC_CHANNEL_HIGH_THRESHOLD)
        {
            Robot.lift->height_set = LIFT_HEIGHT_UP;
        }
        else
        {
            Robot.lift->height_set = LIFT_HEIGHT_DOWN;
        }

        // --- 机器人状态切换 (通道9) ---
        if (rc_channels.chan9_raw > RC_CHANNEL_HIGH_THRESHOLD)
        {
            // PWM 值最高，对应 DEBUG 模式
            Robot.status = ROBOT_DEBUG;
        }
        else if (rc_channels.chan9_raw < RC_CHANNEL_LOW_THRESHOLD)
        {
            // PWM 值最低，对应 LOCK 模式 这是在最上面
            Robot.status = ROBOT_LOCK;
        }
        else
        {
            // PWM 值在中间，对应 CONTROL 模式
            Robot.status = ROBOT_CONTROL;
        }

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
    HAL_UARTEx_ReceiveToIdle_DMA(&MAVLINK_SERIAL, (uint8_t *)mavlink_recv_buff, MAVLINK_RECV_LEN);
}

/**
 * @brief 初始化函数，用于注册回调并启动第一次接收
 */
void mavlink_uart_init(void) // 改个更合适的名字
{
    // 注册回调函数
    HAL_UART_RegisterRxEventCallback(&MAVLINK_SERIAL, mavlink_callback);

    // 启动第一次中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(&MAVLINK_SERIAL, (uint8_t *)mavlink_recv_buff, MAVLINK_RECV_LEN);
}
