#include "app_can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#include "elog.h"
#include "queue.h"

static xQueueHandle xCANSendQueue = NULL; //
static xQueueHandle xCANRcvQueue = NULL;  //


M2006_Motor_t g_m2006_motors[MAX_M2006_MOTORS];
M3508_Motor_t g_m3508_motors[MAX_M3508_MOTORS];
Go_M8010_6_Motor_t g_gom8010_6_motors[MAX_GO8010_6_MOTORS];

static void CAN_Rx_Task(void *pvParameters)
{
    static CanMessage_t RxMsg; // 接受用的变量
    // static Message msg;  // 这个是CanOpen的message 我们要把它注释掉
    uint8_t i = 0;
    for (;;)
    {
        if (xQueueReceive(xCANRcvQueue, &RxMsg, 100) == pdTRUE)
        { // 接收队列中的消息
          // canDispatch(&msg); // 我们重写这个函数
            /*  canDispatch 函数： 传参（can msg的指针）
                 实现can 的分发 与 处理
            */
        }
    }
}
void CAN_Rcv_DateFromISR(CanMessage_t *RxMsg)
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if (NULL != xCANRcvQueue)
    {
        if (xQueueSendToBackFromISR(xCANRcvQueue, RxMsg, &xHigherPriorityTaskWoken) != pdPASS)
        {
            log_w("Insert Date to Rx QuxQueue is not OK\n");
        }
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}
void canDispatch(CanMessage_t *msg)
{
    if (msg == NULL)
    {
        log_e("canDispatch: Received NULL message pointer");
        return;
    }
    uint32_t can_id = msg->id;
    uint8_t *data = msg->data;
    uint8_t can_port = msg->can_num;
    if (can_port == 1) // 处理M2006电机
    {
        if (can_id >= 0x201 && can_id <= 0x206)
        {
            uint8_t motor_idx = can_id - 0x201;
            if (motor_idx < MAX_M2006_MOTORS)
            {
                M2006_Motor_t *motor = &g_m2006_motors[motor_idx];
                motor->rotor_mech_angle = (data[0] << 8) | data[1];
                motor->rotor_speed = (data[2] << 8) | data[3];
                motor->actual_torque_current = (data[4] << 8) | data[5];
            }
        }
    }
    else if (can_port == 2) // 处理M3508电机和宇树电机
    {
        if (can_id >= 0x201 && can_id <= 0x205)
        {
            uint8_t motor_idx = can_id - 0x201;
            if (motor_idx < MAX_M3508_MOTORS)
            {
                M3508_Motor_t *motor = &g_m3508_motors[motor_idx];
                motor->rotor_mech_angle = (data[0] << 8) | data[1];
                motor->rotor_speed = (data[2] << 8) | data[3];
                motor->actual_torque_current = (data[4] << 8) | data[5];
                motor->motor_temperature = data[6];
            }
        }
        else if (can_id > 0x999) // 示例
        {
        }
    }
    else
    {
        log_e("canDispatch: Invalid CAN port number %d for ID 0x%X", can_port, can_id);
    }
}