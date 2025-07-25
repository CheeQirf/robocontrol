#include "app_can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#define LOG_TAG "APP_CAN"
#include "elog.h"
#include "queue.h"
#include "bsp_can.h"
#include "app_motor.h"

static xQueueHandle xCANSendQueue = NULL; //
static xQueueHandle xCANRcvQueue = NULL;  //

static motorMeasure_t motor_arm[4];
static motorMeasure_t motor_wrist[2];
static motorMeasure_t motor_lift[4];
static motorMeasure_t motor_stretch;

void canDispatch(CanMessage_t *msg);

#define get_dji_motor_measure(ptr, data)                         \
    {                                                            \
        (ptr)->last_angle = (ptr)->angle;                        \
        (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]);   \
        (ptr)->rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperture = (data)[6];                           \
    }

static void CAN_Rx_Task(void *pvParameters)
{
    static CanMessage_t RxMsg; // 接受用的变量
    // static Message msg;  // 这个是CanOpen的message 我们要把它注释掉
    // uint8_t i = 0;
    for (;;)
    {
        if (xQueueReceive(xCANRcvQueue, &RxMsg, 100) == pdTRUE)
        { // 接收队列中的消息

            taskENTER_CRITICAL(); // 临界区保护
            canDispatch(&RxMsg);  // 我们重写这个函数
            taskEXIT_CRITICAL();

            /*  canDispatch 函数： 传参（can msg的指针）
                 实现can 的分发 与 处理
            */
        }
    }
}
void CAN_Rcv_DataFromISR(CanMessage_t *RxMsg)
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
    /*
     CAN1: 两个编码器 4 个3508
     CAN2: 6个M2006  1 个3508
    */
    uint32_t id = msg->id;
    uint8_t can_index = msg->can_index;
    uint8_t len = msg->dlc;
    uint8_t *data = msg->data;

    if (can_index == 1)
    {
        switch (id)
        {
            // 编码器处理
            // 0x122 0x123是编码器的 id号 需要上位机设置一下先
        case 0x122:
        case 0x123:
            // TODO 完成编码器canframe解析
					log_i("test:id:0x123\n");
            break;
            // 电机处理
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
						log_i("test:id:0x201\n");
            get_dji_motor_measure(&motor_lift[id - 0x201], data);
            break;

        default:
            break;
        }
    }
    else if (can_index == 2)
    {
        switch (id)
        {
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
        case 0x205:
        case 0x206:
            get_dji_motor_measure(&motor_arm[id - 0x201], data);
            break;
        case 0x207:
            get_dji_motor_measure(&motor_stretch, data);
            break;

        default:
            break;
        }
    }
}

inline motorMeasure_t *get_motor_arm_measure_ptr(uint8_t i)
{
    return &motor_arm[i];
}
inline motorMeasure_t *get_motor_lift_measure_ptr(uint8_t i)
{
    return &motor_lift[i];
}
inline motorMeasure_t *get_motor_stretch_measure_ptr(uint8_t i)
{
    return &motor_stretch;
}
inline motorMeasure_t *get_motor_wrist_measure_ptr(uint8_t i)
{
    return &motor_wrist[i];
}