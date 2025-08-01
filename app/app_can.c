#include "app_can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#define LOG_TAG "APP_CAN"
#include "elog.h"
#include "queue.h"
#include "bsp_can.h"
#include "app_motor.h"

#define CAN_QUEUE_LENGTH 5

// static xQueueHandle xCANSendQueue = NULL; //
static xQueueHandle xCANRcvQueue = NULL; //

static motorMeasure_t motor_arm[4];
static motorMeasure_t motor_wrist[2];
static motorMeasure_t motor_lift[4];
static motorMeasure_t motor_stretch;
static OID_Encoder_t oid_encoder[2];
void canDispatch(CanMessage_t *msg);

#define get_dji_motor_measure(ptr, data)                                    \
    {                                                                       \
        (ptr)->last_angle = (ptr)->angle;                                   \
        (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]);              \
        (ptr)->rpm = (int16_t)((data)[2] << 8 | (data)[3]);                 \
        (ptr)->current = (int16_t)((data)[4] << 8 | (data)[5]);             \
        (ptr)->temperture = (data)[6];                                      \
        int16_t delta_angle = (ptr)->angle - (ptr)->last_angle;             \
                                                                            \
        if (delta_angle > 4096)                                             \
        {                                                                   \
            /*                                                              \
             * 发生了反转过零 (下绕)                               \
             * 物理过程: e.g., 从 10 -> 0 -> 8191 -> 8190              \
             * 原始读数: last_angle = 10, angle = 8190                  \
             * 直接计算: delta = 8190 - 10 = 8180 (一个大的正数)  \
             * 期望结果: 一个小的负数                             \
             * 修正方法: 减去一整圈                                \
             */                                                             \
            delta_angle -= 8192;                                            \
        }                                                                   \
        else if (delta_angle < -4096)                                       \
        {                                                                   \
            /*                                                              \
             * 发生了正转过零 (上绕)                               \
             * 物理过程: e.g., 从 8190 -> 8191 -> 0 -> 10              \
             * 原始读数: last_angle = 8190, angle = 10                  \
             * 直接计算: delta = 10 - 8190 = -8180 (一个大的负数) \
             * 期望结果: 一个小的正数                             \
             * 修正方法: 加上一整圈                                \
             */                                                             \
            delta_angle += 8192;                                            \
        }                                                                   \
        (ptr)->pos += delta_angle;                                          \
    }
#define get_encoder_data(ptr, data)                                                    \
    {                                                                                  \
        (ptr)->encoder_count = data[6] << 24 | data[5] << 16 | data[4] << 8 | data[3]; \
    }
void CAN_Rx_Task(void *pvParameters)
{
    xCANRcvQueue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CanMessage_t));
    CanMessage_t RxMsg;
    for (;;)
    {
        if (xQueueReceive(xCANRcvQueue, &RxMsg, 100) == pdTRUE)
        {

            taskENTER_CRITICAL(); // 临界区保护
            canDispatch(&RxMsg);
            taskEXIT_CRITICAL();
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
        case 0x01:
        case 0x02:
            // TODO 完成编码器canframe解析
            if (msg->dlc != 7)
                log_w("wrong data length\n");
            get_encoder_data(&oid_encoder[id - 0x001], data);
            break;
            // 电机处理
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
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
            get_dji_motor_measure(&motor_arm[id - 0x201], data);
            break;
        case 0x205:
        case 0x206:
            get_dji_motor_measure(&motor_wrist[id - 0x205], data);
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
inline motorMeasure_t *get_motor_stretch_measure_ptr(void)
{
    return &motor_stretch;
}
inline motorMeasure_t *get_motor_wrist_measure_ptr(uint8_t i)
{
    return &motor_wrist[i];
}
inline OID_Encoder_t *get_encoder_measure_ptr(uint8_t i)
{
    return &oid_encoder[i];
}