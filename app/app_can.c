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

static motorMeasure_t motor_claw[4];//爪子的4个M2006电机
static motorMeasure_t motor_wrist[2];
static motorMeasure_t motor_lift[4];
static motorMeasure_t motor_stretch;//一个M3508 用来伸长手臂
static OID_Encoder_t motor_encoder[2];

void canDispatch(CanMessage_t *msg);

#define CAN_QUEUE_LENGTH 5

#define get_dji_motor_measure(ptr, data)                         \
    do{                                                            \
        (ptr)->last_angle = (ptr)->angle;                        \
        (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]);   \
        (ptr)->rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperture = (data)[6];                           \
    }while(0)
#define get_encoder_count(ptr, data) \
		do{                                  \
			(ptr)->encoder_count = data[5] << 16 | data[4] << 8 | data[3];\
		}while(0)
#define get_encoder_rads_count(ptr,data)\
		do{                         \
			(ptr)->encoder_rads_count = data[6]<<24 | data[5] <<16 | data[4]<<8 | data[3];\
		}while(0)

void APP_CAN_Init(void) {
    xCANSendQueue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CanMessage_t));
    xCANRcvQueue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CanMessage_t));
}
static void CAN_Rx_Task(void *pvParameters)
{
    static CanMessage_t RxMsg; // 接受用的变量
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
        case 0x01:
        case 0x02:
						if(data[2]==0x01)
								get_encoder_count(&motor_encoder[id-0x01],data);
						else if (data[2]==0x0A)
								get_encoder_rads_count(&motor_encoder[id-0x01],data);
						log_i("test_encoder\n");
						break;
        case 0x201:
        case 0x202:
        case 0x203:
        case 0x204:
            get_dji_motor_measure(&motor_lift[id - 0x201], data);
						log_i("test_m3508\n");
            break;
        default:
						//log_w("No compatible id for can %d\n",can_index);
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
            get_dji_motor_measure(&motor_claw[id - 0x201], data);
            break;
        case 0x207://M3508
            get_dji_motor_measure(&motor_stretch, data);
            break;

        default:
					//log_w("No compatible id for can %d\n",can_index);
            break;
        }
    }
}

inline motorMeasure_t *get_motor_claw_measure_ptr(uint8_t i)
{
    return &motor_claw[i];
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
inline OID_Encoder_t * get_encoder_measuer_ptr(uint8_t i)
{
	return &motor_encoder[i];
}