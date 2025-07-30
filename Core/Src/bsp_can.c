#include "bsp_can.h"
#define LOG_TAG "BSP_CAN"
#include "app_can.h"
#include "can.h"
#include "string.h"
#include "cmsis_os.h"
#include "elog.h"

#define CAN1_FILTER_MODE_MASK_ENABLE 1 ///< CAN1过滤器模式选择：=1：屏蔽位模式  =0：屏蔽列表模式
#define CAN2_FILTER_MODE_MASK_ENABLE 1 ///< CAN2过滤器模式选择：=1：屏蔽位模式  =0：屏蔽列表模式

#define CAN1_BASE_ID 0x10F00266 ///< 主CAN过滤ID
#define CAN2_BASE_ID 0x10F0F126 ///< 从CAN过滤ID

#define CAN1_FILTER_BANK 0  ///< 主CAN过滤器组编号
#define CAN2_FILTER_BANK 14 ///< 从CAN过滤器组编号

/// CAN过滤器寄存器位宽类型定义
typedef union
{
    __IO uint32_t value;
    struct
    {
        uint8_t REV : 1;    ///< [0]    ：未使用
        uint8_t RTR : 1;    ///< [1]    : RTR（数据帧或远程帧标志位）
        uint8_t IDE : 1;    ///< [2]    : IDE（标准帧或扩展帧标志位）
        uint32_t EXID : 18; ///< [21:3] : 存放扩展帧ID
        uint16_t STID : 11; ///< [31:22]: 存放标准帧ID
    } Sub;
} CAN_FilterRegTypeDef;

// CAN_FMR寄存器位宽类型定义
typedef union
{
    __IO uint32_t value;
    struct
    {
        uint8_t FINIT : 1;
        uint8_t RESERVER_0 : 7;
        uint8_t CAN2SB : 6;
        uint32_t RESERVER_1 : 18;
    } Sub;
} FMR_TypeDef;
HAL_StatusTypeDef CAN_SetAllReceivingFilters(void)
{
    CAN_FilterTypeDef canFilter;
    HAL_StatusTypeDef result = HAL_OK;

    // --- Configure CAN1 Filter ---
    canFilter.FilterBank = 0;                      // Use filter bank 0 for CAN1
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;  // ID Mask mode
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit scale for any ID type

    // To receive all frames, set FilterIdHigh/Low to 0x0000 and MaskIdHigh/Low to 0x0000.
    // A mask of 0 means "don't care" for all bits.
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;

    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0
    canFilter.FilterActivation = ENABLE;           // Enable the filter
    // SlaveStartFilterBank is crucial here for dual-CAN systems.
    // It tells CAN1 which filter bank range belongs to CAN2.
    canFilter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK)
    {
        log_e("Failed to configure CAN1 filter!");
        result = HAL_ERROR;
    }
    else
    {
        log_i("CAN1 filter set to receive all frames (Bank 0, assigned to CAN1).");
    }

    // --- Configure CAN2 Filter ---
    // Make sure hcan2 is properly initialized before calling this.
    // You should also have MX_CAN2_Init() called somewhere before this.
    canFilter.FilterBank = 14;                     // Use the first filter bank assigned to CAN2
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;  // ID Mask mode
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit scale

    // To receive all frames, again, set FilterIdHigh/Low to 0x0000 and MaskIdHigh/Low to 0x0000.
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;

    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0 for CAN2
    canFilter.FilterActivation = ENABLE;           // Enable the filter
    // SlaveStartFilterBank is not relevant when configuring CAN2's filters.
    // It's only used when configuring CAN1's filters to define the split point.
    // You can set it to any value (e.g., 0 or 28), or even omit it if the HAL
    // implementation correctly ignores it for CAN2. Setting it to 28 is safe
    // as it's beyond the last filter bank.
    canFilter.SlaveStartFilterBank = 28; // This parameter is ignored for CAN2

    if (HAL_CAN_ConfigFilter(&hcan2, &canFilter) != HAL_OK)
    {
        log_e("Failed to configure CAN2 filter!");
        result = HAL_ERROR;
    }
    else
    {
        log_i("CAN2 filter set to receive all frames (Bank %d, assigned to CAN2).", 14);
    }

    return result;
}

/// CAN初始化
void CAN_Init(void)
{
    CAN_SetAllReceivingFilters();                                      // 初始化CNA过滤器
    HAL_CAN_Start(&hcan1);                                             // 启动CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // **CAN1 FIFO0
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
uint32_t txmailbox;
/**
 * CAN数据传输
 * @param  buf    待发送的数据
 * @param  len    数据长度
 * @param  number CAN编号，=0：CAN1，=1：CAN2
 * @return        0：成功  other：失败
 */
HAL_StatusTypeDef CAN_Transmit(CanMessage_t *msg)
{
    /* 复制发送内容 */
    CAN_TxHeaderTypeDef pTXHeader;
    if (msg->dlc > 8 || msg->can_index == 0 || msg->can_index > 2)
    {
        log_e("Can Transmit send message error set !");
        return HAL_ERROR;
    }
    pTXHeader.DLC = (uint32_t)msg->dlc;
    // pTXHeader.IDE = CAN_ID_STD;
    pTXHeader.IDE = msg->ide ? CAN_ID_EXT : CAN_ID_STD;
    pTXHeader.RTR = CAN_RTR_DATA;
    if (msg->ide)
    {
        pTXHeader.ExtId = msg->id;
    }
    else
    {
        pTXHeader.StdId = msg->id;
    }

    pTXHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(msg->can_index == 1 ? &hcan1 : &hcan2, &pTXHeader, msg->data, &txmailbox) != HAL_OK)
    {
        log_e("HAL_CAN_AddTxMessage HAL_ERROR");
        log_e("TxMailbox:%d\t", txmailbox);
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef CAN_send_motor_currents(uint8_t can_index, uint32_t id_tag,
                                          int16_t current1, int16_t current2,
                                          int16_t current3, int16_t current4)
{
    CanMessage_t msg;

    msg.can_index = can_index;
    msg.ide = 0; // 标准帧
    msg.id = id_tag;
    msg.dlc = 8;
	
    // 填充数据 (高位在前)
    msg.data[0] = (uint8_t)(current1 >> 8);
    msg.data[1] = (uint8_t)(current1);
    msg.data[2] = (uint8_t)(current2 >> 8);
    msg.data[3] = (uint8_t)(current2);
    msg.data[4] = (uint8_t)(current3 >> 8);
    msg.data[5] = (uint8_t)(current3);
    msg.data[6] = (uint8_t)(current4 >> 8);
    msg.data[7] = (uint8_t)(current4);

    uint8_t retry_count = 0;
    while (CAN_Transmit(&msg) != HAL_OK)
    {
        retry_count++;
        if (retry_count > 3) // 如果重试3次还不行，就真的报错放弃
        {
            log_e("CAN send failed after 5 retries!");
            return HAL_ERROR;
        }
        // 等待一个非常短的时间，给硬件发送的机会
        // 如果在RTOS任务中，用osDelay(1)；如果可能在中断中，用HAL_Delay()或微秒延时
        osDelay(1); // 等待1ms
    }
    return HAL_OK;
}

/**
 * CAN FIFO0 数据接收中断回调函数
 * @param hcan CAN句柄
 */
/*CAN接收FIFO0挂起中断处理函数 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_RxHeaderTypeDef RxMessage;
    static CanMessage_t Message;
    HAL_StatusTypeDef status;
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, Message.data); // 接收CAN消息
    if (HAL_OK == status)
    {
        Message.ide = (RxMessage.IDE == 0) ? false : true;
        Message.id = Message.ide ? RxMessage.ExtId : RxMessage.StdId;
        Message.can_index = (hcan == &hcan1) ? 1 : 2;
        CAN_Rcv_DataFromISR(&Message);
    }
}
