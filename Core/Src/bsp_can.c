#include "bsp_can.h"
#include "can.h"
#include "string.h"

#define LOG_TAG "BSP_CAN"
#include "elog.h"


#define CAN1_FILTER_MODE_MASK_ENABLE 1	///< CAN1过滤器模式选择：=1：屏蔽位模式  =0：屏蔽列表模式
#define CAN2_FILTER_MODE_MASK_ENABLE 1  ///< CAN2过滤器模式选择：=1：屏蔽位模式  =0：屏蔽列表模式

#define CAN1_BASE_ID  0x10F00266		///< 主CAN过滤ID
#define CAN2_BASE_ID  0x10F0F126		///< 从CAN过滤ID

#define CAN1_FILTER_BANK  0             ///< 主CAN过滤器组编号
#define CAN2_FILTER_BANK  14            ///< 从CAN过滤器组编号

/// CAN过滤器寄存器位宽类型定义
typedef union
{
    __IO uint32_t value;
    struct
    {
        uint8_t REV : 1;			///< [0]    ：未使用
        uint8_t RTR : 1;			///< [1]    : RTR（数据帧或远程帧标志位）
        uint8_t IDE : 1;			///< [2]    : IDE（标准帧或扩展帧标志位）
        uint32_t EXID : 18;			///< [21:3] : 存放扩展帧ID
        uint16_t STID : 11;			///< [31:22]: 存放标准帧ID
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
	}Sub;
}FMR_TypeDef;
HAL_StatusTypeDef CAN_SetAllReceivingFilters(void)
{
    CAN_FilterTypeDef canFilter;
    HAL_StatusTypeDef result = HAL_OK;

    // --- Configure CAN1 Filter ---
    canFilter.FilterBank = 0; // Use filter bank 0 for CAN1
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK; // ID Mask mode
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit scale for any ID type

    // To receive all frames, set FilterIdHigh/Low to 0x0000 and MaskIdHigh/Low to 0x0000.
    // A mask of 0 means "don't care" for all bits.
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;

    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0
    canFilter.FilterActivation = ENABLE; // Enable the filter
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
    canFilter.FilterBank = 14; // Use the first filter bank assigned to CAN2
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK; // ID Mask mode
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit scale

    // To receive all frames, again, set FilterIdHigh/Low to 0x0000 and MaskIdHigh/Low to 0x0000.
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;

    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO0 for CAN2
    canFilter.FilterActivation = ENABLE; // Enable the filter
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
 

///// 设置CAN1的过滤器（主CAN）
//static void CAN1_Filter_Config(void)
//{
//	CAN_FilterTypeDef sFilterConfig;
//    CAN_FilterRegTypeDef IDH = {0};
//    CAN_FilterRegTypeDef IDL = {0};

//	IDH.Sub.IDE  = 0;								// 标准帧
//	IDH.Sub.STID = 0;								// 标准帧ID值
//    IDH.Sub.EXID = (CAN1_BASE_ID >> 16) & 0xFFFF;	// 扩展帧高16位ID值
//	
//	IDL.Sub.IDE  = 1;								// 扩展帧
//	IDL.Sub.STID = 0;								// 标准帧ID值
//    IDL.Sub.EXID = (CAN1_BASE_ID & 0xFFFF);			// 扩展帧低16位ID值

//	sFilterConfig.FilterBank           = CAN1_FILTER_BANK;								// 设置过滤器组编号
//#if CAN1_FILTER_MODE_MASK_ENABLE
//    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;							// 屏蔽位模式
//#else
//	sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;							// 列表模式
//#endif
//    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;							// 32位宽
//    sFilterConfig.FilterIdHigh         = IDH.value;										// 标识符寄存器一ID高十六位，放入扩展帧位
//    sFilterConfig.FilterIdLow          = IDL.value;										// 标识符寄存器一ID低十六位，放入扩展帧位
//    sFilterConfig.FilterMaskIdHigh     = IDH.value;										// 标识符寄存器二ID高十六位，放入扩展帧位
//    sFilterConfig.FilterMaskIdLow      = IDL.value;										// 标识符寄存器二ID低十六位，放入扩展帧位
//    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;									// 过滤器组关联到FIFO0
//    sFilterConfig.FilterActivation     = ENABLE;										// **过滤器
//    sFilterConfig.SlaveStartFilterBank = CAN2_FILTER_BANK;								// 设置CAN2的起始过滤器组（对于单CAN的CPU或从CAN此参数无效；对于双CAN的CPU此参数为从CAN的起始过滤器组编号）
//    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
//	{
//		FMR_TypeDef regval = {0};
//		regval.value = hcan1.Instance->FMR;
//		log_i("------ CAN1:> FMR:0x%0X  CAN2SB:0x%X ", regval.value, regval.Sub.CAN2SB);
//	}
//}

///// 设置CAN2的过滤器（从CAN）
//static void CAN2_Filter_Config(void)
//{
//	CAN_FilterTypeDef sFilterConfig;
//    CAN_FilterRegTypeDef IDH = {0};
//    CAN_FilterRegTypeDef IDL = {0};

//	  IDH.Sub.IDE  = 0;
//	  IDH.Sub.STID = 0;
//    IDH.Sub.EXID = (CAN2_BASE_ID >> 16) & 0xFFFF;
//	
//	  IDL.Sub.IDE  = 1;
//	  IDL.Sub.STID = 0;
//    IDL.Sub.EXID = (CAN2_BASE_ID & 0xFFFF);

//    sFilterConfig.FilterBank           = CAN2_FILTER_BANK;								// 设置过滤器组编号
//#if CAN2_FILTER_MODE_MASK_ENABLE
//    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;							// 屏蔽位模式
//#else
//	sFilterConfig.FilterMode           = CAN_FILTERMODE_IDLIST;							// 列表模式
//#endif
//    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;							// 32位宽
//    sFilterConfig.FilterIdHigh         = IDH.value;										// 标识符寄存器一ID高十六位，放入扩展帧位
//    sFilterConfig.FilterIdLow          = IDL.value;										// 标识符寄存器一ID低十六位，放入扩展帧位
//    sFilterConfig.FilterMaskIdHigh     = IDH.value;										// 标识符寄存器二ID高十六位，放入扩展帧位
//    sFilterConfig.FilterMaskIdLow      = IDL.value;										// 标识符寄存器二ID低十六位，放入扩展帧位
//    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;									// 过滤器组关联到FIFO0
//    sFilterConfig.FilterActivation     = ENABLE;										// **过滤器
//    sFilterConfig.SlaveStartFilterBank = 28;											// 无效
//    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
//    {
//        Error_Handler();
//    }
//	{
//		FMR_TypeDef regval = {0};
//		regval.value = hcan2.Instance->FMR;
//		log_i("------ CAN2:> FMR:0x%0X  CAN2SB:0x%X  ", regval.value, regval.Sub.CAN2SB);
//	}
//}

/// CAN初始化
void CAN_Init(void)
{
		CAN_SetAllReceivingFilters();												// 初始化CNA过滤器
    HAL_CAN_Start(&hcan1);																// 启动CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);					// **CAN1 FIFO0
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * CAN数据传输
 * @param  buf    待发送的数据
 * @param  len    数据长度
 * @param  number CAN编号，=0：CAN1，=1：CAN2
 * @return        0：成功  other：失败
 */
uint8_t CAN_Transmit(const void* buf, uint32_t len, uint8_t number)
{
    uint32_t txmailbox = 0;
    uint32_t offset = 0;
    CAN_TxHeaderTypeDef hdr;

    hdr.IDE = CAN_ID_EXT;													// ID类型：扩展帧
    hdr.RTR = CAN_RTR_DATA;													// 帧类型：数据帧
    hdr.StdId = 0;															// 标准帧ID,最大11位，也就是0x7FF
    hdr.ExtId = number == 0 ? CAN1_BASE_ID : CAN2_BASE_ID;					// 扩展帧ID,最大29位，也就是0x1FFFFFFF
    hdr.TransmitGlobalTime = DISABLE;

    while (len != 0)
    {
        hdr.DLC = len > 8 ? 8 : len;			// 数据长度
        if (HAL_CAN_AddTxMessage(number == 0 ? &hcan1 : &hcan2, &hdr, ((uint8_t *)buf) + offset, &txmailbox) != HAL_OK)
            return 1;
        offset += hdr.DLC;
        len -= hdr.DLC;
    }
    return 0;
}

uint8_t CAN1_RX_STA = 0;		///< CAN1数据接收标志：[7]:数据 [6:0]:未使用
uint8_t CAN2_RX_STA = 0;		///< CAN2数据接收标志：[7]:数据 [6:0]:未使用

uint8_t CAN1_RX_BUF[8];			///< CAN1数据接收缓存
uint8_t CAN2_RX_BUF[8];			///< CAN2数据接收缓存

uint8_t CAN1_TX_BUF[8];			///< CAN1数据发送缓存
uint8_t CAN2_TX_BUF[8];			///< CAN2数据发送缓存

/**
 * CAN FIFO0 数据接收中断回调函数
 * @param hcan CAN句柄
 */
	/*CAN接收FIFO0挂起中断处理函数*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	  static CAN_RxHeaderTypeDef RxMessage;
    uint8_t  data[8];
    HAL_StatusTypeDef status;
    if(hcan == &hcan1) 
    {
        status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data);
        if (HAL_OK == status)
        {                             
            log_i("--->CAN1 Data Receieve!");
            log_i("RxMessage.StdId is %#x", RxMessage.StdId);
            log_i("data[0] is 0x%02x", data[0]);
            log_i("data[1] is 0x%02x", data[1]);
            log_i("data[2] is 0x%02x", data[2]);
            log_i("data[3] is 0x%02x", data[3]);
            log_i("<---");
        }
    }else if(hcan == &hcan2) 
    {
        status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data);
        if (HAL_OK == status)
        {                             
            log_i("--->CAN2 Data Receieve!");
            log_i("RxMessage.StdId is %#x", RxMessage.StdId);
            log_i("data[0] is 0x%02x", data[0]);
            log_i("data[1] is 0x%02x", data[1]);
            log_i("data[2] is 0x%02x", data[2]);
            log_i("data[3] is 0x%02x", data[3]);
            log_i("<---");
        }
    }
}


///< CAN数据处理函数
inline void CAN_RecvHandler(void)
{
    // CAN1有数据收到
    if (CAN1_RX_STA & 0x80)
    {
		int i = 0;
		memcpy(CAN1_TX_BUF, CAN1_RX_BUF, sizeof(CAN1_RX_BUF));		// 拷贝出数据
		CAN1_RX_STA = 0;											// 重置CAN1接收状态
		for(i = 0; i != 8; i++)
		{
			log_i("CAN1_TX_BUF[%d]:0x%X", i, CAN1_TX_BUF[i]);
		}
		log_i("\r\n\r\n");
    }

    // CAN2有数据收到
    if (CAN2_RX_STA & 0x80)
    {
		int i = 0;
		memcpy(CAN2_TX_BUF, CAN2_RX_BUF, sizeof(CAN2_RX_BUF));		// 拷贝出数据
		CAN2_RX_STA = 0;											// 重置CAN1接收状态
		for(i = 0; i != 8; i++)
		{
			log_i("CAN2_TX_BUF[%d]:0x%X", i, CAN2_TX_BUF[i]);
		}
		log_i("\r\n\r\n");
    }
}

