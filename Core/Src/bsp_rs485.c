#include "bsp_rs485.h"
#include "usart.h"
#include "app_485.h"

extern UART_HandleTypeDef huart1;

static uint8_t rs485_1_rx_buf[RS485_1_RX_BUFFER_SIZE];

HAL_StatusTypeDef BSP_RS485_1_Transmit_IT(uint8_t *pData, uint16_t Size)
{
    // 1. 切换到发送模式
    RS485_1_TRANSMIT_MODE();

    // 2. 启动中断发送
    return HAL_UART_Transmit_IT(&huart1, pData, Size);
}

void RS485_Init(void)
{
    // RS485_1 默认设置为接收模式，以便监听总线
    RS485_1_RECEIVE_MODE();
    HAL_UART_Receive_IT(&huart1, rs485_1_rx_buf, RS485_1_RX_BUFFER_SIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // RS485_1 发送完成，切换回接收模式
        RS485_1_RECEIVE_MODE();
        // 如果有需要，可以在这里发出一个信号量或标志，通知任务发送已完成
        // 例如：xSemaphoreGiveFromISR(xUnitreeTxCompleteSemaphore, &xHigherPriorityTaskWoken);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; // 用于 FreeRTOS 任务调度

    if (huart->Instance == USART1)
    {
        // 收到一帧完整数据
        // 将接收到的数据发送到 FreeRTOS 队列，由 Unitree_RS485_Rx_Task 进行处理
        // 传递数据副本，防止 unitree_rx_frame_buffer 被覆盖
        if (xQueueSendToBackFromISR(xUnitreeRxQueue, rs485_1_rx_buf, &xHigherPriorityTaskWoken) != pdPASS)
        {
            // 队列已满，数据可能丢失。这里可以添加警告日志。
            // log_w("Unitree Rx Queue Full! Dropping frame.");
        }

        // 重要：处理完数据后，需要重新启动接收，以便接收下一帧数据
        // 继续期望接收一个完整的宇树电机反馈帧
        HAL_UART_Receive_IT(&huart1, rs485_1_rx_buf, RS485_1_RX_BUFFER_SIZE);
    }
    // 如果使用了 FreeRTOS 队列，并且中断唤醒了更高优先级的任务，需要执行任务切换
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
