#include "bsp_rs485.h"
#include "usart.h" // 包含HAL库中USART句柄的定义，如 huart1, huart2

// 外部声明串口句柄，这些句柄在 CubeMX 生成的 main.c 或 usart.c 中定义
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// 定义接收缓冲区
#define RS485_RX_BUFFER_SIZE 64 // 根据您的实际需要调整缓冲区大小
static uint8_t rs485_1_rx_buf[RS485_RX_BUFFER_SIZE];
static uint8_t rs485_2_rx_buf[RS485_RX_BUFFER_SIZE];

/**
  * @brief  RS485 1 非阻塞发送数据 (对应 USART1)
  * 此函数会立即返回。发送完成后的操作（如切换方向）将在中断回调中进行。
  * @param  pData: 指向待发送数据的指针
  * @param  Size: 待发送数据的字节数
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef RS485_1_Transmit_IT(uint8_t *pData, uint16_t Size)
{
    // 1. 切换到发送模式
    RS485_1_TRANSMIT_MODE(); // PC15 拉高

    // 2. 启动中断发送
    // 函数会立即返回，发送过程在后台进行
    return HAL_UART_Transmit_IT(&huart1, pData, Size);
}

/**
  * @brief  RS485 2 非阻塞发送数据 (对应 USART2)
  * 此函数会立即返回。发送完成后的操作（如切换方向）将在中断回调中进行。
  * @param  pData: 指向待发送数据的指针
  * @param  Size: 待发送数据的字节数
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef RS485_2_Transmit_IT(uint8_t *pData, uint16_t Size)
{
    // 1. 切换到发送模式
    RS485_2_TRANSMIT_MODE(); // PB3 拉高

    // 2. 启动中断发送
    return HAL_UART_Transmit_IT(&huart2, pData, Size);
}

/**
  * @brief  初始化所有 RS485 端口为默认状态，并启动非阻塞接收。
  * 此函数应在系统启动时调用一次。
  * @retval None
  */
void RS485_Init(void)
{
    // RS485_1 默认高电平 (发送模式)，但通常启动时应设置为接收模式，以便监听总线
    // 如果系统初始启动就应该接收，则改为 RECEIVE_MODE
    RS485_1_RECEIVE_MODE(); // 建议初始设为接收模式，避免总线冲突

    // RS485_2 默认低电平 (接收模式)
    RS485_2_RECEIVE_MODE();

    // 启动非阻塞接收
    HAL_UART_Receive_IT(&huart1, rs485_1_rx_buf, RS485_RX_BUFFER_SIZE);
    HAL_UART_Receive_IT(&huart2, rs485_2_rx_buf, RS485_RX_BUFFER_SIZE);
}

/**
  * @brief  UART 发送完成回调函数。
  * 当数据通过中断发送完成时，此函数会被 HAL 库调用。
  * @param  huart: 指向 UART_HandleTypeDef 结构体的指针。
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // RS485_1 发送完成，切换回接收模式
        RS485_1_RECEIVE_MODE();
        // 如果有需要，可以在这里发出一个信号量或标志，通知任务发送已完成
    }
    else if (huart->Instance == USART2)
    {
        // RS485_2 发送完成，切换回接收模式
        RS485_2_RECEIVE_MODE();
        // 如果有需要，可以在这里发出一个信号量或标志，通知任务发送已完成
    }
}

/**
  * @brief  UART 接收完成回调函数。
  * 当通过中断接收到指定长度的数据时，此函数会被 HAL 库调用。
  * @param  huart: 指向 UART_HandleTypeDef 结构体的指针。
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 处理 RS485_1 接收到的数据
        // 例如：将 rs485_1_rx_buf 中的数据复制到 FreeRTOS 队列中
        // xQueueSendFromISR(xRS485_1_RxQueue, rs485_1_rx_buf, &xHigherPriorityTaskWoken);
        // 重要：处理完数据后，需要重新启动接收，以便接收下一帧数据
        HAL_UART_Receive_IT(&huart1, rs485_1_rx_buf, RS485_RX_BUFFER_SIZE);
    }
    else if (huart->Instance == USART2)
    {
        // 处理 RS485_2 接收到的数据
        // 例如：将 rs485_2_rx_buf 中的数据复制到 FreeRTOS 队列中
        // xQueueSendFromISR(xRS485_2_RxQueue, rs485_2_rx_buf, &xHigherPriorityTaskWoken);
        // 重要：处理完数据后，需要重新启动接收
        HAL_UART_Receive_IT(&huart2, rs485_2_rx_buf, RS485_RX_BUFFER_SIZE);
    }
    // portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); // 如果使用了 FreeRTOS 队列
}

/**
  * @brief  UART 错误回调函数。
  * @param  huart: 指向 UART_HandleTypeDef 结构体的指针。
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 在这里处理 UART 错误，例如帧错误、过载错误等
    // 可以清除错误标志并重新启动接收
    // __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE);
    // HAL_UART_Receive_IT(huart, rx_buffer, RX_BUFFER_SIZE); // 重新启动接收
}