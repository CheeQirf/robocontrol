#include "bsp_uart.h"
#include "lwrb.h"
#include "usart.h"
#include "dma.h"

#define UART4_RX_DMA_BUFFER_LEN (20u)
#define UART4_RX_RB_LEN (128u)

lwrb_t usart4_rx_rb;// Ring buffer instance for TX data
lwrb_t usart4_tx_rb;// Ring buffer instance for TX data

uint8_t usart4_rx_dma_buffer[UART4_RX_DMA_BUFFER_LEN];
uint8_t usart4_rx_rb_data[UART4_RX_RB_LEN];// Ring buffer data array for RX DMA
uint8_t usart4_tx_rb_data[2280];// Ring buffer data array for TX DMA

volatile size_t usart4_tx_dma_current_len;// Length of currently active TX DMA transfer
volatile uint8_t usart4_rx_flag;

uint8_t USART4_Start_DmaTx(void)
{
    uint32_t primask;
    uint8_t started = 0;
    
    primask = __get_PRIMASK();
    __disable_irq();

    if (usart4_tx_dma_current_len == 0
            && (usart4_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart4_tx_rb)) > 0) 
    {
        __HAL_DMA_DISABLE(&hdma_uart4_tx);

        __HAL_DMA_CLEAR_FLAG(&hdma_uart4_tx
            , __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_uart4_tx));
        __HAL_DMA_CLEAR_FLAG(&hdma_uart4_tx
            , __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_uart4_tx));
        __HAL_DMA_CLEAR_FLAG(&hdma_uart4_tx
            , __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_uart4_tx));
//        __HAL_DMA_CLEAR_FLAG(&hdma_uart4_tx
//            , __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_uart4_tx));

        HAL_UART_Transmit_DMA(&huart4
            , (uint8_t*)lwrb_get_linear_block_read_address(&usart4_tx_rb)
            , (uint16_t)usart4_tx_dma_current_len);

        started = 1;
    }
    __set_PRIMASK(primask);
    
    return started;
}
void USART4_TxTcCb(UART_HandleTypeDef *huart)
{
        lwrb_skip(&usart4_tx_rb, usart4_tx_dma_current_len);	/* Skip sent data, mark as read */
        usart4_tx_dma_current_len = 0;          			   /* Clear length variable */
        USART4_Start_DmaTx();         					       /* Start sending more data */
}
void USART4_Init(void)
{
    lwrb_init(&usart4_tx_rb, usart4_tx_rb_data, sizeof(usart4_tx_rb_data));
    lwrb_init(&usart4_rx_rb, usart4_rx_rb_data, sizeof(usart4_rx_rb_data));

    //HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_dma_buffer, sizeof(usart1_rx_dma_buffer));
    //HAL_UART_RegisterRxEventCallback(&huart1, USART1_RxEventCb);
    HAL_UART_RegisterCallback(&huart4, HAL_UART_TX_COMPLETE_CB_ID, USART4_TxTcCb);
}

void USART4_SendData(const uint8_t *p_data, uint16_t len)
{
    lwrb_write(&usart4_tx_rb, p_data, len); /* Write data to TX buffer for loopback */
    USART4_Start_DmaTx();
}
