#include "bsp_uart.h"
#include "unity.h"
#include "mock_usart.h"
#include "mock_dma.h"
#include "mock_lwrb.h"

// Global variables for testing
lwrb_t usart4_rx_rb;
lwrb_t usart4_tx_rb;
uint8_t usart4_rx_dma_buffer[UART4_RX_DMA_BUFFER_LEN];
uint8_t usart4_rx_rb_data[UART4_RX_RB_LEN];
uint8_t usart4_tx_rb_data[2280];
volatile size_t usart4_tx_dma_current_len;
volatile uint8_t usart4_rx_flag;

void setUp(void)
{
    // Initialize ring buffers
    lwrb_init_ExpectAndReturn(&usart4_tx_rb, usart4_tx_rb_data, sizeof(usart4_tx_rb_data), 0);
    lwrb_init_ExpectAndReturn(&usart4_rx_rb, usart4_rx_rb_data, sizeof(usart4_rx_rb_data), 0);

    // Initialize HAL UART and DMA mocks
    HAL_UART_RegisterCallback_ExpectAndReturn(&huart4, HAL_UART_TX_COMPLETE_CB_ID, USART4_TxTcCb, HAL_OK);

    USART4_Init();
}

void tearDown(void)
{
    // Reset global variables
    usart4_tx_dma_current_len = 0;
    usart4_rx_flag = 0;
}

void test_USART4_Start_DmaTx_When_No_Data(void)
{
    // Setup
    usart4_tx_dma_current_len = 0;
    lwrb_get_linear_block_read_length_ExpectAndReturn(&usart4_tx_rb, 0);

    // Execute
    uint8_t result = USART4_Start_DmaTx();

    // Verify
    TEST_ASSERT_EQUAL(0, result);
}

void test_USART4_Start_DmaTx_When_Data_Available(void)
{
    // Setup
    usart4_tx_dma_current_len = 0;
    lwrb_get_linear_block_read_length_ExpectAndReturn(&usart4_tx_rb, 10);
    lwrb_get_linear_block_read_address_ExpectAndReturn(&usart4_tx_rb, (void *)0x12345678);
    HAL_UART_Transmit_DMA_ExpectAndReturn(&huart4, (uint8_t *)0x12345678, 10, HAL_OK);

    // Execute
    uint8_t result = USART4_Start_DmaTx();

    // Verify
    TEST_ASSERT_EQUAL(1, result);
}

void test_USART4_TxTcCb(void)
{
    // Setup
    usart4_tx_dma_current_len = 10;
    lwrb_skip_ExpectAndReturn(&usart4_tx_rb, 10, 10);
    USART4_Start_DmaTx_ExpectAndReturn(1);

    // Execute
    USART4_TxTcCb(&huart4);

    // Verify
    TEST_ASSERT_EQUAL(0, usart4_tx_dma_current_len);
}

void test_USART4_SendData(void)
{
    // Setup
    uint8_t test_data[] = {0x01, 0x02, 0x03};
    lwrb_write_ExpectAndReturn(&usart4_tx_rb, test_data, 3, 3);
    USART4_Start_DmaTx_ExpectAndReturn(1);

    // Execute
    USART4_SendData(test_data, 3);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_USART4_Start_DmaTx_When_No_Data);
    RUN_TEST(test_USART4_Start_DmaTx_When_Data_Available);
    RUN_TEST(test_USART4_TxTcCb);
    RUN_TEST(test_USART4_SendData);
    return UNITY_END();
}