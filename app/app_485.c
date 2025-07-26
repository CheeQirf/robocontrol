#include "app_485.h"
#include "bsp_rs485.h" // 包含 bsp_rs485，以便调用发送函数
#include "string.h" 
#define LOG_TAG "APP_485"
#include "elog.h"
#include <math.h> 

UnitreeMotor_Measure_t motor_arm[2]; // 对应两个宇树电机

// FreeRTOS 队列定义
xQueueHandle xUnitreeRxQueue = NULL; // 用于从中断中接收完整的宇树电机反馈帧

static void Unitree_Parse_Feedback_Frame(uint8_t *frame_data);

// CRC16 函数实现 (示例，请根据协议具体实现)
uint16_t unitree_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0x0000; // 协议中通常会指定初始值，例如 0xFFFF 或 0x0000
    // 对于 CRC16-CCITT (XModem)，多项式为 0x1021
    // 您需要根据宇树电机的具体实现来编写这个函数
    for (uint32_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8); // Cast data[i] to uint16_t before shifting
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 初始化 RS485 任务和队列
 */
void app_485_Init(void)
{
    // 创建 FreeRTOS 队列
    xUnitreeRxQueue = xQueueCreate(5, UNITREE_RX_FRAME_SIZE); // 队列深度5，每个项大小为一帧

    // 调用 BSP 层初始化 RS485 硬件和启动接收
    BSP_RS485_Init();

    log_i("Unitree RS485 app initialized.");
}

/**
 * @brief RS485 接收数据处理任务
 */
void Unitree_RS485_Rx_Task(void *pvParameters)
{
    uint8_t received_frame[UNITREE_RX_FRAME_SIZE];

    for (;;)
    {
        // 从队列中等待接收数据
        if (xQueueReceive(xUnitreeRxQueue, received_frame, portMAX_DELAY) == pdTRUE)
        {
            // 收到一帧完整数据，开始解析
            // 1. 检查帧头
            if (received_frame[0] == 0xFD && received_frame[1] == 0xEE)
            {
                // 2. 校验 CRC
                // CRC 在帧的最后两个字节 (Byte 14 和 Byte 15)
                // 假设 CRC 是高字节在前 (Big-Endian)，即 data[14]是高8位，data[15]是低8位
                // **请根据宇树电机协议确认 CRC 字节顺序**
                uint16_t received_crc = (uint16_t)((received_frame[UNITREE_RX_FRAME_SIZE - 2] << 8) | received_frame[UNITREE_RX_FRAME_SIZE - 1]);
                uint16_t calculated_crc = unitree_crc16(received_frame, UNITREE_RX_FRAME_SIZE - 2); // 校验前 14 字节

                if (received_crc == calculated_crc)
                {
                    // 3. CRC 校验通过，解析数据
                    Unitree_Parse_Feedback_Frame(received_frame);
                    // log_i("Unitree feedback frame parsed. ID: %d", received_frame[2] & 0x0F);
                }
                else
                {
                    log_w("Unitree feedback CRC error! Received: 0x%04X, Calculated: 0x%04X", received_crc, calculated_crc);
                }
            }
            else
            {
                // log_w("Unitree feedback frame head error! 0x%02X 0x%02X", received_frame[0], received_frame[1]);
            }
        }
    }
}

/**
 * @brief 解析宇树电机反馈数据帧
 * @param frame_data 16 字节的反馈帧数据
 */
static void Unitree_Parse_Feedback_Frame(uint8_t *frame_data)
{
    
    uint8_t motor_id = frame_data[2] & 0x0F; // ID 在字节 2 的低 4 位

    if (motor_id < 2) // 假设只有两个宇树电机，ID为0和1
    {
        motor_arm[motor_id].id = motor_id;
        motor_arm[motor_id].status = (frame_data[2] >> 4) & 0x07; // Status 在字节 2 的高 3 位

        // 解析有符号 16 位值 (小端): Byte X+1 << 8 | Byte X
        // τ_fbk (24-39 bit) -> 字节 4, 5
        motor_arm[motor_id].tau_fbk = (int16_t)((frame_data[5] << 8) | frame_data[4]);
        // ω_fbk (40-55 bit) -> 字节 6, 7
        motor_arm[motor_id].omega_fbk_raw = (int16_t)((frame_data[7] << 8) | frame_data[6]);

        // 解析有符号 32 位值 (小端): Byte X+3 << 24 | Byte X+2 << 16 | Byte X+1 << 8 | Byte X
        // θ_fbk (56-87 bit) -> 字节 8, 9, 10, 11
        motor_arm[motor_id].theta_fbk_raw = (int32_t)(((uint32_t)frame_data[11] << 24) |
                                                      ((uint32_t)frame_data[10] << 16) |
                                                      ((uint32_t)frame_data[9] << 8) |
                                                      (uint32_t)frame_data[8]);

        // TEMP (88-95 bit) -> 字节 12
        motor_arm[motor_id].temp = frame_data[12];
        // MERROR (96-98 bit) -> 字节 13 的低 3 位
        motor_arm[motor_id].merror = (frame_data[13] & 0x07);

        // FORCE (99-110 bit, 12bit) -> 字节 13 的高 4 位 和 字节 14
        // (frame_data[13] 的高4位作为高4bit) | frame_data[14] 作为低8bit
        motor_arm[motor_id].force_raw = ((uint16_t)(frame_data[13] & 0xF0) << 4) | (uint16_t)frame_data[14];

        // 转换为实际物理量
        motor_arm[motor_id].tau_N_m = (float)motor_arm[motor_id].tau_fbk / 256.0f;
        motor_arm[motor_id].omega_rad_s = (float)motor_arm[motor_id].omega_fbk_raw / 256.0f;
        motor_arm[motor_id].theta_rad = (float)motor_arm[motor_id].theta_fbk_raw / 32768.0f * (2.0f * (float)3.1415926);
    }
    else
    {
        log_w("Received Unitree frame with invalid motor ID: %d", motor_id);
    }
}

/**
 * @brief 发送宇树电机控制指令
 * @param motor_id 目标电机 ID
 * @param tau_set 期望扭矩原始值 (int16_t)
 * @param omega_set 期望速度原始值 (int16_t)
 * @param theta_set 期望位置原始值 (int32_t)
 * @param Kp_pos 位置刚度系数 (int16_t)
 * @param Kp_spd 速度阻尼系数 (int16_t)
 * @param motor_mode 电机工作模式 (3bit)
 */
void Unitree_RS485_Send_Command(uint8_t motor_id, int16_t tau_set, int16_t omega_set, int32_t theta_set, int16_t Kp_pos, int16_t Kp_spd, uint8_t motor_mode)
{
    uint8_t tx_buffer[UNITREE_TX_FRAME_SIZE]; // 控制帧大小为 17 字节

    // 填充帧头
    tx_buffer[0] = 0xFE; 
    tx_buffer[1] = 0xEE; 

    // 填充 ID 和 STATUS
    tx_buffer[2] = ((motor_mode & 0x07) << 4) | (motor_id & 0x0F); // ID (低4位) | STATUS (高3位)

    // 填充期望扭矩 (tau_set, 16bit, 假设小端)
    tx_buffer[3] = (uint8_t)(tau_set & 0xFF);
    tx_buffer[4] = (uint8_t)((tau_set >> 8) & 0xFF);

    // 填充期望速度 (omega_set, 16bit, 假设小端)
    tx_buffer[5] = (uint8_t)(omega_set & 0xFF);
    tx_buffer[6] = (uint8_t)((omega_set >> 8) & 0xFF);

    // 填充期望位置 (theta_set, 32bit, 假设小端)
    tx_buffer[7] = (uint8_t)(theta_set & 0xFF);
    tx_buffer[8] = (uint8_t)((theta_set >> 8) & 0xFF);
    tx_buffer[9] = (uint8_t)((theta_set >> 16) & 0xFF);
    tx_buffer[10] = (uint8_t)((theta_set >> 24) & 0xFF);

    // 填充 Kp_pos (16bit, 假设小端)
    tx_buffer[11] = (uint8_t)(Kp_pos & 0xFF);
    tx_buffer[12] = (uint8_t)((Kp_pos >> 8) & 0xFF);

    // 填充 Kp_spd (16bit, 假设小端)
    tx_buffer[13] = (uint8_t)(Kp_spd & 0xFF);
    tx_buffer[14] = (uint8_t)((Kp_spd >> 8) & 0xFF);

    // 计算并填充 CRC16 (CRC 覆盖前 15 字节，共 17 字节，CRC 是最后 2 字节)
    uint16_t crc = unitree_crc16(tx_buffer, 15); // CRC 校验前 15 字节
    tx_buffer[15] = (uint8_t)(crc & 0xFF); // CRC 低字节
    tx_buffer[16] = (uint8_t)((crc >> 8) & 0xFF); // CRC 高字节

    // 调用 BSP 层发送函数，它会处理方向切换
    if (BSP_RS485_1_Transmit_IT(tx_buffer, sizeof(tx_buffer)) != HAL_OK)
    {
        log_e("Unitree RS485 Transmit_IT Failed!");
        // 注意：这里不需要手动切换回接收模式，因为 BSP_RS485_1_Transmit_IT
        // 内部已经由 TxCpltCallback 处理了
    }
}

/**
 * @brief 获取宇树电机测量数据指针
 * @param i 电机索引 (0 或 1)
 * @return 指向 UnitreeMotor_Measure_t 结构体的指针
 */
inline UnitreeMotor_Measure_t* get_unitree_motor_measure_ptr(uint8_t i)
{
    if (i < 2)
    {
        return &motor_arm[i];
    }
    return NULL; // 无效索引
}