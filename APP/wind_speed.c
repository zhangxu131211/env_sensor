#include "wind_speed.h"
#include "usart.h"
#include "stdio.h"
#include <string.h>

// ************************** 全局变量定义 **************************
static WindSpeed_DataDef g_wind_data = {0};
uint8_t g_rx_buf[32] = {0};        // 接收缓冲区
uint8_t g_rx_len = 0;             // 接收长度
uint8_t g_rx_complete = 0;         // 接收完成标志
static uint16_t crc_calc, crc_recv;
// ************************** 私有函数声明 **************************
static uint16_t Modbus_CRC16(uint8_t *buf, uint8_t len);

// ************************** CRC16-Modbus校验 **************************
static uint16_t Modbus_CRC16(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

// ************************** 风速传感器初始化 **************************
void WindSpeed_Init(void) {
    memset(&g_wind_data, 0, sizeof(WindSpeed_DataDef));
    g_rx_len = 0;
    g_rx_complete = 0;
    HAL_UART_Receive_IT(&huart2, g_rx_buf, WIND_FRAME_LEN);
    DEBUG_LOG("风速传感器初始化完成\r\n");
}

// ************************** 帧解析函数 **************************
HAL_StatusTypeDef WindSpeed_ParseFrame(void) {
    DEBUG_LOG("开始风速数据解析...\r\n");
    // 1. 校验帧长度，风速传感器固定9字节帧
    if (g_rx_len != WIND_FRAME_LEN) {
        DEBUG_LOG("帧长度错误: 期望%d，实际%d\r\n", WIND_FRAME_LEN, g_rx_len);
        return HAL_ERROR;
    }

    // 2. 校验地址和功能码
    if (g_rx_buf[0] != WIND_MODBUS_ADDR || g_rx_buf[1] != WIND_MODBUS_FUNC) {
        DEBUG_LOG("地址/功能码错误: 地址0x%02X，功能码0x%02X\r\n", g_rx_buf[0], g_rx_buf[1]);
        return HAL_ERROR;
    }

    // 3. 校验CRC16，帧最后2字节为CRC
    crc_recv = (g_rx_buf[8] << 8) | g_rx_buf[7];  // 接收到的CRC（低字节在前）
    crc_calc = Modbus_CRC16(g_rx_buf, WIND_FRAME_LEN - 2);  // 计算CRC
    if (crc_calc != crc_recv) {
        DEBUG_LOG("CRC校验失败: 计算0x%04X，接收0x%04X\r\n", crc_calc, crc_recv);
        return HAL_ERROR;
    }

    // 4. 解析数据：0x0024=36=3.6m/s，0x0003=3级
    uint16_t speed_raw = (g_rx_buf[3] << 8) | g_rx_buf[4];
    g_wind_data.wind_speed = speed_raw / 10.0f;  // 单位为0.1m/s 转 m/s
    g_wind_data.wind_grade = (g_rx_buf[5] << 8) | g_rx_buf[6];
    g_wind_data.is_valid = 1;
    g_wind_data.err_cnt = 0;

    // 打印解析结果
    DEBUG_LOG("风速解析成功: 速度%.1f m/s，风级%d\r\n",
           g_wind_data.wind_speed, g_wind_data.wind_grade);

    return HAL_OK;
}



// 修改WindSpeed_Query函数中的发送部分
HAL_StatusTypeDef WindSpeed_Query(void)
{
    uint8_t query_frame[8] = {0};
    uint16_t crc;
    HAL_StatusTypeDef status;

    // 1. 组装查询帧
    query_frame[0] = WIND_MODBUS_ADDR;
    query_frame[1] = WIND_MODBUS_FUNC;
    query_frame[2] = (WIND_REG_START >> 8);
    query_frame[3] = (WIND_REG_START & 0xFF);
    query_frame[4] = (WIND_REG_LEN >> 8);
    query_frame[5] = (WIND_REG_LEN & 0xFF);
    crc = Modbus_CRC16(query_frame, 6);
    query_frame[6] = (crc & 0xFF);
    query_frame[7] = (crc >> 8);

    // 2. 清除接收缓冲区，避免旧数据干扰
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    g_rx_len = 0;
    g_rx_complete = 0;

    // 3. 打印要发送的查询帧（调试用）
    
    // DEBUG_LOG("发送查询帧: ");
    // for (int i = 0; i < 8; i++) {
    //     DEBUG_LOG("%02X ", query_frame[i]);
    // }
    // DEBUG_LOG("\r\n");

    HAL_UART_Receive_IT(&huart2, g_rx_buf, WIND_FRAME_LEN);
    // 4. 发送查询帧 - 优化超时时间
    status = HAL_UART_Transmit(&huart2, query_frame, 8, 100);  // 增加到100ms超时
    
    HAL_Delay(100);

    if (status != HAL_OK) {
        DEBUG_LOG("查询帧发送失败，错误码: %d\r\n", status);    
        return HAL_ERROR;
    }
    
    DEBUG_LOG("查询帧发送成功，等待响应...\r\n");
    return HAL_OK;
}

// ************************** 检查接收状态函数 **************************
HAL_StatusTypeDef WindSpeed_CheckReceive(void)
{
    if (g_rx_complete)
    {
        DEBUG_LOG("接收到完整风速帧: ");
        for (int i = 0; i < g_rx_len; i++) {
            DEBUG_LOG("%02X ", g_rx_buf[i]);
        }
        DEBUG_LOG("\r\n");
        
        return WindSpeed_ParseFrame();
    }
    
    return HAL_BUSY;  // 还未接收完成
}

// ************************** 获取风速数据 **************************
WindSpeed_DataDef WindSpeed_GetData(void)
{
    WindSpeed_DataDef temp_data = g_wind_data;
    g_wind_data.is_valid = 0;  // 读取后清除有效标志
    return temp_data;
}

// ************************** HAL回调函数 **************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // 接收完成，设置标志
        printf("wind uart rx ok");
			  g_rx_complete = 1;
        g_rx_len = WIND_FRAME_LEN;
        // WindSpeed_ParseFrame();
        // HAL_UART_Receive_IT(&huart2, g_rx_buf, WIND_FRAME_LEN);

        // printf("\r\n=== 接收数据帧 ===\r\n");
        // printf("接收长度: %d 字节\r\n", g_rx_len);
        // printf("数据内容: ");
        // for (int i = 0; i < g_rx_len; i++) {
        //      printf("%02X ", g_rx_buf[i]);
        // }
        // printf("\r\n");
        // WindSpeed_ParseFrame();
        // if (g_rx_len != WIND_FRAME_LEN) {
        //     DEBUG_LOG("帧长度错误: 期望%d，实际%d\r\n", WIND_FRAME_LEN, g_rx_len);
        // }

        // // 2. 校验地址和功能码
        // if (g_rx_buf[0] != WIND_MODBUS_ADDR || g_rx_buf[1] != WIND_MODBUS_FUNC) {
        //     DEBUG_LOG("地址/功能码错误: 地址0x%02X，功能码0x%02X\r\n", g_rx_buf[0], g_rx_buf[1]);
        // }

        // // 3. 校验CRC16，帧最后2字节为CRC
        // crc_recv = (g_rx_buf[8] << 8) | g_rx_buf[7];  // 接收到的CRC（低字节在前）
        // crc_calc = Modbus_CRC16(g_rx_buf, WIND_FRAME_LEN - 2);  // 计算CRC
        // if (crc_calc != crc_recv) {
        //     DEBUG_LOG("CRC校验失败: 计算0x%04X，接收0x%04X\r\n", crc_calc, crc_recv);
        // }

        // // 4. 解析数据：0x0024=36=3.6m/s，0x0003=3级
        // uint16_t speed_raw = (g_rx_buf[3] << 8) | g_rx_buf[4];
        // g_wind_data.wind_speed = speed_raw / 10.0f;  // 单位为0.1m/s 转 m/s
        // g_wind_data.wind_grade = (g_rx_buf[5] << 8) | g_rx_buf[6];
        // g_wind_data.is_valid = 1;
        // g_wind_data.err_cnt = 0;

        // // 打印解析结果
        // printf("风速解析成功: 速度%.1f m/s，风级%d\r\n",
        //     g_wind_data.wind_speed, g_wind_data.wind_grade);

        // if (parse_result == HAL_OK)
        //      {
        //         WindSpeed_DataDef data = WindSpeed_GetData();
        //          printf("获取到风速数据: %.1f m/s, %d 级\r\n", 
        //                        data.wind_speed, data.wind_grade);
        //      }
        // else
        //      {
        //          printf("风速数据解析失败\r\n");
        //      }
        // 回显接收的数据（可选）
        // HAL_UART_Transmit(&huart2, g_rx_buf, g_rx_len, 100);
    
    }
}



