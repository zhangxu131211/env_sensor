#include "wind_speed.h"
#include "usart.h"  // 包含串口2句柄huart2

// 全局变量（静态，仅本文件可见）
static WindSpeed_DataDef g_wind_data = {0};
static uint8_t g_rx_buf[32] = {0};  // 接收缓冲区
static uint8_t g_rx_len = 0;        // 接收长度

/**
 * @brief  CRC16-Modbus校验（传感器帧必备，按说明书协议）
 * @param  buf: 数据缓冲区
 * @param  len: 数据长度
 * @retval CRC16值（低字节在前）
 */
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

/**
 * @brief  切换串口2波特率（调试115200 ? 传感器4800）
 * @param  baudrate: 目标波特率
 * @retval HAL状态
 */
static HAL_StatusTypeDef UART2_SetBaudrate(uint32_t baudrate) {
    huart2.Init.BaudRate = baudrate;
    if (HAL_UART_DeInit(&huart2) != HAL_OK) {
        WIND_LOG("串口波特率切换失败（DeInit）");
        return HAL_ERROR;
    }
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        WIND_LOG("串口波特率切换失败（Init）");
        return HAL_ERROR;
    }
    WIND_LOG("串口波特率切换为：%d", baudrate);
    return HAL_OK;
}

/**
 * @brief  串口2接收中断回调（用于接收传感器返回帧）
 * @param  huart: 串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && g_rx_len < sizeof(g_rx_buf) - 1) {
        g_rx_buf[g_rx_len++] = huart->pRxBuffPtr[0];  // 缓存接收数据
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart2, &g_rx_buf[g_rx_len], 1);
    }
}

/**
 * @brief  风速传感器初始化（初始化缓冲区和参数）
 * @retval 无
 */
void WindSpeed_Init(void) {
    memset(&g_wind_data, 0, sizeof(WindSpeed_DataDef));
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    g_rx_len = 0;

    // 启动串口2中断接收（用于接收传感器数据）
    HAL_UART_Receive_IT(&huart2, g_rx_buf, 1);
    WIND_LOG("风速传感器驱动初始化完成");
}

/**
 * @brief  解析传感器返回帧（按说明书7.2节）
 * @retval HAL状态（HAL_OK=解析成功）
 */
static HAL_StatusTypeDef WindSpeed_ParseFrame(void) {
    uint16_t crc_calc, crc_recv;

    // 1. 校验帧长度（传感器返回帧固定9字节）
    if (g_rx_len != WIND_FRAME_LEN) {
        WIND_LOG("帧长度错误：期望%d，实际%d", WIND_FRAME_LEN, g_rx_len);
        return HAL_ERROR;
    }

    // 2. 校验地址和功能码
    if (g_rx_buf[0] != WIND_MODBUS_ADDR || g_rx_buf[1] != WIND_MODBUS_FUNC) {
        WIND_LOG("地址/功能码错误：地址0x%02X，功能码0x%02X", g_rx_buf[0], g_rx_buf[1]);
        return HAL_ERROR;
    }

    // 3. 校验CRC16（传感器帧最后2字节为CRC）
    crc_recv = (g_rx_buf[8] << 8) | g_rx_buf[7];  // 接收的CRC（低字节在前）
    crc_calc = Modbus_CRC16(g_rx_buf, WIND_FRAME_LEN - 2);  // 计算CRC
    if (crc_calc != crc_recv) {
        WIND_LOG("CRC校验失败：计算0x%04X，接收0x%04X", crc_calc, crc_recv);
        return HAL_ERROR;
    }

    // 4. 解析数据（说明书示例：0x0024=36→3.6m/s，0x0003=3级）
    uint16_t speed_raw = (g_rx_buf[3] << 8) | g_rx_buf[4];
    g_wind_data.wind_speed = speed_raw / 10.0f;  // 单位：0.1m/s → m/s
    g_wind_data.wind_grade = (g_rx_buf[5] << 8) | g_rx_buf[6];
    g_wind_data.is_valid = 1;
    g_wind_data.err_cnt = 0;

    // 打印解析结果（调试用）
    WIND_LOG("解析成功：风速%.1f m/s，风级%d级",
             g_wind_data.wind_speed, g_wind_data.wind_grade);

    return HAL_OK;
}

/**
 * @brief  查询风速数据（核心函数，自动切换波特率）
 * @retval HAL状态（HAL_OK=查询成功）
 */
HAL_StatusTypeDef WindSpeed_Query(void) {
    uint8_t query_frame[8] = {0};
    uint16_t crc;
    HAL_StatusTypeDef status = HAL_OK;

    // 1. 保存当前调试波特率，切换为传感器波特率（4800）
    uint32_t temp_baud = huart2.Init.BaudRate;
    if (UART2_SetBaudrate(WIND_BAUDRATE) != HAL_OK) {
        g_wind_data.err_cnt++;
        return HAL_ERROR;
    }

    // 2. 组装查询帧（说明书7.2节标准指令：01 03 00 00 00 02 C4 0B）
    query_frame[0] = WIND_MODBUS_ADDR;          // 地址
    query_frame[1] = WIND_MODBUS_FUNC;          // 功能码
    query_frame[2] = (WIND_REG_START >> 8);     // 起始寄存器高字节
    query_frame[3] = (WIND_REG_START & 0xFF);   // 起始寄存器低字节
    query_frame[4] = (WIND_REG_LEN >> 8);       // 读取长度高字节
    query_frame[5] = (WIND_REG_LEN & 0xFF);     // 读取长度低字节
    crc = Modbus_CRC16(query_frame, 6);
    query_frame[6] = (crc & 0xFF);              // CRC低字节
    query_frame[7] = (crc >> 8);                // CRC高字节

    // 3. 发送查询帧
    WIND_LOG("发送查询帧：0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
             query_frame[0], query_frame[1], query_frame[2], query_frame[3],
             query_frame[4], query_frame[5], query_frame[6], query_frame[7]);
    if (HAL_UART_Transmit(&huart2, query_frame, 8, 100) != HAL_OK) {
        WIND_LOG("查询帧发送失败");
        status = HAL_ERROR;
        goto RESTORE_BAUDRATE;  // 发送失败也要恢复波特率
    }

    // 4. 等待接收传感器返回帧（超时1秒）
    HAL_Delay(100);  // 传感器响应时间<1秒（说明书4节）
    if (g_rx_len >= WIND_FRAME_LEN) {
        // 解析接收帧
        if (WindSpeed_ParseFrame() != HAL_OK) {
            status = HAL_ERROR;
        }
    } else {
        WIND_LOG("未收到返回帧，接收长度：%d", g_rx_len);
        status = HAL_ERROR;
    }

RESTORE_BAUDRATE:
    // 5. 恢复调试波特率（115200）
    if (UART2_SetBaudrate(temp_baud) != HAL_OK) {
        WIND_LOG("波特率恢复失败！");
        status = HAL_ERROR;
    }

    // 6. 重置接收缓冲区，准备下一次查询
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    g_rx_len = 0;
    HAL_UART_Receive_IT(&huart2, g_rx_buf, 1);  // 重启中断接收

    // 7. 错误计数更新
    if (status != HAL_OK) {
        g_wind_data.err_cnt++;
        WIND_LOG("查询失败，累计错误次数：%d", g_wind_data.err_cnt);
    }

    return status;
}

/**
 * @brief  获取解析后的风速数据
 * @retval 风速数据结构体（含风速、风级、有效性标记）
 */
WindSpeed_DataDef WindSpeed_GetData(void) {
    WindSpeed_DataDef temp_data = g_wind_data;
    g_wind_data.is_valid = 0;  // 读取后重置有效标记
    return temp_data;
}


