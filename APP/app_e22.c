#include "app_e22.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>   // 必须包含：处理可变参数


/**
 * @brief  E22模块初始化
 * @note   封装底层初始化接口，统一初始化入口
 */
void App_E22_Init(void)
{
    // 初始化E22缓冲区和回调函数
    E22_Init();
    // 启动E22 DMA接收（如需接收数据则必须调用）
    E22_StartReceive();
    // 切换到传输模式（默认发送前确保模式正确）
    E22_Mode(E22_Mode_Transmission);
}

/**
 * @brief  发送测试数据（AA BB CC）到指定地址和信道
 * @param  address: 目标地址（示例传3）
 * @param  channel: 目标信道（示例传4）
 * @retval 无
 */
void App_E22_Send_Test_Data(uint16_t address, uint8_t channel)
{
    // 1. 确保E22处于传输模式（双重保障）
   // E22_Mode(E22_Mode_Transmission);

    // 2. 初始化发送数据包结构体
    e22_uart_packet_struct_t send_packet = {0};
    
    // 填充测试数据：0xAA 0xBB 0xCC
    send_packet.buffer[0] = 0xAA;
    send_packet.buffer[1] = 0xBB;
    send_packet.buffer[2] = 0xCC;
    
    // 设置数据长度（3个字节）
    send_packet.length = 3;

		
    // 3. 调用底层E22_Send发送数据
    E22_Send(address, channel, &send_packet);

	  // 4. 串口2打印调试信息
    uint8_t debug_buf[64] = {0};
    uint16_t debug_len = sprintf((char*)debug_buf, 
                                 "E22发送测试数据: 地址=0x%04X, 信道=%x, 数据=%x %x %x\r\n", 
                                 address, channel, send_packet.buffer[0], send_packet.buffer[1], send_packet.buffer[2]);
    HAL_UART_Transmit(&huart2, debug_buf, debug_len, 100);
    // （可选）等待DMA发送完成（如需同步操作，取消注释）
    // while (HAL_UART_GetState(&E22_UART_HANDLE) == HAL_UART_STATE_BUSY_TX)
    // {
    //     // 等待发送完成，避免数据覆盖
    // }
}

/**
 * @brief  （可选）E22接收数据处理函数
 * @note   如需处理接收的数据，可调用此函数
 * @retval 无
 */
void App_E22_Handle_Receive_Data(void)
{
    uint8_t recv_buf[E22_UART_BUFFER_MAX_LENGTH] = {0};
    uint8_t recv_len = 0;

    // 检查接收缓冲区是否有数据
    if (!E22_UART_Buffer_isEmpty())
    {
        // 取出接收的数据包
        recv_len = E22_UART_Packet_Pop(recv_buf);
        
        // 此处可添加数据处理逻辑（示例：打印/解析数据）
        HAL_UART_Transmit(&huart2, recv_buf, recv_len, 100); // 串口1打印接收数据
    }
}



// 声明外部句柄（需和工程中一致）
//extern UART_HandleTypeDef huart2;          // 调试用串口2
//extern UART_HandleTypeDef E22_UART_HANDLE; // E22模块的UART句柄



/**
 * @brief  通用E22发送函数（类似printf，支持可变参数）
 * @param  address: 目标地址（如3）
 * @param  channel: 目标信道（如4）
 * @param  format:  格式控制符（支持 %x(十六进制字节)、%d(十进制)、%c(字符)、%s(字符串)）
 * @param  ...:     可变参数（要发送的具体数据）
 * @retval uint8_t: 0=成功，1=缓冲区溢出，2=参数错误
 * @example: 
 *					App_E22_Send_Data(E22_RX_ADDR, E22_RX_CHANNEL, "%x%x%x", 0xAA, 0xBB, 0xCC);	// 发送 AA BB CC（无空格）
 *          App_E22_Send_Data(3, 4, "%x %x %x", 0xAA, 0xBB, 0xCC); // 发送 AA 20 BB 20 CC（带空格）
 *          App_E22_Send_Data(5, 8, "%s", "Hello E22");            // 发送字符串
 *          App_E22_Send_Data(10, 2, "%c%d%x", 'A', 123, 0xFF);    // 发送A 123 FF
 */
uint8_t App_E22_Send_Data(uint16_t address, uint8_t channel, const char *format, ...)
{
    // 1. 入参校验
    if (format == NULL || address > 0xFFFF || channel > 0xFF) {
        return 2; // 参数错误
    }

    // 2. 确保E22处于传输模式
    E22_Mode(E22_Mode_Transmission);

    // 3. 处理可变参数，填充发送缓冲区
    va_list args;
    va_start(args, format);

    uint8_t send_buf[E22_SEND_BUF_MAX_LEN] = {0}; // E22发送缓冲区
    uint16_t send_len = 0;                        // 实际要发送的字节数
    char *fmt_ptr = (char*)format;                // 格式符解析指针

    // 解析格式符，填充发送缓冲区
    while (*fmt_ptr != '\0' && send_len < E22_SEND_BUF_MAX_LEN) {
        if (*fmt_ptr == '%') {
            fmt_ptr++; // 跳过%，解析后续格式符
            switch (*fmt_ptr) {
                case 'x': // 十六进制字节（单字节）
                    send_buf[send_len++] = (uint8_t)va_arg(args, int); // 按字节取
                    break;
                case 'd': // 十进制数（转为ASCII，这里简化为单字节数值）
                    send_buf[send_len++] = (uint8_t)va_arg(args, int);
                    break;
                case 'c': // 字符（单字节）
                    send_buf[send_len++] = (uint8_t)va_arg(args, int); // char会提升为int
                    break;
                case 's': // 字符串（多字节）
                {
                    char *str = va_arg(args, char*);
                    uint16_t str_len = strlen(str);
                    // 防止缓冲区溢出
                    if (send_len + str_len > E22_SEND_BUF_MAX_LEN) {
                        str_len = E22_SEND_BUF_MAX_LEN - send_len;
                    }
                    memcpy(&send_buf[send_len], str, str_len);
                    send_len += str_len;
                    break;
                }
                default: // 未知格式符，直接作为普通字节发送
                    send_buf[send_len++] = *fmt_ptr;
                    break;
            }
        } else {
            // 非格式符，直接作为普通字节发送（比如空格、逗号等）
            send_buf[send_len++] = *fmt_ptr;
        }
        fmt_ptr++;
    }
    va_end(args);

    // 4. 检查缓冲区是否溢出
    if (send_len >= E22_SEND_BUF_MAX_LEN) {
        return 1; // 缓冲区溢出
    }

    // 5. 封装E22数据包结构体
    e22_uart_packet_struct_t send_packet = {0};
    memcpy(send_packet.buffer, send_buf, send_len);
    send_packet.length = send_len;

    // 6. 调用底层E22_Send发送数据
    E22_Send(address, channel, &send_packet);

#if 0		
    // 7. 串口2调试打印（可变参数同步打印）
    char debug_buf[128] = {0};
    // 先打印基础信息
    uint16_t debug_len = sprintf(debug_buf, "E22发送: 地址=0x%04X, 信道=%d, 数据=", address, channel);
    // 再拼接发送的实际数据（十六进制格式）
    for (uint16_t i = 0; i < send_len; i++) {
        debug_len += sprintf(debug_buf + debug_len, "%02X ", send_buf[i]);
    }
    debug_len += sprintf(debug_buf + debug_len, "\r\n");
    // 串口2发送调试信息
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_buf, debug_len, 100);

    // 8. （可选）等待DMA发送完成，避免数据覆盖
    while (HAL_UART_GetState(&E22_UART_HANDLE) == HAL_UART_STATE_BUSY_TX);
#endif
    return 0; // 发送成功
}

/**
 * @brief  E22协议数组发送函数（直接发送字节数组，无空格、无格式解析）
 * @param  address: 目标地址（如3）
 * @param  channel: 目标信道（如4）
 * @param  proto_array: 待发送的协议字节数组（如组装好的温湿度/经纬度数组）
 * @param  array_len: 协议数组的有效长度
 * @retval uint8_t: 0=成功，1=缓冲区溢出，2=参数错误
 * @example: 
 *          uint8_t proto_data[] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0xFF}; // 示例协议数组
 *          App_E22_Send_Protocol_Array(3, 4, proto_data, sizeof(proto_data)); // 发送完整数组（无空格）
 *          App_E22_Send_Protocol_Array(E22_RX_ADDR, E22_RX_CHANNEL, packet.buf, packet.len); // 发送协议包数组
 */
uint8_t App_E22_Send_Protocol_Array(uint16_t address, uint8_t channel, const uint8_t *proto_array, uint16_t array_len)
{
    // 1. 入参严格校验
    if (proto_array == NULL ||        // 数组指针为空
        address > 0xFFFF ||          // 地址超出16位范围
        channel > 0xFF ||            // 信道超出8位范围
        array_len == 0 ||            // 数组长度为0
        array_len > E22_SEND_BUF_MAX_LEN) { // 数组长度超出缓冲区上限
        return 2; // 参数错误
    }

    // 2. 确保E22处于传输模式（和原有逻辑一致）
    E22_Mode(E22_Mode_Transmission);

    // 3. 直接拷贝协议数组到发送缓冲区（无空格、无格式转换）
    uint8_t send_buf[E22_SEND_BUF_MAX_LEN] = {0};
    uint16_t send_len = array_len;
    memcpy(send_buf, proto_array, send_len); // 原样拷贝，无任何额外字符

    // 4. 检查缓冲区溢出（双重保障）
    if (send_len >= E22_SEND_BUF_MAX_LEN) {
        return 1; // 缓冲区溢出
    }

    // 5. 封装E22数据包结构体（和原有逻辑一致）
    e22_uart_packet_struct_t send_packet = {0};
    memcpy(send_packet.buffer, send_buf, send_len);
    send_packet.length = send_len;

    // 6. 调用底层E22_Send发送数据（核心发送逻辑）
    E22_Send(address, channel, &send_packet);

#if 0 // 调试打印开关，开启后串口2打印发送信息
    // 7. 串口2调试打印（仅打印数组内容，无多余空格）
    char debug_buf[128] = {0};
    uint16_t debug_len = sprintf(debug_buf, "E22协议发送: 地址=0x%04X, 信道=%d, 长度=%d, 数据=", address, channel, send_len);
    // 拼接十六进制数据（无空格）
    for (uint16_t i = 0; i < send_len; i++) {
        debug_len += sprintf(debug_buf + debug_len, "%02X", send_buf[i]);
    }
    debug_len += sprintf(debug_buf + debug_len, "\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_buf, debug_len, 100);

    // 8. （可选）等待DMA发送完成，避免数据覆盖
    while (HAL_UART_GetState(&E22_UART_HANDLE) == HAL_UART_STATE_BUSY_TX);
#endif
    return 0; // 发送成功
}
