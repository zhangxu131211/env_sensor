#ifndef __APP_E22_H
#define __APP_E22_H

#include "e22.h"
#include "stm32l4xx_hal.h"  // 根据实际STM32型号调整（如stm32f1xx_hal.h/stm32f4xx_hal.h）

//无人机发射方
#define E22_TX_ADDR						1
#define E22_TX_CHANNEL				2

//地面接收方
#define E22_RX_ADDR					3
#define E22_RX_CHANNEL				4

// 定义E22发送缓冲区最大长度（根据模块手册，建议≤128字节）
#define E22_SEND_BUF_MAX_LEN  256

/**
 * @brief  E22模块初始化（封装底层E22_Init/E22_StartReceive）
 * @note   需在系统初始化后调用
 */
void App_E22_Init(void);

/**
 * @brief  发送测试数据（AA BB CC）到指定地址和信道
 * @param  address: 目标地址（uint16_t）
 * @param  channel: 目标信道（uint8_t）
 * @retval 无
 */
void App_E22_Send_Test_Data(uint16_t address, uint8_t channel);

/**
 * @brief  通用E22发送函数（类似printf，支持可变参数）
 * @param  address: 目标地址（如3）
 * @param  channel: 目标信道（如4）
 * @param  format:  格式控制符（支持 %x(十六进制字节)、%d(十进制)、%c(字符)、%s(字符串)）
 * @param  ...:     可变参数（要发送的具体数据）
 * @retval uint8_t: 0=成功，1=缓冲区溢出，2=参数错误
 * @example: 
 *          App_E22_Send_Data(3, 4, "%x %x %x", 0xAA, 0xBB, 0xCC); // 发送AA BB CC
 *          App_E22_Send_Data(5, 8, "%s", "Hello E22");            // 发送字符串
 *          App_E22_Send_Data(10, 2, "%c%d%x", 'A', 123, 0xFF);    // 发送A 123 FF
 */
uint8_t App_E22_Send_Data(uint16_t address, uint8_t channel, const char *format, ...);

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
uint8_t App_E22_Send_Protocol_Array(uint16_t address, uint8_t channel, const uint8_t *proto_array, uint16_t array_len);



#endif // __APP_E22_H

