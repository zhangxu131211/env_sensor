#ifndef __WIND_SPEED_H
#define __WIND_SPEED_H

#include "stm32l4xx_hal.h"
#include <string.h>

// -------------------------- 调试模式开关（与你的全局宏一致）--------------------------
#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#include "stdio.h"
// 复用你的调试日志宏（带文件、函数、行号）
#define WIND_LOG(fmt, ...)  printf("[%s | %s:%d] [风速传感器] " fmt "\n", \
                                  __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define WIND_LOG(fmt, ...)
#endif

// -------------------------- 风速传感器参数（严格按说明书）--------------------------
#define WIND_MODBUS_ADDR       0x01        // 传感器默认地址
#define WIND_MODBUS_FUNC       0x03        // 读取寄存器功能码
#define WIND_REG_START         0x0000      // 风速起始寄存器
#define WIND_REG_LEN           0x0002      // 读取2个寄存器（风速+风级）
#define WIND_BAUDRATE          4800        // 传感器固定波特率（说明书7.2节）
#define WIND_FRAME_LEN         9           // 传感器返回帧长度（9字节）
#define WIND_QUERY_INTERVAL    2000        // 查询间隔（2000ms）
#define DEBUG_BAUDRATE         115200      // 调试串口默认波特率（与usart.c一致）

// -------------------------- 数据结构体 --------------------------
typedef struct {
    float wind_speed;  // 风速值（m/s）
    uint8_t wind_grade; // 风级（0-17级）
    uint8_t is_valid;  // 数据有效标记（1=有效）
    uint8_t err_cnt;   // 错误计数（调试用）
} WindSpeed_DataDef;

// -------------------------- 函数声明 --------------------------
void WindSpeed_Init(void);                  // 风速传感器初始化（初始化缓冲区和参数）
HAL_StatusTypeDef WindSpeed_Query(void);    // 查询风速数据（自动切换波特率）
WindSpeed_DataDef WindSpeed_GetData(void);  // 获取解析后的风速数据

#endif


