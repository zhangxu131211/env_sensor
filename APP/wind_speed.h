#ifndef __WIND_SPEED_H
#define __WIND_SPEED_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

// ************************** 宏定义 **************************
#define WIND_MODBUS_ADDR    0x01    // 设备地址
#define WIND_MODBUS_FUNC    0x03    // 功能码：读保持寄存器
#define WIND_REG_START      0x0000  // 起始寄存器地址
#define WIND_REG_LEN        0x0002  // 读取寄存器数量
#define WIND_FRAME_LEN      9       // 响应帧长度（固定9字节）
#define WIND_BAUDRATE       4800    // 波特率

// ************************** 数据结构 **************************
typedef struct {
    float wind_speed;   // 风速 (m/s)
    uint16_t wind_grade; // 风级
    uint8_t is_valid;   // 数据有效性标志
    uint8_t err_cnt;    // 错误计数
} WindSpeed_DataDef;

extern uint8_t g_rx_complete;
extern uint8_t g_rx_buf[32];
extern uint8_t g_rx_len;

// ************************** 函数声明 **************************
void WindSpeed_Init(void);
HAL_StatusTypeDef WindSpeed_Query(void);
WindSpeed_DataDef WindSpeed_GetData(void);
HAL_StatusTypeDef WindSpeed_CheckReceive(void);
HAL_StatusTypeDef WindSpeed_ParseFrame(void);

#endif /* __WIND_SPEED_H */

