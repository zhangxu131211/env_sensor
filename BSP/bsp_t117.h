#ifndef INC_BSP_T117_H_
#define INC_BSP_T117_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include <math.h>
#include <string.h>

/* ========================== 核心配置（根据硬件调整）========================== */
// I2C地址（ADDR引脚连接对应，默认VDD=0x41，需与硬件一致）
#define T117_I2C_ADDR_GND    0x40    // ADDR接GND
#define T117_I2C_ADDR_VDD    0x41    // ADDR接VDD（默认配置）
#define T117_I2C_ADDR_SDA    0x42    // ADDR接SDA
#define T117_I2C_ADDR_SCL    0x43    // ADDR接SCL

// 实际使用的I2C地址（写：左移1位，读：左移1位+1）
#define T117_I2C_W_ADDR      (T117_I2C_ADDR_GND << 1)  // 写地址：0x82
#define T117_I2C_R_ADDR      (T117_I2C_ADDR_GND << 1 | 0x01)  // 读地址：0x83

/* ========================== 寄存器地址定义（T117P手册10.1节）========================== */
#define T117_TEMP_LSB        0x00    // 温度低位寄存器
#define T117_TEMP_MSB        0x01    // 温度高位寄存器
#define T117_CRC_TEMP        0x02    // 温度数据CRC校验寄存器
#define T117_STATUS          0x03    // 状态寄存器（读）
#define T117_TEMP_CMD        0x04    // 测温指令寄存器（写）
#define T117_TEMP_CFG        0x05    // 配置寄存器（读/写）
#define T117_ALERT_MODE      0x06    // 报警模式寄存器（读/写）
#define T117_EE_CMD          0x17    // E2PROM指令寄存器（写）

/* ========================== 分辨率定义（T117P手册9.1节）========================== */
#define T117_RESOLUTION      (1.0f / 256.0f)  // 0.00390625℃/LSB（16位分辨率）

/* ========================== 枚举类型定义（T117P手册9.2/9.3节）========================== */
// 测量模式（Temp_Cmd寄存器bit7-6）
typedef enum {
    T117_CONTI_CONVERT = 0x00,  // 连续测量（回读0x00）
    T117_STOP_CONVERT  = 0x40,  // 停止测量（默认）
    T117_SINGLE_CONVERT = 0xC0  // 单次测量
} T117_CONV_MODE;

// 加热模式（Temp_Cmd寄存器bit3-0）
typedef enum {
    T117_HEAT_OFF = 0x00,  // 加热关闭（默认）
    T117_HEAT_ON  = 0x0A   // 加热开启（0b1010）
} T117_HEAT_MODE;

// 平均次数（Temp_Cfg寄存器bit4-3）
typedef enum {
    T117_AVG_1   = 0x00,  // 平均1次，转换时间2.2ms
    T117_AVG_8   = 0x08,  // 平均8次，转换时间5.2ms（默认）
    T117_AVG_16  = 0x10,  // 平均16次，转换时间8.5ms
    T117_AVG_32  = 0x18   // 平均32次，转换时间15.3ms
} T117_AVG_MODE;

// 测量频率（Temp_Cfg寄存器bit7-5）
typedef enum {
    T117_FREQ_8HZ  = 0x00,  // 每秒8次
    T117_FREQ_4HZ  = 0x20,  // 每秒4次
    T117_FREQ_2HZ  = 0x40,  // 每秒2次
    T117_FREQ_1HZ  = 0x60,  // 每秒1次（默认）
    T117_FREQ_2S   = 0x80,  // 每2秒1次
    T117_FREQ_4S   = 0xA0,  // 每4秒1次
    T117_FREQ_8S   = 0xC0,  // 每8秒1次
    T117_FREQ_16S  = 0xE0   // 每16秒1次
} T117_MEAS_FREQ;

// 低功耗模式（Temp_Cfg寄存器bit0）
typedef enum {
    T117_PD_OFF = 0x00,  // 不进入低功耗
    T117_PD_ON  = 0x01   // 执行指令后进入低功耗（默认）
} T117_PD_MODE;

/* ========================== 函数声明 ========================== */
// 初始化T117P（配置测量模式、平均次数、频率等）
bool BSP_T117_Init(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer);

// 软件复位（装载E2PROM数据到寄存器）
bool BSP_T117_SoftReset(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer);

// 设置测量模式（连续/单次/停止）
bool BSP_T117_SetConvMode(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_CONV_MODE mode);

// 设置平均次数（影响精度和转换时间）
bool BSP_T117_SetAveraging(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_AVG_MODE avg);

// 设置测量频率（连续模式下有效）
bool BSP_T117_SetMeasFreq(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_MEAS_FREQ freq);

// 设置低功耗模式
bool BSP_T117_SetPDMode(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, T117_PD_MODE pd);

// 启动单次测温（仅单次测量模式有效）
bool BSP_T117_StartSingleConv(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer);

// 读取温度值（含CRC校验）
bool BSP_T117_GetTemperature(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, float* pTemp);

// 读取状态寄存器
bool BSP_T117_GetStatus(I2C_HandleTypeDef* hi2c, uint8_t* pBuffer, uint8_t* pStatus);

// CRC8校验函数（多项式：X?+X?+X?+1，初始值0x00）
static uint8_t BSP_T117_CRC8(uint8_t* pData, uint8_t len);

#endif /* INC_BSP_T117_H_ */
