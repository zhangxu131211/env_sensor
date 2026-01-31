#ifndef __GY95T_H
#define __GY95T_H

#include "main.h"
#include "i2c.h"
#include <string.h>
#include <stdio.h>
#include <math.h>  // 用于四元数归一化

/* GY95T 核心配置（严格遵循手册） */
#define GY95T_I2C_ADDR_7BIT  0x52        // 手册默认I2C地址（7位，0xA4>>1）
#define GY95T_I2C_ADDR       (GY95T_I2C_ADDR_7BIT << 1)  // HAL库要求左移1位（含读写位）
#define GY95T_REG_START      0x08        // 数据读取起始寄存器（ACC_X_L）
#define GY95T_DATA_LEN       35          // 0x08~0x2A共35字节（0x2A-0x08+1=35）
#define GY95T_QUAT_REG_START 0x23        // 四元数起始寄存器（Q0_L）
#define GY95T_QUAT_DATA_LEN   8           // 四元数总字节数（Q0~Q3各2字节）

/* 传感器量程配置（默认值，手册0x07寄存器定义） */
#define ACC_FS_DEFAULT       3           // 默认±16G（0=±2G,1=±4G,2=±8G,3=±16G）
#define GYRO_FS_DEFAULT      3           // 默认±2000°/s（0=±250,1=±500,2=±1000,3=±2000）

/* 量程对应的LSB换算系数（手册数据） */
#define ACC_LSB_2G           16384.0f    // ±2G → 16384 LSB/G
#define ACC_LSB_4G           8192.0f     // ±4G → 8192 LSB/G
#define ACC_LSB_8G           4096.0f     // ±8G → 4096 LSB/G
#define ACC_LSB_16G          2048.0f     // ±16G → 2048 LSB/G
#define GYRO_LSB_250DPS      131.0f      // ±250°/s → 131 LSB/(°/s)
#define GYRO_LSB_500DPS      65.5f       // ±500°/s → 65.5 LSB/(°/s)
#define GYRO_LSB_1000DPS     32.8f       // ±1000°/s → 32.8 LSB/(°/s)
#define GYRO_LSB_2000DPS     16.4f       // ±2000°/s → 16.4 LSB/(°/s)

// 常用配置宏定义（可直接调用）
#define CAL_METHOD_DEFAULT    0x03  // 默认：软硬铁椭球校准
#define CAL_METHOD_3D         0x01  // 3D极限值校准
#define CAL_METHOD_2D         0x00  // 2D极限值校准
#define CAL_AUTO_ON           0x20  // 开启磁场自动校准
#define CAL_ANTI_INTERFERENCE 0x10  // 开启抗干扰校准

// 常用量程宏定义
#define ACC_FS_2G     0
#define ACC_FS_4G     1
#define ACC_FS_8G     2
#define ACC_FS_16G    3

#define GYRO_FS_250   0
#define GYRO_FS_500   1
#define GYRO_FS_1000  2
#define GYRO_FS_2000  3

#define MAG_FS_2G     0
#define MAG_FS_8G     1
#define MAG_FS_12G    2
#define MAG_FS_30G    3

#define WORK_MODE_HORIZONTAL  1
#define WORK_MODE_VERTICAL    0

/* GY95T 完整数据结构体（含所有传感器数据） */
__packed typedef struct
{
    // 1. 加速计数据（0x08~0x0D）
    int16_t acc_x_raw;   // X轴原始值（-32768~32767）
    int16_t acc_y_raw;   // Y轴原始值
    int16_t acc_z_raw;   // Z轴原始值
    float   acc_x_g;     // X轴实际值（单位：g）
    float   acc_y_g;     // Y轴实际值（单位：g）
    float   acc_z_g;     // Z轴实际值（单位：g）
    
    // 2. 陀螺仪数据（0x0E~0x13）
    int16_t gyro_x_raw;  // X轴原始值（-32768~32767）
    int16_t gyro_y_raw;  // Y轴原始值
    int16_t gyro_z_raw;  // Z轴原始值
    float   gyro_x_dps;  // X轴实际值（单位：°/s）
    float   gyro_y_dps;  // Y轴实际值（单位：°/s）
    float   gyro_z_dps;  // Z轴实际值（单位：°/s）
    
    // 3. 姿态角数据（0x14~0x19）
    int16_t roll_raw;    // 横滚角原始值 → 换算后0.01°
    int16_t pitch_raw;   // 俯仰角原始值 → 换算后0.01°
    int16_t yaw_raw;     // 航向角原始值 → 换算后0.01°
    float   roll_deg;    // 横滚角实际值（单位：°）
    float   pitch_deg;   // 俯仰角实际值（单位：°）
    float   yaw_deg;     // 航向角实际值（单位：°）
    
    // 4. 校准与温度（0x1A~0x1C）
    uint8_t level;       // 磁场校准精度（1~255，100最佳）
    int16_t temp_raw;    // 温度原始值 → 换算后0.01°C
    float   temp_c;      // 温度实际值（单位：°C）
    
    // 5. 磁场数据（0x1D~0x22）
    int16_t mag_x_raw;   // X轴原始值（-32768~32767）
    int16_t mag_y_raw;   // Y轴原始值
    int16_t mag_z_raw;   // Z轴原始值
    
    // 6. 四元数数据（0x23~0x2A）
    float   q0;          // 四元数Q0（-1.0~1.0）
    float   q1;          // 四元数Q1（-1.0~1.0）
    float   q2;          // 四元数Q2（-1.0~1.0）
    float   q3;          // 四元数Q3（-1.0~1.0）
		
		//7.标志位
		uint8_t update_flag;
} GY95T_DataDef;

extern GY95T_DataDef gy95t_data;

/* 函数声明 */
HAL_StatusTypeDef GY95T_Init(uint8_t freq_level);       // 初始化（配置更新频率）
HAL_StatusTypeDef GY95T_ReadData(GY95T_DataDef *gy_data); // 读取完整数据
HAL_StatusTypeDef GY95T_WriteReg(uint8_t reg_addr, uint8_t data); // I2C写寄存器
void GY95T_ConvertData(GY95T_DataDef *gy_data);        // 原始数据换算为实际物理值
HAL_StatusTypeDef GY95T_Calibrate(uint8_t cal_type);    // 校准功能（加陀/磁场）
HAL_StatusTypeDef GY95T_AutoCalibrate(void);            // 自动校准流程（初始化时调用）
void GY95T_NormalizeQuaternion(GY95T_DataDef *gy_data); // 四元数归一化
HAL_StatusTypeDef GY95T_SetRange(uint8_t acc_fs, uint8_t gyro_fs, uint8_t mag_fs, uint8_t work_mode);


#endif /* __GY95T_H */

