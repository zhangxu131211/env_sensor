/**
  ******************************************************************************
  * @file    sht40ad1b_app.h
  * @author  zhangxu
  * @brief   SHT40AD1B应用层头文件
  ******************************************************************************
  */

#ifndef __APP_SHT40_H
#define __APP_SHT40_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含寄存器层头文件 */
#include "sht40ad1b_reg.h"


/* 宏定义 -------------------------------------------------------------------*/
#define sht40_i2c hi2c2

#define SHT40AD1B_I2C_ADDR  0x8A  // SHT40 I2C地址（7位0x44 << 1）

extern float sht40_humidity;
extern float sht40_temperature;

/* 函数声明 ------------------------------------------------------------------*/
int32_t SHT40AD1B_Init(void);
int32_t SHT40AD1B_ReadData(float *hum, float *temp);
void SHT40AD1B_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SHT40_H */

