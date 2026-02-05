/**
  ******************************************************************************
  * @file    app_sht40.c
  * @author  zhangxu
  * @brief   SHT40AD1B应用层驱动（HAL I2C版）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_sht40.h"
#include "sht40ad1b_reg.h"
#include "i2c.h"
#include "stdio.h"
#include "main.h"

/* 全局变量 ------------------------------------------------------------------*/
/* 定义I2C句柄（需与硬件初始化的I2C句柄一致） */
extern I2C_HandleTypeDef sht40_i2c;  

/* SHT40AD1B设备上下文（核心结构体） */
static stmdev_ctx_t sht40ad1b_ctx;

/* 温湿度数据存储 */
static float sht40_humidity;
static float sht40_temperature;

/* 私有函数声明 --------------------------------------------------------------*/
static int32_t sht40ad1b_i2c_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t sht40ad1b_i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void sht40ad1b_delay_ms(uint32_t ms);

/**
  * @brief  初始化SHT40AD1B传感器
  * @retval 0:成功, 非0:失败
  */
int32_t SHT40AD1B_Init(void)
{
    uint8_t dev_id = 0;
    
    /* 1. 初始化设备上下文（绑定I2C读写/延时函数） */
    sht40ad1b_ctx.write_reg = sht40ad1b_i2c_write;  // 绑定写函数
    sht40ad1b_ctx.read_reg  = sht40ad1b_i2c_read;   // 绑定读函数
    sht40ad1b_ctx.mdelay    = sht40ad1b_delay_ms;   // 绑定延时函数
    sht40ad1b_ctx.handle    = &sht40_i2c;               // 绑定I2C句柄

    /* 2. 读取设备ID（验证传感器连接） */
    int32_t ret = sht40ad1b_device_id_get(&sht40ad1b_ctx, &dev_id);
    if (ret != 0)
    {
        DEBUG_LOG("SHT40AD1B Read ID Failed! Ret: %d\r\n", ret);
        return -1;
    }
    DEBUG_LOG("SHT40AD1B Device ID: 0x%02X\r\n", dev_id);

    /* 3. 验证ID合法性（SHT40系列典型ID：0x89/0x00，根据手册调整） */
    if (dev_id != 0x89 && dev_id != 0x00)
    {
        DEBUG_LOG("SHT40AD1B ID Invalid! ID: 0x%02X\r\n", dev_id);
        return -2;
    }

    DEBUG_LOG("SHT40AD1B Init Success!\r\n");
    return 0;
}


/**
  * @brief  读取SHT40AD1B温湿度数据
  * @param  hum: 湿度数据指针(%)
  * @param  temp: 温度数据指针(°C)
  * @retval 0:成功, 非0:失败
  */
int32_t SHT40AD1B_ReadData(float *hum, float *temp)
{
    if (hum == NULL || temp == NULL)
    {
        return -1;
    }

    float data[2] = {0.0f};  // [0]:湿度, [1]:温度
    
    /* 调用寄存器层函数读取温湿度 */
    int32_t ret = sht40ad1b_data_get(&sht40ad1b_ctx, data);
    if (ret != 0)
    {
        DEBUG_LOG("SHT40AD1B Read Data Failed! Ret: %d\r\n", ret);
        return -2;
    }

    /* 数据校验（SHT40量程：湿度0~100%RH，温度-40~125°C） */
    if (data[0] < 0.0f || data[0] > 100.0f || 
        data[1] < -40.0f || data[1] > 125.0f)
    {
        DEBUG_LOG("SHT40AD1B Data Out Of Range! H:%.2f, T:%.2f\r\n", data[0], data[1]);
        return -3;
    }

    /* 更新数据 */
    *hum = data[0];
    *temp = data[1];
    sht40_humidity = data[0];
    sht40_temperature = data[1];

    // DEBUG_LOG("SHT40AD1B Data - Humidity: %.2f%%, Temperature: %.2f°C\r\n", *hum, *temp);
    return 0;
}

/**
  * @brief  SHT40AD1B I2C写函数（适配stmdev_ctx_t）
  * @param  handle: I2C句柄指针
  * @param  reg: 寄存器地址（SHT40无显式寄存器，仅指令）
  * @param  bufp: 写数据缓冲区
  * @param  len: 写数据长度
  * @retval 0:成功, 非0:失败
  */
static int32_t sht40ad1b_i2c_write(void *handle, uint8_t reg,  uint8_t *bufp, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    (void)reg;  // SHT40通过指令通信，无寄存器地址，忽略该参数

    /* HAL I2C写函数：地址0x44<<1=0x88，无寄存器地址，直接写指令/数据 */
    if (HAL_I2C_Master_Transmit(hi2c, SHT40AD1B_I2C_ADDR, (uint8_t *)bufp, len, 100) != HAL_OK)
    {
        /* 重试一次（增强鲁棒性） */
        if (HAL_I2C_Master_Transmit(hi2c, SHT40AD1B_I2C_ADDR, (uint8_t *)bufp, len, 100) != HAL_OK)
        {
            return -1;
        }
    }
    return 0;
}

/**
  * @brief  SHT40AD1B I2C读函数（适配stmdev_ctx_t）
  * @param  handle: I2C句柄指针
  * @param  reg: 寄存器地址（SHT40无显式寄存器，仅指令）
  * @param  bufp: 读数据缓冲区
  * @param  len: 读数据长度
  * @retval 0:成功, 非0:失败
  */
static int32_t sht40ad1b_i2c_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    (void)reg;  // SHT40通过指令通信，无寄存器地址，忽略该参数

    /* HAL I2C读函数：地址0x44<<1=0x88 */
    if (HAL_I2C_Master_Receive(hi2c, SHT40AD1B_I2C_ADDR, bufp, len, 100) != HAL_OK)
    {
        /* 重试一次（增强鲁棒性） */
        if (HAL_I2C_Master_Receive(hi2c, SHT40AD1B_I2C_ADDR, bufp, len, 100) != HAL_OK)
        {
            return -1;
        }
    }
    return 0;
}

/**
  * @brief  毫秒级延时函数（适配stmdev_ctx_t）
  * @param  ms: 延时毫秒数
  * @retval 无
  */
static void sht40ad1b_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
  * @brief  测试函数：循环读取温湿度
  * @retval 无
  */
void SHT40AD1B_Test(void)
{
    if (SHT40AD1B_Init() != 0)
    {
        DEBUG_LOG("SHT40AD1B Init Failed!\r\n");
        return;
    }

    while (1)
    {
        float hum, temp;
        if (SHT40AD1B_ReadData(&hum, &temp) == 0)
        {
            DEBUG_LOG("业务使用 - 湿度: %.2f%%, 温度: %.2f°C\r\n", hum, temp);
        }
        HAL_Delay(1000);  // 1秒读取一次
    }
}


