#ifndef APP_BMP580_H
#define APP_BMP580_H

#include "stm32l4xx_hal.h"
#include "bmp5.h"
#include "bmp5_defs.h"

/* 外部I2C句柄声明（用户需在主函数中初始化I2C外设） */
extern I2C_HandleTypeDef hi2c2;
#define bmp580_i2c 							hi2c2

/* BMP580 I2C地址（SDO引脚拉低为0x46，拉高为0x47，根据硬件配置修改） */
#define BMP580_I2C_ADDR        (0x47 << 1)  // HAL库I2C地址需左移1位（7位地址）

/* 传感器数据结构体 */
typedef struct {
    float pressure;   // 压力(Pa)
    float temperature;// 温度(°C)
} BMP580_Data_t;

extern struct bmp5_dev bmp580_dev;
extern BMP580_Data_t bmp580_data;

/* 函数声明 */
/**
 * @brief  BMP580传感器初始化
 * @param  dev: BMP5设备结构体指针
 * @retval 状态码: BMP5_OK(0)成功，<0失败
 */
int8_t APP_BMP580_Init(struct bmp5_dev *dev);

/**
 * @brief  配置BMP580传感器（连续模式、使能压力/温度、IIR滤波等）
 * @param  dev: BMP5设备结构体指针
 * @retval 状态码: BMP5_OK(0)成功，<0失败
 */
int8_t APP_BMP580_Config(struct bmp5_dev *dev);

/**
 * @brief  获取BMP580传感器数据（压力+温度）
 * @param  dev: BMP5设备结构体指针
 * @param  data: 传感器数据存储结构体指针
 * @retval 状态码: BMP5_OK(0)成功，<0失败
 */
int8_t APP_BMP580_GetData(struct bmp5_dev *dev, BMP580_Data_t *data);

/**
 * @brief  微秒级延迟函数（适配BMP5驱动要求）
 * @param  us: 延迟时间(us)
 * @param  intf_ptr: 接口指针（未使用）
 */
void BMP580_DelayUs(uint32_t us, void *intf_ptr);

/**
 * @brief  封装的BMP580测试函数（main直接调用）
 */
void APP_BMP580_Test(void);

int8_t BMP580_Init(void);


#endif /* APP_BMP580_H */

