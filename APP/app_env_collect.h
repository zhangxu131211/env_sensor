#ifndef __APP_ENV_COLLECT_H
#define __APP_ENV_COLLECT_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

// ==================== 配置参数（可根据需求修改） ====================
#define I2C_TIMEOUT_MS        50      // I2C通信超时时间(ms)
#define COLLECT_RETRY_CNT     2       // 传感器通信失败重试次数
#define COLLECT_INTERVAL      1000    // 采集间隔时间(ms)

// ==================== 传感器数据结构体 ====================
/**
 * @brief 环境传感器采集数据结构体
 * @note  存储所有传感器的采集数据和状态
 */
typedef struct {
    float t117_temp;   // TMP117高精度温度 (℃)
    float sht40_temp;  // SHT40温度 (℃)
    float sht40_humi;  // SHT40湿度 (%RH)
    float bmp580_temp; // BMP580温度 (℃)
    float bmp580_press;// BMP580气压 (hPa)
    uint8_t collect_flag;// 采集状态标识: 1-成功(至少1个传感器采集成功)，0-全部失败
    uint8_t t117_ok;   // TMP117采集状态: 1-成功，0-失败
    uint8_t sht40_ok;  // SHT40采集状态: 1-成功，0-失败
    uint8_t bmp580_ok; // BMP580采集状态: 1-成功，0-失败
} EnvSensorData_t;

// ==================== 外部接口函数 ====================
/**
 * @brief  环境传感器初始化（统一初始化所有传感器）
 * @retval 0-全部初始化成功，-1-部分/全部失败
 */
int8_t EnvSensor_Init(void);

/**
 * @brief  环境传感器数据采集（含I2C冲突处理）
 * @param  p_data: 存储采集数据的结构体指针
 * @retval 0-采集成功，-1-采集失败
 */
int8_t EnvSensor_Collect(EnvSensorData_t *p_data);

/**
 * @brief  环境传感器主任务（周期采集，建议放在main循环中）
 * @note   内部按COLLECT_INTERVAL自动触发采集
 */
void EnvSensor_MainTask(void);

/**
 * @brief  I2C总线软复位（解决总线卡死问题）
 * @param  hi2c: 目标I2C句柄（如&hi2c2）
 */
void I2C_SoftReset(I2C_HandleTypeDef *hi2c);

#endif /* __APP_ENV_COLLECT_H */

