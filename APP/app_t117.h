#ifndef INC_APP_T117_H_
#define INC_APP_T117_H_

#include "bsp_t117.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>



/* ========================== 应用层错误码定义 ========================== */
typedef enum {
    APP_T117_OK          = 0,  // 操作成功
    APP_T117_INIT_FAIL   = 1,  // 初始化失败
    APP_T117_READ_FAIL   = 2,  // 温度读取失败（含CRC校验失败）
    APP_T117_PARAM_ERR   = 3   // 参数错误
} APP_T117_ErrCode;

/* ========================== 应用层配置（可根据需求调整）========================== */
#define APP_T117_MEAS_MODE    T117_SINGLE_CONVERT  // 测量模式：单次测量
#define APP_T117_AVG_MODE     T117_AVG_8           // 平均次数：8次（平衡精度/速度）
#define APP_T117_MEAS_FREQ    T117_FREQ_1HZ        // 连续模式下频率：1Hz（单次模式无效）
#define APP_T117_PD_MODE      T117_PD_ON           // 低功耗模式：开启
#define APP_T117_READ_DELAY   1000                 // 测温间隔（ms）

/* ========================== 函数声明 ========================== */
/**
 * @brief  T117P应用层初始化（封装BSP层初始化，配置默认参数）
 * @retval APP_T117_ErrCode: 错误码（APP_T117_OK表示成功）
 */
APP_T117_ErrCode APP_T117_Init(void);

/**
 * @brief  读取T117P温度值（封装BSP层读取，含参数校验）
 * @param  pTemp: 温度值存储指针（单位：℃，精度保留4位小数）
 * @retval APP_T117_ErrCode: 错误码
 */
APP_T117_ErrCode APP_T117_ReadTemperature(float *pTemp);

/**
 * @brief  T117P测试函数（main函数可直接调用，循环读取温度并打印）
 * @note   需提前初始化串口（用于打印）、I2C2、系统时钟
 * @retval 无
 */
void APP_T117_Test(void);

/**
 * @brief  读取并验证T117P配置参数（供main直接调用）
 * @retval APP_T117_ErrCode: 错误码
 */
APP_T117_ErrCode APP_T117_CheckConfig(void);

#endif /* INC_APP_T117_H_ */

