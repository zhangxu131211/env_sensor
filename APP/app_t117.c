#include "app_t117.h"
#include "main.h"


/* 外部声明I2C2句柄（需在I2C初始化文件中定义） */
extern I2C_HandleTypeDef hi2c2;

/* 静态缓冲区（用于T117P数据读写，避免频繁申请内存） */
static uint8_t s_t117Buffer[3] = {0};

/**
 * @brief  T117P应用层初始化
 * @retval APP_T117_ErrCode: 错误码
 */
APP_T117_ErrCode APP_T117_Init(void) {

    // 调用BSP层初始化
    if (!BSP_T117_Init(&hi2c2, s_t117Buffer)) {
        DEBUG_LOG("[T117] Init Err: BSP init failed\r\n");
        return APP_T117_INIT_FAIL;
    }

    return APP_T117_OK;
}

/**
 * @brief  读取T117P温度值
 * @param  pTemp: 温度值存储指针
 * @retval APP_T117_ErrCode: 错误码
 */
APP_T117_ErrCode APP_T117_ReadTemperature(float *pTemp) {
    // 1. 参数校验
    if (pTemp == NULL) {
        DEBUG_LOG("[T117] Read Err: Param is NULL\r\n");
        return APP_T117_PARAM_ERR;
    }

    // 2. 调用BSP层读取温度（含CRC校验）
    if (!BSP_T117_GetTemperature(&hi2c2, s_t117Buffer, pTemp)) {
        DEBUG_LOG("[T117] Read Err: CRC failed or I2C error\r\n");
        return APP_T117_READ_FAIL;
    }

    // 3. 温度值范围校验（T117P量程：-103℃~153℃）
    if (*pTemp < -103.0f || *pTemp > 153.0f) {
        DEBUG_LOG("[T117] Read Warn: Temp out of range (%.4f℃)\r\n", *pTemp);
    }

    return APP_T117_OK;
}

/**
 * @brief  T117P测试函数（main函数直接调用）
 * @retval 无
 */
void APP_T117_Test(void) {
    float temperature = 0.0f;
    APP_T117_ErrCode err;

    // 1. 初始化T117P
    err = APP_T117_Init();
    if (err != APP_T117_OK) {
        DEBUG_LOG("[T117] Test Err: Init failed (err=%d)\r\n", err);
        while (1) {
            HAL_Delay(1000); // 初始化失败，死循环等待复位
        }
    }
		
    // 2. 循环读取温度并打印
    DEBUG_LOG("[T117] Test Start: Read temp every %dms\r\n", APP_T117_READ_DELAY);
    while (1) {
        // 读取温度
        err = APP_T117_ReadTemperature(&temperature);
        if (err == APP_T117_OK) {
            DEBUG_LOG("[T117] Temp: %.4f ℃\r\n", temperature);
        } else {
            DEBUG_LOG("[T117] Read Temp Failed (err=%d)\r\n", err);
        }
        // 延时
        HAL_Delay(APP_T117_READ_DELAY);
    }
}


