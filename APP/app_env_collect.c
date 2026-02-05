#include "app_env_collect.h"
#include "app_bmp580.h"
#include "app_sht40.h"
#include "app_t117.h"
#include "i2c.h"       
#include "gpio.h"      
#include "stdio.h"     
#include <string.h>

// ==================== 全局变量 ====================
static EnvSensorData_t g_env_data = {0};  // 全局环境数据缓存
static uint32_t g_last_collect_tick = 0;  // 上次采集时间戳

// ==================== 静态函数声明 ====================
/**
 * @brief  传感器采集通用重试封装
 * @param  func: 传感器采集函数指针（返回0成功，非0失败）
 * @retval 0-成功，-1-失败
 */
static int8_t Sensor_Collect_WithRetry(int8_t (*func)(void));

// ==================== 静态函数实现 ====================
/**
 * @brief  I2C总线软复位（解决总线卡死）
 * @param  hi2c: I2C句柄
 */
void I2C_SoftReset(I2C_HandleTypeDef *hi2c)
{
    // 1. 关闭I2C外设
    HAL_I2C_DeInit(hi2c);
    HAL_Delay(5);
    
    // 2. 重新初始化I2C（使用现有GPIO配置，不重新配置引脚）
    HAL_I2C_Init(hi2c);
    HAL_Delay(5);
    
    DEBUG_LOG("I2C SoftReset完成，实例: %p\r\n", hi2c);
}

/**
 * @brief  传感器采集通用重试封装
 * @param  func: 采集函数指针
 * @retval 0-成功，-1-失败
 */
static int8_t Sensor_Collect_WithRetry(int8_t (*func)(void))
{
    int8_t ret = -1;
    for(uint8_t i = 0; i < COLLECT_RETRY_CNT; i++)
    {
        ret = func();
        if(ret == 0)
        {
            break;  // 采集成功，退出重试
        }
        HAL_Delay(10);  // 重试前延时，降低总线冲突概率
    }
    return ret;
}

// ==================== T117采集适配函数 ====================
static int8_t T117_Collect(void)
{
    float temp = 0.0f;
    APP_T117_ErrCode err = APP_T117_ReadTemperature(&temp);
    if(err != APP_T117_OK)
    {
        DEBUG_LOG("[T117] Collect Failed, ErrCode: %d\r\n", err);
        return -1;
    }
    g_env_data.t117_temp = temp;
    return 0;
}

// ==================== SHT40采集适配函数 ====================
static int8_t SHT40_Collect(void)
{
    float hum = 0.0f, temp = 0.0f;
    int32_t ret = SHT40AD1B_ReadData(&hum, &temp);
    if(ret != 0)
    {
        DEBUG_LOG("[SHT40] Collect Failed, Ret: %d\r\n", ret);
        return -1;
    }
    g_env_data.sht40_temp = temp;
    g_env_data.sht40_humi = hum;
    return 0;
}

// ==================== BMP580采集适配函数 ====================
static int8_t BMP580_Collect(void)
{
    BMP580_Data_t bmp_data = {0};
    int8_t ret = APP_BMP580_GetData(&bmp580_dev, &bmp_data);
    if(ret != BMP5_OK)
    {
        DEBUG_LOG("[BMP580] Collect Failed, Ret: %d\r\n", ret);
        return -1;
    }
    g_env_data.bmp580_temp = bmp_data.temperature;
    g_env_data.bmp580_press = bmp_data.pressure / 100.0f;  // Pa -> hPa
    return 0;
}

// ==================== 公开函数实现 ====================
/**
 * @brief  环境传感器统一初始化
 * @retval 0-全部成功，-1-部分/全部失败
 */
int8_t EnvSensor_Init(void)
{
    int8_t ret = 0;
    uint8_t err_cnt = 0;
    
    // 初始化前复位I2C总线
     I2C_SoftReset(&hi2c2);
    
    // 初始化T117
     if(APP_T117_Init() != APP_T117_OK)
     {
         err_cnt++;
        DEBUG_LOG("[T117] Init Failed\r\n");
     }
     HAL_Delay(5);
    
    // 初始化SHT40
     if(SHT40AD1B_Init() != 0)
     {
         err_cnt++;
         DEBUG_LOG("[SHT40] Init Failed\r\n");
     }
     HAL_Delay(5);
    
    // 初始化BMP580
    if(BMP580_Init() != BMP5_OK)
    {
        err_cnt++;
        DEBUG_LOG("[BMP580] Init Failed\r\n");
    }
    
    // 初始化失败则返回错误
    if(err_cnt > 0)
    {
        ret = -1;
        DEBUG_LOG("EnvSensor Init Failed, Err Count: %d\r\n", err_cnt);
    }
    else
    {
        DEBUG_LOG("EnvSensor Init Success\r\n");
    }
    return ret;
}

/**
 * @brief  环境传感器数据采集
 * @param  p_data: 数据存储指针
 * @retval 0-成功，-1-失败
 */
int8_t EnvSensor_Collect(EnvSensorData_t *p_data)
{
    if(p_data == NULL) return -1;
    
    // 清空原有数据
    memset(p_data, 0, sizeof(EnvSensorData_t));
    
    // 检查I2C状态，异常则复位
    if(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
    {
        DEBUG_LOG("I2C Bus Not Ready, Reset...\r\n");
        I2C_SoftReset(&hi2c2);
    }
    
    // 采集T117（带重试）
    if(Sensor_Collect_WithRetry(T117_Collect) == 0)
    {
        p_data->t117_temp = g_env_data.t117_temp;
        p_data->t117_ok = 1;
    }
    HAL_Delay(5);
    
    // 采集SHT40（带重试）
    if(Sensor_Collect_WithRetry(SHT40_Collect) == 0)
    {
        p_data->sht40_temp = g_env_data.sht40_temp;
        p_data->sht40_humi = g_env_data.sht40_humi;
        p_data->sht40_ok = 1;
    }
    HAL_Delay(5);
    
    // 采集BMP580（带重试）
    if(Sensor_Collect_WithRetry(BMP580_Collect) == 0)
    {
        p_data->bmp580_temp = g_env_data.bmp580_temp;
        p_data->bmp580_press = g_env_data.bmp580_press;
        p_data->bmp580_ok = 1;
    }
    
    // 采集状态：至少1个传感器成功则整体成功
    p_data->collect_flag = (p_data->t117_ok | p_data->sht40_ok | p_data->bmp580_ok);
    
    return p_data->collect_flag ? 0 : -1;
}

/**
 * @brief  环境传感器主任务（周期采集）
 */
void EnvSensor_MainTask(void)
{
    if(HAL_GetTick() - g_last_collect_tick >= COLLECT_INTERVAL)
    {
        g_last_collect_tick = HAL_GetTick();
        
        // 执行采集
        int8_t collect_ret = EnvSensor_Collect(&g_env_data);
        
        // 打印采集结果
        if(collect_ret == 0)
        {
            DEBUG_LOG("===== Env Sensor Data =====\r\n");
            if(g_env_data.t117_ok)
                DEBUG_LOG("T117 Temp: %.4f ℃\r\n", g_env_data.t117_temp);
            if(g_env_data.sht40_ok)
                DEBUG_LOG("SHT40 Temp: %.2f ℃, Hum: %.2f %%RH\r\n", 
                       g_env_data.sht40_temp, g_env_data.sht40_humi);
            if(g_env_data.bmp580_ok)
                DEBUG_LOG("BMP580 Temp: %.2f ℃, Press: %.2f hPa\r\n", 
                       g_env_data.bmp580_temp, g_env_data.bmp580_press);
            DEBUG_LOG("==========================\r\n");
        }
        else
        {
            DEBUG_LOG("EnvSensor Collect Failed, Reset I2C...\r\n");
            I2C_SoftReset(&hi2c2);
        }
    }
}

