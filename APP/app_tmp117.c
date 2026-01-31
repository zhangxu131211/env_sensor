#include "app_tmp117.h"
#include "bsp_tmp117.h"
//#include "i2c.h"
#include "stdio.h"


// 全局I2C通信缓冲区（与驱动层共享）
uint8_t tmp117_buffer[3];
// 存储当前温度值
static double current_temperature = 0.0;

/**
 * @brief  初始化TMP117传感器
 * @return 初始化成功返回true，失败返回false
 */
bool App_TMP117_Init(void) {
    uint16_t device_id;
    
	if (HAL_I2C_IsDeviceReady(&tmp117_i2c, TMP117_I2C_R_ADDR_GND, 3, 100) != HAL_OK) {
        printf("TMP117: I2C设备未就绪，地址错误或接线异常\r\n");
        return false;
    }
	
    // 1. 读取设备ID验证传感器连接
    if (!getDeviceID(&tmp117_i2c, tmp117_buffer, &device_id)) {
        printf("TMP117: 读取设备ID失败\r\n");
        return false;
    }
    if (device_id != 0x0117) {
        printf("TMP117: 设备ID不匹配 (实际:0x%04X, 预期:0x%04X)\r\n", 
               device_id, 0x0117);
        return false;
    }
    
    // 2. 软件复位传感器（清除原有校准偏移）
    if (!softwareReset(&tmp117_i2c, tmp117_buffer)) {
        printf("TMP117: 软件复位失败\r\n");
        return false;
    }
    HAL_Delay(2);  // 等待复位完成（数据手册要求至少2ms）
    
    // 3. 配置传感器工作模式
    if (!setConversionMode(&tmp117_i2c, tmp117_buffer, TMP117_CC_MODE)) {  // 连续转换模式
        printf("TMP117: 设置转换模式失败\r\n");
        return false;
    }
    
    // 4. 配置转换时间（125ms）
    if (!setConversionTime(&tmp117_i2c, tmp117_buffer, TMP117_C125mS)) {
        printf("TMP117: 设置转换时间失败\r\n");
        return false;
    }
    
    // 5. 配置平均次数（8次平均，提高精度）
    if (!setAveraging(&tmp117_i2c, tmp117_buffer, TMP117_AVG_8)) {
        printf("TMP117: 设置平均次数失败\r\n");
        return false;
    }
    
    // 6. 配置报警极性（高电平有效）
    if (!setAlertPolarity(&tmp117_i2c, tmp117_buffer, TMP117_POL_H)) {
        printf("TMP117: 设置报警极性失败\r\n");
        return false;
    }
    
    // 7. 配置报警模式为数据就绪模式
    if (!setAlertMode(&tmp117_i2c, tmp117_buffer, TMP117_DATA_MODE)) {
        printf("TMP117: 设置报警模式失败\r\n");
        return false;
    }
    
    printf("TMP117: 初始化成功\r\n");
    return true;
}

/**
 * @brief  读取当前温度值
 * @param  temp: 存储温度值的指针（单位：℃）
 * @return 读取成功返回true，失败返回false
 */
bool App_TMP117_ReadTemp(double *temp) {
    if (temp == NULL) return false;
    
    // 直接读取温度，不叠加校准偏移
    if (getResultTemperature(&tmp117_i2c, tmp117_buffer, &current_temperature)) {
        *temp = current_temperature;
        return true;
    } else {
        printf("TMP117: 温度读取失败\r\n");
        return false;
    }
}

/**
 * @brief  设置温度报警阈值
 * @param  high_temp: 高温报警阈值（℃）
 * @param  low_temp: 低温报警阈值（℃）
 * @return 配置成功返回true，失败返回false
 */
bool App_TMP117_SetAlarmThreshold(double high_temp, double low_temp) {
    if (!setHighLimitTemperature(&tmp117_i2c, tmp117_buffer, high_temp)) {
        printf("TMP117: 设置高温阈值失败\r\n");
        return false;
    }
    if (!setLowLimitTemperature(&tmp117_i2c, tmp117_buffer, low_temp)) {
        printf("TMP117: 设置低温阈值失败\r\n");
        return false;
    }
    return true;
}

/**
 * @brief  传感器校准（仅执行一次，基于已知准确温度）
 * @param  target_temp: 已知的目标温度（如标准温度计的25.0℃）
 * @return 校准成功返回true，失败返回false
 */
bool App_TMP117_Calibrate(double target_temp) {
    if (calibrate(&tmp117_i2c, tmp117_buffer, target_temp)) {
        printf("TMP117: 校准成功，校准基准温度：%.2f ℃\r\n", target_temp);
        return true;
    } else {
        printf("TMP117: 校准失败\r\n");
        return false;
    }
}

/**
 * @brief  应用主循环
 */
void App_TMP117_MainTask(void) {
    double temp;
    int8_t init_retry = 3;  // 初始化重试次数

    // 带重试的初始化
    while (init_retry-- > 0) {
        if (App_TMP117_Init()) {
            break;
        }
        printf("TMP117: 重试初始化...（剩余%d次）\r\n", init_retry);
        HAL_Delay(1000);
    }
    
    if (init_retry < 0) {
        printf("TMP117: 初始化失败，进入错误状态\r\n");
        while (1) {
            // 可添加错误指示（如LED闪烁）
            HAL_Delay(500);
        }
    }
    
    // 设置报警阈值（示例：高温30℃，低温10℃）
//    App_TMP117_SetAlarmThreshold(30.0, 10.0);
    
    // 可选：仅执行一次校准（传入已知的准确温度，如25.0℃）
    // 若无需校准，注释此行即可
//    App_TMP117_Calibrate(25.0);
    	HAL_Delay(200);  // 等待第一次转换完成

    // 主循环：周期性读取温度（仅读取，不重复校准）
    while (1) {
        if (App_TMP117_ReadTemp(&temp)) {

					        // 临时调试：打印raw值
//        uint8_t data[2];
//        HAL_I2C_Mem_Read(&tmp117_i2c, TMP117_I2C_R_ADDR_GND, 0x00, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
//        int16_t raw = (int16_t)((data[0] << 8) | data[1]);
//        printf("Raw: %d (0x%04X), Temp: %.4f ℃\r\n", raw, (uint16_t)raw, current_temperature);
					
            printf("温度: %.4f ℃\r\n", temp);
            
            // 超温判断（可选启用）
//            if (temp > 30.0) {
//                printf("警告：温度过高！\r\n");
//            } else if (temp < 10.0) {
//                printf("警告：温度过低！\r\n");
//            }
        }
        HAL_Delay(1000);  // 1秒读取一次（大于配置的125ms转换时间，合理）
    }
}
