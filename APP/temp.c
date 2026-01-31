#include "temp.h"
#include "app_sht40.h"
#include "app_t117.h"
#include "task_scheduler.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32l4xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

static OverlapMode_e g_overlap_mode = OVERLAP_MODE_PRI_SHT4X;
volatile SensorCache_t g_t117p_cache = {0};
volatile SensorCache_t g_sht4x_cache = {0};
volatile SensorData_t g_fused_result = {0};

// I2C 设备地址（根据你的实际修改）
#define T117_ADDR       0x40  // T117P 默认地址，改成你的实际地址
#define SHT40_ADDR      0x89  // SHT40AD1B
#define BMP580_ADDR     0x47  // BMP580

#define ACK_TIMEOUT     100   // 100ms ACK 超时
#define MAX_ACK_FAIL    3     // 连续3次无ACK标记硬件故障

typedef enum {
    STATE_CHECK_T117,       // 检测T117是否在线
    STATE_READ_T117,
    STATE_DELAY_T117,
    STATE_CHECK_SHT40,      // 检测SHT40是否在线
    STATE_READ_SHT40,
    STATE_DELAY_SHT40,
    STATE_FUSION
} SeqState_t;

static SeqState_t seq_state = STATE_CHECK_T117;
static uint32_t state_delay_start = 0;

/**
 * @brief 严格的总线恢复（处理死锁）
 * @note  当检测到BUSY或设备无响应时，发送STOP并重新初始化
 */
void I2C2_Bus_Recovery(void) {
    printf("[I2C] Bus recovery...\r\n");
    
    // 1. 发送 STOP 条件（如果总线被占用）
    if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY)) {
        // 发送 STOP
        hi2c2.Instance->CR2 |= I2C_CR2_STOP;
        HAL_Delay(5);
        
        // 如果还 BUSY，发送 9 个时钟脉冲恢复
        if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY)) {
            GPIO_InitTypeDef GPIO_InitStruct = {0};
            
            // 临时配置 SCL 为 GPIO 输出
            __HAL_RCC_GPIOB_CLK_ENABLE();  // 根据你的 SCL 引脚修改
            GPIO_InitStruct.Pin = GPIO_PIN_10;  // I2C2_SCL 引脚
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
            
            // 发送 9 个时钟脉冲
            for (int i = 0; i < 9; i++) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
                HAL_Delay(1);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
                HAL_Delay(1);
            }
            
            // 发送 STOP 条件（SDA 低->高，SCL 高）
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
            HAL_Delay(1);
        }
    }
    
    // 2. 重新初始化 I2C（保守 100kHz 配置）
    HAL_I2C_DeInit(&hi2c2);
    HAL_Delay(10);
    
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x30420F13;  // 100kHz @ 80MHz
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        printf("[I2C] Re-init failed!\r\n");
    }
    
    // 禁用模拟滤波器（已知STM32L4有bug），数字滤波器拉满
    HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 15);
    HAL_Delay(5);
    
    printf("[I2C] Recovery done\r\n");
}

/**
 * @brief 检查设备 ACK（返回是否在线）
 */
bool I2C2_Check_Device_ACK(uint8_t dev_addr, uint32_t timeout) {
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, dev_addr << 1, 1, timeout);
    return (ret == HAL_OK);
}

/**
 * @brief 带ACK确认的传输
 */
bool I2C2_Transmit_With_ACK(uint8_t addr, uint8_t* data, uint16_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c2, addr << 1, data, len, ACK_TIMEOUT);
    if (ret != HAL_OK) {
        if (ret == HAL_BUSY) printf("[I2C] TX BUSY to 0x%02X\r\n", addr);
        else if (ret == HAL_TIMEOUT) printf("[I2C] TX TIMEOUT to 0x%02X\r\n", addr);
        else printf("[I2C] TX ERROR to 0x%02X, ret=%d\r\n", addr, ret);
        return false;
    }
    return true;
}

/**
 * @brief 带ACK确认的接收
 */
bool I2C2_Receive_With_ACK(uint8_t addr, uint8_t* data, uint16_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c2, addr << 1, data, len, ACK_TIMEOUT);
    if (ret != HAL_OK) {
        printf("[I2C] RX FAIL from 0x%02X, ret=%d\r\n", addr, ret);
        return false;
    }
    return true;
}

/**
 * @brief 初始化（带ACK检测）
 */
bool Sensor_Init(void) {
    
    if (SHT40AD1B_Init() != 0) {
        printf("[SHT4x] Init cmd failed\r\n");
        return false;
    }
    
    // 检查 T117（如果地址已知）

    if (APP_T117_Init() == APP_T117_OK) {
         g_t117p_cache.hardware_fault = false;
         HAL_Delay(50);
    } else {
         printf("[T117] Init cmd failed\r\n");
         g_t117p_cache.hardware_fault = true;
    }
    return true;
}

/**
 * @brief 读取 SHT40（手动实现带 ACK 检查，不用库函数）
 * @note  如果 SHT40AD1B_ReadData 内部用了 HAL，这里需要修改它或绕开
 *        假设 SHT40 读数是：发命令 -> 延时 10ms -> 读 6 字节数据
 */
static bool SHT40_Read_Manual(float* hum, float* temp) {
    uint8_t cmd = 0xFD;  // 高精度测量命令
    uint8_t data[6] = {0};
    
    // 1. 检测设备在线
    if (!I2C2_Check_Device_ACK(SHT40_ADDR, ACK_TIMEOUT)) {
        g_sht4x_cache.ack_fail_cnt++;
        return false;
    }
    
    // 2. 发送命令
    if (!I2C2_Transmit_With_ACK(SHT40_ADDR, &cmd, 1)) {
        g_sht4x_cache.ack_fail_cnt++;
        return false;
    }
    
    // 3. 等待转换（10ms高精测量）
    HAL_Delay(10);
    
    // 4. 再次检测 ACK（设备可能忙）
    uint32_t wait_start = Scheduler_GetSysTick();
    while (!I2C2_Check_Device_ACK(SHT40_ADDR, 10)) {
        if (Scheduler_GetSysTick() - wait_start > 50) {  // 最多等50ms
            return false;  // 设备忙超时
        }
    }
    
    // 5. 读取数据
    if (!I2C2_Receive_With_ACK(SHT40_ADDR, data, 6)) {
        g_sht4x_cache.ack_fail_cnt++;
        return false;
    }
    
    // 6. CRC 校验（这里简化，实际需要校验 data[2] 和 data[5]）
    // 假设数据格式正确，解析温湿度
    uint16_t t_ticks = (data[0] << 8) | data[1];
    uint16_t rh_ticks = (data[3] << 8) | data[4];
    
    *temp = -45.0f + 175.0f * ((float)t_ticks / 65535.0f);
    *hum = -6.0f + 125.0f * ((float)rh_ticks / 65535.0f);
    if (*hum > 100.0f) *hum = 100.0f;
    if (*hum < 0.0f) *hum = 0.0f;
    
    g_sht4x_cache.ack_fail_cnt = 0;
    return true;
}

/**
 * @brief 读取 T117（手动实现带 ACK 检查）
 * @note  T117 通常是发地址直接读或写寄存器后读，根据你的 APP_T117 实现调整
 *        假设 T117 是简单读取 2 字节温度数据
 */
static bool T117_Read_Manual(float* temp) {
    // 如果已标记故障，快速返回
    if (g_t117p_cache.hardware_fault) return false;
    
    // 1. 检测设备在线（关键！确认 ACK）
    if (!I2C2_Check_Device_ACK(T117_ADDR, ACK_TIMEOUT)) {
        g_t117p_cache.ack_fail_cnt++;
        if (g_t117p_cache.ack_fail_cnt >= MAX_ACK_FAIL) {
            g_t117p_cache.hardware_fault = true;
            printf("[T117] No ACK %d times, disabled\r\n", MAX_ACK_FAIL);
        }
        return false;
    }
    
    // 2. 调用你的读取函数（假设内部用HAL，但前面已确认ACK，成功率提高）
    // 如果 APP_T117_ReadTemperature 内部自己操作 I2C，确保它也有超时处理
    bool ret = APP_T117_ReadTemperature(temp);
    
    if (!ret) {
        g_t117p_cache.ack_fail_cnt++;
        if (g_t117p_cache.ack_fail_cnt >= MAX_ACK_FAIL) {
            // 连续失败不立即禁用，尝试总线恢复后再试一次
            I2C2_Bus_Recovery();
            ret = APP_T117_ReadTemperature(temp);  // 再试一次
            if (!ret) {
                g_t117p_cache.hardware_fault = true;
                printf("[T117] Read failed after recovery, disabled\r\n");
            } else {
                g_t117p_cache.ack_fail_cnt = 0;  // 恢复成功
            }
        }
        return ret;
    }
    
    g_t117p_cache.ack_fail_cnt = 0;
    return true;
}

/**
 * @brief 主状态机（严格 ACK 顺序）
 */
void Task_Sensor_Sequence_Collect(void) {
    uint32_t now = Scheduler_GetSysTick();
    
    switch(seq_state) {
        case STATE_CHECK_T117: {
            if (g_t117p_cache.hardware_fault || !I2C2_Check_Device_ACK(T117_ADDR, 50)) {
                // 不在线或已故障，跳过到 SHT40
                seq_state = STATE_CHECK_SHT40;
            } else {
                seq_state = STATE_READ_T117;
            }
            break;
        }
            
        case STATE_READ_T117: {
            float temp = 0.0f;
            bool ok = T117_Read_Manual(&temp);
            
            __disable_irq();
            if (ok && temp > -50.0f && temp < 150.0f) {
                g_t117p_cache.temp = temp;
                g_t117p_cache.valid = true;
                g_t117p_cache.timestamp = now;
            } else {
                g_t117p_cache.valid = false;
            }
            __enable_irq();
            
            state_delay_start = now;
            seq_state = STATE_DELAY_T117;
            break;
        }
            
        case STATE_DELAY_T117:
            // T117 读取后强制延时 150ms（给总线充分恢复时间）
            if (now - state_delay_start >= 150) {
                seq_state = STATE_CHECK_SHT40;
            }
            break;
            
        case STATE_CHECK_SHT40: {
            if (!I2C2_Check_Device_ACK(SHT40_ADDR, 50)) {
                printf("[SHT4x] No ACK\r\n");
                I2C2_Bus_Recovery();  // 尝试恢复
                seq_state = STATE_FUSION;  // 跳过读取直接融合
            } else {
                seq_state = STATE_READ_SHT40;
            }
            break;
        }
            
        case STATE_READ_SHT40: {
            float temp = 0.0f, hum = 0.0f;
            bool ok = SHT40_Read_Manual(&hum, &temp);
            
            __disable_irq();
            if (ok && temp >= TEMP_THRESH_LOW && temp <= TEMP_THRESH_HIGH) {
                g_sht4x_cache.temp = temp;
                g_sht4x_cache.humidity = hum;
                g_sht4x_cache.valid = true;
                g_sht4x_cache.timestamp = now;
            } else {
                g_sht4x_cache.valid = false;
            }
            __enable_irq();
            
            state_delay_start = now;
            seq_state = STATE_DELAY_SHT40;
            break;
        }
            
        case STATE_DELAY_SHT40:
            // SHT40 后延时 50ms
            if (now - state_delay_start >= 50) {
                seq_state = STATE_FUSION;
            }
            break;
            
        case STATE_FUSION: {
            __disable_irq();
            SensorCache_t t117 = g_t117p_cache;
            SensorCache_t sht4 = g_sht4x_cache;
            __enable_irq();
            
            SensorData_t result = {0};
            bool t117_ok = (t117.valid && !t117.hardware_fault && 
                          (now - t117.timestamp) < 2000);
            bool sht4_ok = (sht4.valid && (now - sht4.timestamp) < 2000);
            
            // 谁有效用谁，优先 SHT40（通常更稳）
            if (sht4_ok) {
                result.temp = sht4.temp;
                result.humidity = sht4.humidity;
                result.valid = true;
                if (t117_ok && fabs(t117.temp - sht4.temp) < 2.0f && 
                    g_overlap_mode == OVERLAP_MODE_DOUBLE_CHECK) {
                    result.temp = (t117.temp + sht4.temp) / 2.0f;
                }
            } else if (t117_ok) {
                result.temp = t117.temp;
                result.humidity = HUMIDITY_INVALID;
                result.valid = true;
            }
            
            __disable_irq();
            if (result.valid) g_fused_result = result;
            __enable_irq();
            
            // 回到起点，下个 100ms 周期再执行
            seq_state = STATE_CHECK_T117;
            break;
        }
    }
}

SensorData_t Sensor_GetFusedData(void) {
    SensorData_t ret;
    __disable_irq();
    ret = g_fused_result;
    if (!ret.valid && g_sht4x_cache.valid) {
        ret.temp = g_sht4x_cache.temp;
        ret.humidity = g_sht4x_cache.humidity;
        ret.valid = true;
    }
    __enable_irq();
    return ret;
}


