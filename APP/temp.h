#ifndef __TEMP_H__
#define __TEMP_H__

#include <stdint.h>
#include <stdbool.h>

#define TEMP_THRESH_LOW    (-40.0f)    
#define TEMP_THRESH_HIGH   (125.0f)    
#define HUMIDITY_INVALID   (-1.0f)     

typedef enum {
    OVERLAP_MODE_PRI_T117P    = 1,    
    OVERLAP_MODE_PRI_SHT4X    = 2,    
    OVERLAP_MODE_DOUBLE_CHECK = 3     
} OverlapMode_e;

typedef struct {
    float temp;       
    float humidity;   
    bool valid;       
    bool hardware_fault;  // 硬件离线（无ACK）
    uint8_t ack_fail_cnt; // ACK失败计数
    uint32_t timestamp;  
} SensorCache_t;

typedef struct {
    float temp;       
    float humidity;   
    bool valid;       
} SensorData_t;

extern volatile SensorCache_t g_t117p_cache;
extern volatile SensorCache_t g_sht4x_cache;
extern volatile SensorData_t g_fused_result;

// 初始化
bool Sensor_Init(void);

// 主采集任务（严格ACK判断）
void Task_Sensor_Sequence_Collect(void);

// 总线管理层（供其他模块调用）
bool I2C2_Check_Device_ACK(uint8_t dev_addr, uint32_t timeout);
void I2C2_Bus_Recovery(void);
bool I2C2_Transmit_With_ACK(uint8_t addr, uint8_t* data, uint16_t len);
bool I2C2_Receive_With_ACK(uint8_t addr, uint8_t* data, uint16_t len);

SensorData_t Sensor_GetFusedData(void);
void Sensor_Set_OverlapMode(OverlapMode_e mode);

#endif /* __TEMP_H__ */

