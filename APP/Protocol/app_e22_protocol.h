#ifndef __APP_E22_PROTOCOL_H__
#define __APP_E22_PROTOCOL_H__

#include "app_bmp580.h"
#include "app_e22.h"
#include "gy95t.h"
#include "temp.h"
#include "gps.h"
#include "rtc.h"
#include <stdint.h>
#include <stdbool.h>

// 协议常量
#define E22_PROTO_HEAD1                  0xAA
#define E22_PROTO_HEAD2                  0x55
#define E22_PROTO_TAIL                   0xEE
#define E22_PROTO_MAX_LEN                128

#define E22_FLAG_TEMP_VALID              (1 << 0)
#define E22_FLAG_HUM_VALID               (1 << 1)
#define E22_FLAG_PRESS_VALID             (1 << 2)
#define E22_FLAG_GPS_VALID               (1 << 3)
#define E22_FLAG_GY95T_VALID             (1 << 4)

#pragma pack(push, 1)
typedef struct {
    float temp;             
    float humidity;         
    float pressure;         
    float lon;              
    float lat;              
    uint32_t date;          
    uint32_t beijing_time;  
    uint8_t flag;           
    GY95T_DataDef gy95t;    
} E22_EnvGPS_Data_t;

typedef struct {
    uint8_t buf[E22_PROTO_MAX_LEN];
    uint16_t len;
    bool valid;
} E22_Protocol_Packet_t;
#pragma pack(pop)

// 外部变量
extern struct bmp5_dev bmp580_dev;
extern BMP580_Data_t bmp580_data;
extern SensorData_t g_sensor_cache;
extern GPS_RMC_Data_t g_gps_cache;
extern GY95T_DataDef gy95t_data;

// 函数声明
uint8_t E22_Protocol_Calc_XOR_Check(const uint8_t *buf, uint16_t len);
bool E22_Protocol_Pack_EnvGPS(const E22_EnvGPS_Data_t *in_data, E22_Protocol_Packet_t *out_packet);
bool E22_Protocol_Unpack_EnvGPS(const E22_Protocol_Packet_t *in_packet, E22_EnvGPS_Data_t *out_data);
bool E22_Protocol_Packet_To_HexStr(const E22_Protocol_Packet_t *packet, char *out_str, uint16_t str_len);
uint8_t Sensor_Global_Init(void);
void Task_Sensor_Collect(void);
void Task_E22_Send(void);
void E22_Protocol_Convert_SensorData(E22_EnvGPS_Data_t *proto_data);

// 传感器任务
void Temp_Collect(void);
void Task_BMP580_Collect(void);
void Task_GPS_Collect(void);
void Task_GY95T_Collect(void);
void Task_Data_Merge(void);

uint8_t GPS_Read_Data(GPS_RMC_Data_t *gps_data);
HAL_StatusTypeDef GY95T_Drone_Init(void);

#endif /* __APP_E22_PROTOCOL_H__ */

