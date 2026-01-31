#include "app_e22_protocol.h"
#include "task_scheduler.h"
#include <string.h>
#include <stdio.h>
#include "i2c.h"

SensorData_t g_sensor_cache = {0};
GPS_RMC_Data_t g_gps_cache = {0};

uint8_t E22_Protocol_Calc_XOR_Check(const uint8_t *buf, uint16_t len) {
    if (buf == NULL || len == 0) return 0;
    uint8_t check_sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        check_sum ^= buf[i];
    }
    return check_sum;
}

bool E22_Protocol_Pack_EnvGPS(const E22_EnvGPS_Data_t *in_data, E22_Protocol_Packet_t *out_packet) {
    if (in_data == NULL || out_packet == NULL) return false;

    memset(out_packet->buf, 0, E22_PROTO_MAX_LEN);
    out_packet->len = 0;
    out_packet->valid = false;

    uint16_t idx = 0;
    
    out_packet->buf[idx++] = E22_PROTO_HEAD1;
    out_packet->buf[idx++] = E22_PROTO_HEAD2;

    memcpy(&out_packet->buf[idx], &in_data->temp, sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_packet->buf[idx], &in_data->humidity, sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_packet->buf[idx], &in_data->pressure, sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_packet->buf[idx], &in_data->lon, sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_packet->buf[idx], &in_data->lat, sizeof(float)); 
    idx += sizeof(float);

    out_packet->buf[idx++] = (uint8_t)((in_data->date / 10000) - 2000);
    out_packet->buf[idx++] = (uint8_t)((in_data->date / 100) % 100);
    out_packet->buf[idx++] = (uint8_t)(in_data->date % 100);

    out_packet->buf[idx++] = (uint8_t)(in_data->beijing_time / 10000);
    out_packet->buf[idx++] = (uint8_t)((in_data->beijing_time / 100) % 100);
    out_packet->buf[idx++] = (uint8_t)(in_data->beijing_time % 100);

    out_packet->buf[idx++] = in_data->flag;

    memcpy(&out_packet->buf[idx], &in_data->gy95t, sizeof(GY95T_DataDef)); 
    idx += sizeof(GY95T_DataDef);

    uint8_t check_sum = E22_Protocol_Calc_XOR_Check(out_packet->buf, idx);
    out_packet->buf[idx++] = check_sum;
    out_packet->buf[idx++] = E22_PROTO_TAIL;

    if (idx > E22_PROTO_MAX_LEN) return false;

    out_packet->len = idx;
    out_packet->valid = true;
    return true;
}

bool E22_Protocol_Unpack_EnvGPS(const E22_Protocol_Packet_t *in_packet, E22_EnvGPS_Data_t *out_data) {
    if (in_packet == NULL || out_data == NULL || !in_packet->valid || 
        in_packet->len < (32 + sizeof(GY95T_DataDef))) {
        return false;
    }

    memset(out_data, 0, sizeof(E22_EnvGPS_Data_t));
    uint16_t idx = 0;

    if (in_packet->buf[idx++] != E22_PROTO_HEAD1 || 
        in_packet->buf[idx++] != E22_PROTO_HEAD2) {
        return false;
    }

    memcpy(&out_data->temp, &in_packet->buf[idx], sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_data->humidity, &in_packet->buf[idx], sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_data->pressure, &in_packet->buf[idx], sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_data->lon, &in_packet->buf[idx], sizeof(float)); 
    idx += sizeof(float);
    memcpy(&out_data->lat, &in_packet->buf[idx], sizeof(float)); 
    idx += sizeof(float);

    out_data->date = (uint32_t)(2000 + in_packet->buf[idx++]) * 10000;
    out_data->date += (uint32_t)in_packet->buf[idx++] * 100;
    out_data->date += (uint32_t)in_packet->buf[idx++];

    out_data->beijing_time = (uint32_t)in_packet->buf[idx++] * 10000;
    out_data->beijing_time += (uint32_t)in_packet->buf[idx++] * 100;
    out_data->beijing_time += (uint32_t)in_packet->buf[idx++];

    out_data->flag = in_packet->buf[idx++];

    memcpy(&out_data->gy95t, &in_packet->buf[idx], sizeof(GY95T_DataDef));
    idx += sizeof(GY95T_DataDef);

    uint8_t recv_check = in_packet->buf[idx++];
    uint8_t calc_check = E22_Protocol_Calc_XOR_Check(in_packet->buf, idx-1);
    if (recv_check != calc_check) return false;

    if (in_packet->buf[idx++] != E22_PROTO_TAIL) return false;

    return true;
}

bool E22_Protocol_Packet_To_HexStr(const E22_Protocol_Packet_t *packet, char *out_str, uint16_t str_len) {
    if (packet == NULL || out_str == NULL || !packet->valid || 
        str_len < (2 * packet->len + 1)) {
        return false;
    }

    char *ptr = out_str;
    for (uint16_t i = 0; i < packet->len; i++) {
        ptr += sprintf(ptr, "%02X", packet->buf[i]);
    }
    *ptr = '\0';
    return true;
}

static void E22_Protocol_Convert_SensorData(E22_EnvGPS_Data_t *proto_data) {
    if (proto_data == NULL) return;
    memset(proto_data, 0, sizeof(E22_EnvGPS_Data_t));

    proto_data->temp = g_sensor_cache.temp;
    proto_data->humidity = g_sensor_cache.humidity;
    proto_data->pressure = bmp580_data.pressure / 100.0f;

    if (g_gps_cache.is_valid && g_gps_cache.status == 'A') {
        proto_data->lon = g_gps_cache.longitude;
        proto_data->lon = (g_gps_cache.lon_dir == 'W') ? -proto_data->lon : proto_data->lon;
        proto_data->lat = g_gps_cache.latitude;
        proto_data->lat = (g_gps_cache.lat_dir == 'S') ? -proto_data->lat : proto_data->lat;
        proto_data->flag |= E22_FLAG_GPS_VALID;
    } else {
        proto_data->lon = 0.0f;
        proto_data->lat = 0.0f;
    }

    RTC_Read_Time();
    proto_data->date = (2000 + g_rtc_time.year) * 10000 + g_rtc_time.month * 100 + g_rtc_time.date;
    
    if (g_gps_cache.is_valid && g_gps_cache.status == 'A') {
        char beijing_time_str[20];
        GPS_UTC_To_Beijing_Time(beijing_time_str, g_gps_cache.utc_time);
        int hour, min, sec;
        sscanf(beijing_time_str, "%d:%d:%d", &hour, &min, &sec);
        proto_data->beijing_time = hour * 10000 + min * 100 + sec;
    } else {
        proto_data->beijing_time = g_rtc_time.hour * 10000 + g_rtc_time.min * 100 + g_rtc_time.sec;
    }

    proto_data->flag = 0;
    if (g_sensor_cache.valid) {
        proto_data->flag |= E22_FLAG_TEMP_VALID | E22_FLAG_PRESS_VALID;
        if (g_sensor_cache.humidity != HUMIDITY_INVALID) {
            proto_data->flag |= E22_FLAG_HUM_VALID;
        }
    }

    memcpy(&proto_data->gy95t, &gy95t_data, sizeof(GY95T_DataDef));
    proto_data->flag |= E22_FLAG_GY95T_VALID;
}

uint8_t Sensor_Global_Init(void) {
    if (!Sensor_Init()) {
        printf("[Sensor] 温湿度传感器初始化失败\r\n");
        return 1;
    }

    if (BMP580_Init() != BMP5_OK) {
        printf("[Sensor] BMP580初始化失败\r\n");
        return 1;
    }

//    if (GY95T_Drone_Init() == HAL_OK) {
//        printf("GY95T Init Success!\r\n");
//    } else {
//        printf("GY95T Init Failed!\r\n");
//        return 1;
//    }
    
    GPS_Init();
    printf("[Sensor] 所有传感器初始化成功\r\n");
    return 0;
}

void Task_BMP580_Collect(void) {
    static uint32_t last_check = 0;
    
    // 每 1000ms 检查一次，且与温湿度采集错开（通过调度器周期错开）
    if (Scheduler_GetSysTick() - last_check < 1000) return;
    last_check = Scheduler_GetSysTick();
    
    // 严格 ACK 检查
    extern bool I2C2_Check_Device_ACK(uint8_t dev_addr, uint32_t timeout);
    extern void I2C2_Bus_Recovery(void);
    
    if (!I2C2_Check_Device_ACK(BMP580_I2C_ADDR, 100)) {
        printf("[BMP580] No ACK\r\n");
        bmp580_data.pressure = 0.0f;
        I2C2_Bus_Recovery();
        return;
    }
    
    // 设备在线，尝试读取
    int ret = APP_BMP580_GetData(&bmp580_dev, &bmp580_data);
    if (ret != 0) {
        printf("[BMP580] Read fail (%d)\r\n", ret);
        bmp580_data.pressure = 0.0f;
    } else {
        // printf("[BMP580] %.2f\r\n", bmp580_data.pressure); // 静默成功
    }
}

void Temp_Collect(void) {
    g_sensor_cache = Sensor_GetFusedData();
    
    // 只在状态变化或异常时打印，正常时静默（避免串口刷屏）
    static uint8_t last_valid = 0;
    if (g_sensor_cache.valid != last_valid) {
        last_valid = g_sensor_cache.valid;
        if (g_sensor_cache.valid) {
            printf("[Sensor] OK: %.2fC, %.2f%%RH\r\n", 
                   g_sensor_cache.temp, g_sensor_cache.humidity);
        } else {
            printf("[Sensor] Invalid\r\n");
        }
    }
}

void Task_GPS_Collect(void) {
    if (GPS_Read_Data(&g_gps_cache) != 0) {
        printf("[GPS] 数据读取失败\r\n");
        g_gps_cache.is_valid = false;
    } else {
        if (g_gps_cache.is_valid && g_gps_cache.status == 'A') {
            printf("[GPS] 有效定位: 纬度=%.6f, 经度=%.6f\r\n", 
                   g_gps_cache.latitude, g_gps_cache.longitude);
        } else {
            printf("[GPS] 无效定位\r\n");
        }
    }
}

void Task_GY95T_Collect(void) {
    // GY95T 通常使用 UART，不涉及 I2C，无需加锁
    if (HAL_GPIO_ReadPin(GY95T_INT_GPIO_PORT, GY95T_INT_PIN) == GPIO_PIN_RESET) {
        while (HAL_GPIO_ReadPin(GY95T_INT_GPIO_PORT, GY95T_INT_PIN) == GPIO_PIN_RESET);
        if (GY95T_ReadData(&gy95t_data) == HAL_OK) { 
            printf("ACC(g): %.3f,%.3f,%.3f | GYRO(dps): %.1f,%.1f,%.1f | Att(°): %.2f,%.2f,%.2f\n",
                   gy95t_data.acc_x_g, gy95t_data.acc_y_g, gy95t_data.acc_z_g,
                   gy95t_data.gyro_x_dps, gy95t_data.gyro_y_dps, gy95t_data.gyro_z_dps,
                   gy95t_data.roll_deg, gy95t_data.pitch_deg, gy95t_data.yaw_deg);
            gy95t_data.update_flag = 1;
        } else {
            gy95t_data.update_flag = 0;
            printf("GY95T Data Read Failed!\r\n");
        }	
    }		
}

void Task_Data_Merge(void) {
    uint8_t valid_sensors = 0;
    if (g_sensor_cache.valid) valid_sensors++;
    if (bmp580_data.pressure > 0) valid_sensors++;
    if (g_gps_cache.is_valid) valid_sensors++;
    if (gy95t_data.update_flag) valid_sensors++;
    
    printf("[DataMerge] 有效传感器数量: %d/4\r\n", valid_sensors);
}

/**
 * @brief E22发送任务（添加 I2C 锁保护，防止与采集冲突）
 */
void Task_E22_Send(void) {
    printf("Task_E22_Send\r\n");
    
    // 检查数据有效性
    if (!g_sensor_cache.valid && bmp580_data.pressure == 0.0f) {
        printf("[E22] 环境传感器数据无效，取消发送\r\n");
        return;
    }

    E22_EnvGPS_Data_t proto_data = {0};
    E22_Protocol_Convert_SensorData(&proto_data);

    E22_Protocol_Packet_t packet = {0};
    if (!E22_Protocol_Pack_EnvGPS(&proto_data, &packet)) {
        printf("[E22] 协议打包失败\r\n");
        return;
    }

    uint8_t err = App_E22_Send_Protocol_Array(E22_RX_ADDR, E22_RX_CHANNEL, packet.buf, packet.len);
    if (err != 0) {
        printf("[E22] 数据发送失败，错误码=%d\r\n", err);
    } else {
        printf("[E22] Data sent: Temp=%.2fC, Hum=%.2f%%RH, Press=%.2f hPa, GPS=%.6f,%.6f\r\n",
               proto_data.temp, proto_data.humidity, proto_data.pressure,
               proto_data.lat, proto_data.lon);
    }
}

uint8_t GPS_Read_Data(GPS_RMC_Data_t *gps_data) {
    if (gps_data == NULL) return 1;
    uint8_t gps_raw_buf[256] = {0};
    uint16_t gps_raw_len = 0;

    if (gps_raw_len > 0) {
        GPS_Parse_RMC(gps_raw_buf, gps_raw_len);
    }
    memcpy(gps_data, &g_gps_data, sizeof(GPS_RMC_Data_t));
    return (gps_data->is_valid && gps_data->status == 'A') ? 0 : 1;
}

