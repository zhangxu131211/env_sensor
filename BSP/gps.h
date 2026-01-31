#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <string.h>

// ************************** 宏定义 **************************
#define GPS_RMC_FIELD_MAX    15      // GNRMC帧最大字段数
#define GPS_TIME_LEN         10      // 时间字符串长度（HH:MM:SS）
#define GPS_DATE_LEN         7       // 原始日期字符串长度（DDMMYY）

// ************************** 数据结构定义 **************************
/**
 * @brief GPS RMC帧解析数据结构体
 */
typedef struct
{
    char utc_time[GPS_TIME_LEN];     // UTC时间（原始：HHMMSS.sss）
    char date[GPS_DATE_LEN];         // 日期（原始：DDMMYY）
    char status;                     // 定位状态（A=有效，V=无效）
    char lat_dir;                    // 纬度方向（N=北，S=南）
    char lon_dir;                    // 经度方向（E=东，W=西）
    float latitude;                  // 纬度（十进制）
    float longitude;                 // 经度（十进制）
    float speed_knot;                // 速度（节）
    float speed_kmh;                 // 速度（公里/小时）
    float course;                    // 航向（度）
    uint8_t is_valid;                // 解析数据是否有效（1=有效，0=无效）
} GPS_RMC_Data_t;

// ************************** 全局变量声明 **************************
extern GPS_RMC_Data_t g_gps_data;    // 全局GPS解析数据

// ************************** 函数声明 **************************
/**
 * @brief 初始化GPS解析模块
 */
void GPS_Init(void);

/**
 * @brief 解析GNRMC帧数据
 * @param buf: 原始GPS数据缓冲区
 * @param len: 缓冲区长度
 * @retval 1=解析成功且有效，0=解析失败/无效
 */
uint8_t GPS_Parse_RMC(uint8_t *buf, uint16_t len);

/**
 * @brief 打印GPS解析结果
 */
void GPS_Print_Result(void);

/**
 * @brief 把 UTC 时间（HHMMSS.sss）格式化成 HH:MM:SS
 * @param out_time: 输出缓冲区（至少9字节）
 * @param utc_time: 原始UTC时间字符串
 */
void GPS_Format_Time(char *out_time, const char *utc_time);

/**
 * @brief UTC时间转北京时间（UTC+8）
 * @param beijing_time: 输出缓冲区（至少9字节）
 * @param utc_time: 原始UTC时间字符串（HHMMSS.sss）
 */
void GPS_UTC_To_Beijing_Time(char *beijing_time, const char *utc_time);

/**
 * @brief GPS原始日期（DDMMYY）格式化成 YYYY年MM月DD日
 * @param out_date: 输出缓冲区（至少16字节）
 * @param gps_date: 原始GPS日期字符串（6位，DDMMYY）
 */
void GPS_Format_Date(char *out_date, const char *gps_date);

#endif /* __GPS_H */

