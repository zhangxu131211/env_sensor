#include "gps.h"
#include <stdio.h>
#include <stdlib.h>

// ************************** 全局变量定义 **************************
GPS_RMC_Data_t g_gps_data;  // 全局GPS解析数据

// ************************** 私有函数声明 **************************
static void GPS_Clear_Data(void);
static float GPS_NMEA_To_Decimal(char *nmea_str);

// ************************** 函数实现 **************************
/**
 * @brief 初始化GPS解析模块
 */
void GPS_Init(void)
{
  GPS_Clear_Data();  // 清空解析数据
}

/**
 * @brief 清空GPS解析数据
 */
static void GPS_Clear_Data(void)
{
  memset(&g_gps_data, 0, sizeof(GPS_RMC_Data_t));
  g_gps_data.is_valid = 0;
}

/**
 * @brief NMEA经纬度格式转十进制
 * @param nmea_str: NMEA格式的经纬度字符串
 * @retval float: 十进制经纬度值
 */
static float GPS_NMEA_To_Decimal(char *nmea_str)
{
  // 1. 基础校验：空指针/过短字符串直接返回0
  if(nmea_str == NULL || strlen(nmea_str) < 5)
  {
    return 0.0f;
  }

  // 2. 核心优化：用数值运算替代字符串截取，避免边界错误
  float nmea_val = atof(nmea_str); // 直接转成数值（如纬度3410.546955，经度10852.440893）
  
  // 3. 自动判断经度/纬度，计算度和分
  int deg = 0;
  float min = 0.0f;
  
  // 纬度：ddmm.mmmmm → 数值<10000（如3410.546955）
  // 经度：dddmm.mmmmm → 数值≥10000（如10852.440893）
  if(nmea_val < 10000.0f) // 纬度（ddmm.mmmmm）
  {
    deg = (int)(nmea_val / 100);       // 3410.546955/100=34.10546955 → 取整34
    min = nmea_val - deg * 100;        // 3410.546955 - 34*100 = 10.546955
  }
  else // 经度（dddmm.mmmmm）
  {
    deg = (int)(nmea_val / 100);       // 10852.440893/100=108.52440893 → 取整108
    min = nmea_val - deg * 100;        // 10852.440893 - 108*100 = 52.440893
  }

  // 4. 十进制转换：度 + 分/60
  return deg + min / 60.0f;
}

/**
 * @brief 解析GNRMC帧数据
 */
uint8_t GPS_Parse_RMC(uint8_t *buf, uint16_t len)
{
  // 1. 清空上次解析数据
  GPS_Clear_Data();

  // 2. 检查输入有效性
  if(buf == NULL || len < 10)
  {
    return 0;
  }

  // 3. 查找$GNRMC帧起始位置
  char *rmc_start = strstr((char*)buf, "$GNRMC");
  if(rmc_start == NULL)
  {
    return 0;
  }

  // 4. 按逗号/换行分割字段
  char *fields[GPS_RMC_FIELD_MAX] = {NULL};
  uint8_t field_idx = 0;
  fields[field_idx] = strtok(rmc_start, ",\r\n");

  while(fields[field_idx] != NULL && field_idx < GPS_RMC_FIELD_MAX - 1)
  {
    field_idx++;
    fields[field_idx] = strtok(NULL, ",\r\n");
  }

  // 5. 检查字段数是否足够（GNRMC至少10个有效字段）
  if(field_idx < 10)
  {
    return 0;
  }

  // 6. 提取核心字段
  // 6.1 UTC时间
  if(fields[1] != NULL)
  {
    strncpy(g_gps_data.utc_time, fields[1], GPS_TIME_LEN - 1);
    g_gps_data.utc_time[GPS_TIME_LEN - 1] = '\0'; // 确保字符串结束符
  }

  // 6.2 定位状态
  if(fields[2] != NULL)
  {
    g_gps_data.status = fields[2][0];
    // 只有定位有效时，才解析经纬度等数据
    if(g_gps_data.status == 'A')
    {
      // 6.3 纬度
      if(fields[3] != NULL)
      {
        g_gps_data.latitude = GPS_NMEA_To_Decimal(fields[3]);
      }

      // 6.4 纬度方向
      if(fields[4] != NULL)
      {
        g_gps_data.lat_dir = fields[4][0];
      }

      // 6.5 经度
      if(fields[5] != NULL)
      {
        g_gps_data.longitude = GPS_NMEA_To_Decimal(fields[5]);
      }

      // 6.6 经度方向
      if(fields[6] != NULL)
      {
        g_gps_data.lon_dir = fields[6][0];
      }

      // 6.7 速度（节）
      if(fields[7] != NULL)
      {
        g_gps_data.speed_knot = atof(fields[7]);
        g_gps_data.speed_kmh = g_gps_data.speed_knot * 1.852f;  // 节转公里/小时
      }

      // 6.8 航向
      if(fields[8] != NULL)
      {
        g_gps_data.course = atof(fields[8]);
      }

      // 6.9 日期
      if(fields[9] != NULL)
      {
        strncpy(g_gps_data.date, fields[9], GPS_DATE_LEN - 1);
        g_gps_data.date[GPS_DATE_LEN - 1] = '\0'; // 确保字符串结束符
      }

      g_gps_data.is_valid = 1;  // 标记解析有效
    }
  }

  return g_gps_data.is_valid;
}

/**
 * @brief 把 UTC 时间（HHMMSS.sss）格式化成 HH:MM:SS
 * @param out_time: 输出缓冲区（至少9字节）
 * @param utc_time: 原始UTC时间字符串
 */
void GPS_Format_Time(char *out_time, const char *utc_time)
{
    if (out_time == NULL || utc_time == NULL) return;

    // 确保字符串长度足够
    if (strlen(utc_time) < 6)
    {
        strcpy(out_time, "00:00:00");
        return;
    }

    // 按 HH:MM:SS 格式拼接
    sprintf(out_time, "%c%c:%c%c:%c%c",
            utc_time[0], utc_time[1],
            utc_time[2], utc_time[3],
            utc_time[4], utc_time[5]);
}

/**
 * @brief UTC时间转北京时间（UTC+8）
 * @param beijing_time: 输出缓冲区（至少9字节）
 * @param utc_time: 原始UTC时间字符串（HHMMSS.sss）
 */
void GPS_UTC_To_Beijing_Time(char *beijing_time, const char *utc_time)
{
    if (beijing_time == NULL || utc_time == NULL) return;

    // UTC 时间格式：HHMMSS.sss
    int hh = (utc_time[0] - '0') * 10 + (utc_time[1] - '0');
    int mm = (utc_time[2] - '0') * 10 + (utc_time[3] - '0');
    int ss = (utc_time[4] - '0') * 10 + (utc_time[5] - '0');

    // 北京时间 = UTC + 8 小时
    hh += 8;

    // 处理跨天
    if (hh >= 24)
    {
        hh -= 24;
    }

    // 格式化为 HH:MM:SS
    sprintf(beijing_time, "%02d:%02d:%02d", hh, mm, ss);
}

/**
 * @brief GPS原始日期（DDMMYY）格式化成 YYYY年MM月DD日
 * @param out_date: 输出缓冲区（至少16字节）
 * @param gps_date: 原始GPS日期字符串（6位，DDMMYY）
 */
void GPS_Format_Date(char *out_date, const char *gps_date)
{
    if (out_date == NULL || gps_date == NULL) return;

    // 确保字符串长度足够（6位）
    if (strlen(gps_date) < 6)
    {
        strcpy(out_date, "未知日期");
        return;
    }

    // 解析原始日期：DDMMYY → 日(DD)、月(MM)、年(YY)
    int dd = (gps_date[0] - '0') * 10 + (gps_date[1] - '0');
    int mm = (gps_date[2] - '0') * 10 + (gps_date[3] - '0');
    int yy = (gps_date[4] - '0') * 10 + (gps_date[5] - '0');
    
    // 补全年份（20xx，适配2000-2099年）
    int year = 2000 + yy;

    // 格式化为「YYYY年MM月DD日」
    sprintf(out_date, "%d年%d月%d日", year, mm, dd);
}

/**
 * @brief 打印GPS解析结果
 */
void GPS_Print_Result(void)
{
    char fmt_time[20];
    char beijing_time[20];
    char fmt_date[20];  // 格式化日期缓冲区

    GPS_Format_Time(fmt_time, g_gps_data.utc_time);
    GPS_UTC_To_Beijing_Time(beijing_time, g_gps_data.utc_time);
    GPS_Format_Date(fmt_date, g_gps_data.date);

    printf("===== LC29H GPS解析结果 =====\r\n");
    printf("UTC时间：%s\r\n", fmt_time);
    printf("北京时间：%s\r\n", beijing_time);   
    printf("定位状态：%c（A=有效，V=无效）\r\n", g_gps_data.status);
  
    if(g_gps_data.is_valid && g_gps_data.status == 'A')
    {
        printf("纬度：%.6f °%c\r\n", g_gps_data.latitude, g_gps_data.lat_dir);
        printf("经度：%.6f °%c\r\n", g_gps_data.longitude, g_gps_data.lon_dir);
        printf("航向：%.2f °\r\n", g_gps_data.course);
        printf("速度：%.2f 节 / %.2f 公里/小时\r\n", g_gps_data.speed_knot, g_gps_data.speed_kmh);
    }
    else
    {
        printf("纬度：未定位\r\n");
        printf("经度：未定位\r\n");
        printf("航向：0.00 °\r\n");
        printf("速度：0.00 节 / 0.00 公里/小时\r\n");
    }
  
    printf("日期：%s\r\n", fmt_date);
    printf("==============================\r\n\r\n");
}


