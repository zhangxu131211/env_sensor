#include "system_state_machine.h"
#include "string.h"
#include "stdint.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "app_env_collect.h"
#include "app_bmp580.h"
#include "app_t117.h"
#include "app_sht40.h"
#include "gps.h"
#include "gy95t.h"
#include "app_e22.h"
#include "usart.h"
#include "rtc.h"

/* 系统状态变量 */
static uint8_t system_state = 0;
static uint32_t system_state_time = 0;

/* 环境数据采集状态变量 */
static uint8_t env_collect_state = 0;
static uint32_t env_collect_time = 0;

/* GPS数据采集状态变量 */
static uint8_t gps_collect_state = 0;
static uint32_t gps_collect_time = 0;

/* GY95T数据采集状态变量 */
static uint8_t gy95t_collect_state = 0;
static uint32_t gy95t_collect_time = 0;

/* 数据采集完成标志 */
bool env_collect_complete = false;
bool gps_collect_complete = false;
bool gy95t_collect_complete = false;

/* BMP580数据存储 */
extern struct bmp5_dev bmp580_dev;
extern BMP580_Data_t bmp580_data;

/* SHT40数据存储 - 本地变量 */
static float g_sht40_humidity = 0.0f;
static float g_sht40_temperature = 0.0f;

/* T117数据存储 */
float g_t117_temperature = 0.0f;

/* 使用现有的GPS数据结构 - 从gps.h中声明 */
extern GPS_RMC_Data_t g_gps_data;

/* GPS相关外部变量声明 */
extern uint8_t gnss_rx_complete;
extern uint8_t gnss_rx_buf[];
extern uint16_t gnss_rx_len;
extern uint8_t GPS_Parse_RMC(uint8_t *buf, uint16_t len);

/* GY95T数据结构定义 - 9轴传感器数据 */
typedef struct {
    // 加速度计数据 (g)
    float acc_x;
    float acc_y;
    float acc_z;
    
    // 陀螺仪数据 (度/秒)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // 磁力计数据 (高斯)
    float mag_x;
    float mag_y;
    float mag_z;
    
    // 姿态角 (度)
    float roll;     // 横滚角
    float pitch;    // 俯仰角
    float yaw;      // 偏航角
    
    // 四元数
    float q0;
    float q1;
    float q2;
    float q3;
    
    // 温度 (摄氏度)
    float temperature;
    
    // 校准等级
    uint8_t calibration_level;
} GY95T_SystemData_t;

/* GY95T数据实例 */
GY95T_SystemData_t g_gy95t_data = {0};

/* 协议数据缓冲区 */
#define PROTOCOL_BUFFER_SIZE 256
#define PROTOCOL_HEADER_1    0xAA
#define PROTOCOL_HEADER_2    0x55
#define PROTOCOL_CMD_DATA    0x01

typedef struct {
    uint8_t header[2];          // 帧头 AA 55
    uint8_t length;             // 数据长度
    uint8_t cmd;                // 命令字
    uint8_t data[240];          // 数据域
    uint8_t checksum;           // 校验和
} Protocol_Frame_t;

static Protocol_Frame_t tx_frame;
static uint8_t protocol_buffer[PROTOCOL_BUFFER_SIZE];
static uint16_t protocol_data_len = 0;

/* 外部函数声明 */
extern HAL_StatusTypeDef GY95T_Drone_Init(void);
extern HAL_StatusTypeDef GY95T_ReadData(GY95T_DataDef *gy_data);

/* 简化的GPS数据结构 - 只包含实际存在的字段 */
typedef struct {
    double latitude;        // 纬度
    double longitude;       // 经度
    float altitude;         // 海拔高度
    uint8_t hour;           // 时
    uint8_t minute;         // 分
    uint8_t second;         // 秒
    uint8_t day;            // 日
    uint8_t month;          // 月
    uint8_t year;           // 年
    uint8_t satellites;     // 卫星数量
    float speed;            // 速度
    uint8_t valid;          // 数据有效性
} Simple_GPS_Data_t;

/* 简化的GPS数据实例 */
Simple_GPS_Data_t g_simple_gps_data = {0};

/* 系统状态机初始化函数 */
void System_StateMachine_Init(void)
{
    // 初始化环境传感器
    EnvSensor_Init();
    
    // 初始化E22模块
    // App_E22_Init();
    
    // 初始化GY95T模块
    GY95T_Drone_Init();
    
    // 初始化系统状态
    system_state = 0;
    system_state_time = HAL_GetTick();
    
    printf("System State Machine Initialized\r\n");
}

/* 协议帧打包函数 */
static void Protocol_PackFrame(void)
{
    uint8_t *ptr = tx_frame.data;
    uint8_t checksum = 0;
    uint16_t data_index = 0;
    
    // 清空发送缓冲区
    memset(&tx_frame, 0, sizeof(Protocol_Frame_t));
    
    // 设置帧头
    tx_frame.header[0] = PROTOCOL_HEADER_1;
    tx_frame.header[1] = PROTOCOL_HEADER_2;
    tx_frame.cmd = PROTOCOL_CMD_DATA;
    
    // 打包环境数据 (20字节)
    memcpy(ptr + data_index, &g_t117_temperature, sizeof(float));
    data_index += sizeof(float);
    
    memcpy(ptr + data_index, &g_sht40_temperature, sizeof(float));
    data_index += sizeof(float);
    
    memcpy(ptr + data_index, &g_sht40_humidity, sizeof(float));
    data_index += sizeof(float);
    
    memcpy(ptr + data_index, &bmp580_data.pressure, sizeof(float));
    data_index += sizeof(float);
    
    memcpy(ptr + data_index, &bmp580_data.temperature, sizeof(float));
    data_index += sizeof(float);
    
    // 打包简化的GPS数据 (32字节)
    memcpy(ptr + data_index, &g_simple_gps_data.latitude, sizeof(double));
    data_index += sizeof(double);
    
    memcpy(ptr + data_index, &g_simple_gps_data.longitude, sizeof(double));
    data_index += sizeof(double);
    
    memcpy(ptr + data_index, &g_simple_gps_data.altitude, sizeof(float));
    data_index += sizeof(float);
    
    memcpy(ptr + data_index, &g_simple_gps_data.hour, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    memcpy(ptr + data_index, &g_simple_gps_data.minute, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    memcpy(ptr + data_index, &g_simple_gps_data.second, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    memcpy(ptr + data_index, &g_simple_gps_data.day, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    memcpy(ptr + data_index, &g_simple_gps_data.month, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    memcpy(ptr + data_index, &g_simple_gps_data.year, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    memcpy(ptr + data_index, &g_simple_gps_data.satellites, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    memcpy(ptr + data_index, &g_simple_gps_data.speed, sizeof(float));
    data_index += sizeof(float);
    
    memcpy(ptr + data_index, &g_simple_gps_data.valid, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    // 打包GY95T 9轴数据 (76字节)
    // 加速度计数据
    memcpy(ptr + data_index, &g_gy95t_data.acc_x, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.acc_y, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.acc_z, sizeof(float));
    data_index += sizeof(float);
    
    // 陀螺仪数据
    memcpy(ptr + data_index, &g_gy95t_data.gyro_x, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.gyro_y, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.gyro_z, sizeof(float));
    data_index += sizeof(float);
    
    // 磁力计数据
    memcpy(ptr + data_index, &g_gy95t_data.mag_x, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.mag_y, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.mag_z, sizeof(float));
    data_index += sizeof(float);
    
    // 姿态角数据
    memcpy(ptr + data_index, &g_gy95t_data.roll, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.pitch, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.yaw, sizeof(float));
    data_index += sizeof(float);
    
    // 四元数数据
    memcpy(ptr + data_index, &g_gy95t_data.q0, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.q1, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.q2, sizeof(float));
    data_index += sizeof(float);
    memcpy(ptr + data_index, &g_gy95t_data.q3, sizeof(float));
    data_index += sizeof(float);
    
    // 温度和校准等级
    memcpy(ptr + data_index, &g_gy95t_data.temperature, sizeof(float));
    data_index += sizeof(float);
    
    memcpy(ptr + data_index, &g_gy95t_data.calibration_level, sizeof(uint8_t));
    data_index += sizeof(uint8_t);
    
    // 设置数据长度
    tx_frame.length = data_index;
    
    // 计算校验和
    checksum = PROTOCOL_HEADER_1 + PROTOCOL_HEADER_2 + tx_frame.length + tx_frame.cmd;
    for(uint16_t i = 0; i < data_index; i++)
    {
        checksum += tx_frame.data[i];
    }
    tx_frame.checksum = checksum;
    
    // 复制到发送缓冲区
    memcpy(protocol_buffer, &tx_frame, sizeof(Protocol_Frame_t) - sizeof(tx_frame.data) + data_index);
    protocol_data_len = sizeof(Protocol_Frame_t) - sizeof(tx_frame.data) + data_index;
    
    App_E22_Send_Protocol_Array(E22_RX_ADDR, E22_RX_CHANNEL, protocol_buffer, protocol_data_len);

    printf("Protocol Frame Packed - Length: %d bytes\r\n", protocol_data_len);
    printf("Data Content: T117=%.2fC, SHT40=%.2fC/%.1f%%, BMP580=%.2fhPa\r\n",
           g_t117_temperature, g_sht40_temperature, g_sht40_humidity, bmp580_data.pressure/100.0f);
    printf("GPS: Lat=%.6f, Lon=%.6f, Alt=%.1fm, Time=%02d:%02d:%02d\r\n",
           g_simple_gps_data.latitude, g_simple_gps_data.longitude, g_simple_gps_data.altitude,
           g_simple_gps_data.hour, g_simple_gps_data.minute, g_simple_gps_data.second);
    printf("GY95T: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°, Temp=%.1f°C\r\n",
           g_gy95t_data.roll, g_gy95t_data.pitch, g_gy95t_data.yaw, g_gy95t_data.temperature);
    printf("SHT40: Temp=%.1f°C, Humidity=%.1f%%\r\n", 
            g_sht40_temperature, g_sht40_humidity);       
}

/* 系统状态机主任务函数 */
void System_StateMachine_MainTask(void)
{
    // 根据当前系统状态执行相应任务
    switch(system_state)
    {
        case 0: // 初始化状态
            if((HAL_GetTick() - system_state_time) > 1000)
            {
                system_state = 1;
                system_state_time = HAL_GetTick();
                printf("Entering Data Collection State\r\n");
            }
            break;
            
        case 1: // 数据采集状态
            // 执行环境数据采集
            EnvCollect_StateMachine();
            
            // 执行GPS数据采集
            GPSCollect_StateMachine();
            
            // 执行GY95T数据采集
            GY95TCollect_StateMachine();
            
            // 检查是否所有数据采集完成
            if(env_collect_complete && gps_collect_complete && gy95t_collect_complete)
            {
                printf("All Data Collection Complete\r\n");
                
                // 组协议帧并发送
                Protocol_PackFrame();
                
                // 重置完成标志，准备下一轮采集
                env_collect_complete = false;
                gps_collect_complete = false;
                gy95t_collect_complete = false;
            }
            break;
            
        default:
            system_state = 0;
            system_state_time = HAL_GetTick();
            break;
    }
}

/* 环境数据采集状态机 */
void EnvCollect_StateMachine(void)
{
    switch(env_collect_state)
    {
        case 0: // 开始采集环境数据
            env_collect_time = HAL_GetTick();
            env_collect_state = 1;
            break;
            
        case 1: // 采集T117温度数据
            if((HAL_GetTick() - env_collect_time) > 100)
            {
                float temperature;
                APP_T117_ReadTemperature(&temperature);
                g_t117_temperature = temperature;
                env_collect_time = HAL_GetTick();
                env_collect_state = 2;
            }
            break;
            
        case 2: // 采集SHT40温湿度数据 - 修复版本
            if((HAL_GetTick() - env_collect_time) > 100)
            {
                float humidity, temperature;
                if(SHT40AD1B_ReadData(&humidity, &temperature) == 0)
                {
                    g_sht40_humidity = humidity;
                    g_sht40_temperature = temperature;
                }
                env_collect_time = HAL_GetTick();
                env_collect_state = 3;
            }
            break;
            
        case 3: // 采集BMP580气压数据
            if((HAL_GetTick() - env_collect_time) > 100)
            {
                APP_BMP580_GetData(&bmp580_dev, &bmp580_data);
                env_collect_time = HAL_GetTick();
                env_collect_state = 4;
            }
            break;
            
        case 4: // 环境数据采集完成
            env_collect_complete = true;
            printf("Environment Data Collection Complete\r\n");
            env_collect_state = 0;
            break;
            
        default:
            env_collect_state = 0;
            break;
    }
}

/* GPS数据采集状态机 - 带RTC备选和北京时间 */
void GPSCollect_StateMachine(void)
{
    static uint32_t gps_check_count = 0;
    static uint32_t last_check_time = 0;
    
    switch(gps_collect_state)
    {
        case 0: // 开始采集GPS数据
            gps_collect_time = HAL_GetTick();
            gps_collect_state = 1;
            gps_check_count = 0;
            printf("=== GPS Collection Started ===\r\n");
            printf("等待GPS数据接收...\r\n");
            break;
            
        case 1: // GPS接收等待状态
            // 每2秒检查一次GPS状态（降低检查频率）
            if((HAL_GetTick() - last_check_time) >= 2000)
            {
                last_check_time = HAL_GetTick();
                gps_check_count++;
                
                printf("--- GPS Check #%lu (after %lu ms) ---\r\n", 
                       gps_check_count, HAL_GetTick() - gps_collect_time);
                printf("GPS接收状态: %s\r\n", gnss_rx_complete ? "完成" : "等待中");
                printf("GPS缓冲区长度: %d\r\n", gnss_rx_len);
                
                // 详细分析GPS缓冲区内容
                if(gnss_rx_len > 0)
                {
                    printf("GPS数据内容: ");
                    for(int i = 0; i < gnss_rx_len && i < 10; i++)
                    {
                        printf("%02X ", gnss_rx_buf[i]);
                    }
                    printf("\r\n");
                    
                    if(gnss_rx_len >= 6)
                    {
                        printf("GPS ASCII内容: %.6s\r\n", gnss_rx_buf);
                    }
                    
                    // 检查是否是有效的NMEA语句
                    if(gnss_rx_buf[0] == '$' && gnss_rx_len > 10)
                    {
                        printf("检测到NMEA格式数据!\r\n");
                    }
                    else if(gnss_rx_buf[0] == 0x00 || gnss_rx_buf[0] == 0xFF)
                    {
                        printf("警告: GPS数据为0x00或0xFF，可能是硬件问题\r\n");
                    }
                    else
                    {
                        printf("非NMEA格式数据，首字节: 0x%02X\r\n", gnss_rx_buf[0]);
                    }
                }
                else
                {
                    printf("GPS缓冲区为空\r\n");
                }
            }
            
            // 处理接收到的GPS数据
            if(gnss_rx_complete && gnss_rx_len > 10)
            {
                printf("=== 检测到完整GPS帧 ===\r\n");
                printf("GPS帧长度: %d 字节\r\n", gnss_rx_len);
                printf("GPS完整数据: %.*s\r\n", gnss_rx_len, gnss_rx_buf);
                
                // 按照现有的GPS处理逻辑
                gnss_rx_complete = 0;
                uint8_t parse_ret = GPS_Parse_RMC(gnss_rx_buf, gnss_rx_len);
                
                printf("GPS解析结果: %d\r\n", parse_ret);
                printf("GPS状态: %c\r\n", g_gps_data.status);
                
                if(parse_ret == 1 && g_gps_data.status == 'A')
                {
                    printf("=== GPS数据有效 ===\r\n");
                    printf("UTC时间: %s\r\n", g_gps_data.utc_time);
                    printf("日期: %s\r\n", g_gps_data.date);
                    printf("纬度: %.6f\r\n", g_gps_data.latitude);
                    printf("经度: %.6f\r\n", g_gps_data.longitude);
                    printf("速度: %.2f km/h\r\n", g_gps_data.speed_kmh);
                    
                    // GPS有效时更新RTC时间（使用UTC时间）
                    uint8_t rtc_update_result = RTC_Update_From_GPS(g_gps_data.utc_time, g_gps_data.date);
                    if(rtc_update_result)
                    {
                        printf("=== RTC时间已同步更新 ===\r\n");
                        RTC_Print_Time();  // 打印更新后的RTC时间
                    }
                    else
                    {
                        printf("=== RTC时间同步失败 ===\r\n");
                    }
                    
                    // 将GPS数据复制到协议结构
                    g_simple_gps_data.latitude = g_gps_data.latitude;
                    g_simple_gps_data.longitude = g_gps_data.longitude;
                    g_simple_gps_data.altitude = 473.386f;
                    g_simple_gps_data.valid = 1;
                    
                    // 解析时间和日期 - UTC转北京时间
                    if(strlen(g_gps_data.utc_time) >= 6)
                    {
                        uint8_t utc_hour = (g_gps_data.utc_time[0] - '0') * 10 + (g_gps_data.utc_time[1] - '0');
                        uint8_t utc_minute = (g_gps_data.utc_time[2] - '0') * 10 + (g_gps_data.utc_time[3] - '0');
                        uint8_t utc_second = (g_gps_data.utc_time[4] - '0') * 10 + (g_gps_data.utc_time[5] - '0');
                        
                        // UTC转北京时间 (UTC+8)
                        uint8_t beijing_hour = utc_hour + 8;
                        if(beijing_hour >= 24)
                        {
                            beijing_hour -= 24;
                        }
                        
                        g_simple_gps_data.hour = beijing_hour;
                        g_simple_gps_data.minute = utc_minute;
                        g_simple_gps_data.second = utc_second;
                        
                        printf("UTC时间: %02d:%02d:%02d -> 北京时间: %02d:%02d:%02d\r\n",
                               utc_hour, utc_minute, utc_second,
                               beijing_hour, utc_minute, utc_second);
                    }
                    
                    // 解析日期数据
                    if(strlen(g_gps_data.date) >= 6)
                    {
                        g_simple_gps_data.day = (g_gps_data.date[0] - '0') * 10 + (g_gps_data.date[1] - '0');
                        g_simple_gps_data.month = (g_gps_data.date[2] - '0') * 10 + (g_gps_data.date[3] - '0');
                        g_simple_gps_data.year = (g_gps_data.date[4] - '0') * 10 + (g_gps_data.date[5] - '0');
                    }
                    
                    // 设置卫星数量和速度
                    g_simple_gps_data.satellites = 11;
                    g_simple_gps_data.speed = g_gps_data.speed_kmh;
                    
                    printf("=== GPS数据已复制到协议结构（北京时间） ===\r\n");
                    printf("协议GPS - 纬度: %.6f, 经度: %.6f, 海拔: %.1fm\r\n", 
                           g_simple_gps_data.latitude, g_simple_gps_data.longitude, g_simple_gps_data.altitude);
                    printf("协议GPS - 北京时间: %02d:%02d:%02d, 日期: %02d-%02d-%02d\r\n",
                           g_simple_gps_data.hour, g_simple_gps_data.minute, g_simple_gps_data.second,
                           g_simple_gps_data.year, g_simple_gps_data.month, g_simple_gps_data.day);
                    printf("协议GPS - 速度: %.2f km/h, 卫星: %d, 有效: %d\r\n",
                           g_simple_gps_data.speed, g_simple_gps_data.satellites, g_simple_gps_data.valid);
                }
                else
                {
                    printf("=== GPS数据无效 ===\r\n");
                    printf("解析结果: %d, 状态: %c\r\n", parse_ret, g_gps_data.status);
                    printf("使用RTC北京时间作为备选...\r\n");
                    
                    // GPS数据无效，使用RTC北京时间作为备选
                    g_simple_gps_data.valid = 0;  // 标记GPS数据无效
                    g_simple_gps_data.latitude = 0.0;
                    g_simple_gps_data.longitude = 0.0;
                    g_simple_gps_data.altitude = 0.0;
                    g_simple_gps_data.satellites = 0;
                    g_simple_gps_data.speed = 0.0;
                    
                    // 从RTC获取北京时间（RTC已经是北京时间）
                    if(RTC_Read_Time())
                    {
                        g_simple_gps_data.hour = g_rtc_time.hour;
                        g_simple_gps_data.minute = g_rtc_time.min;
                        g_simple_gps_data.second = g_rtc_time.sec;
                        g_simple_gps_data.day = g_rtc_time.date;
                        g_simple_gps_data.month = g_rtc_time.month;
                        g_simple_gps_data.year = g_rtc_time.year;
                        
                        printf("RTC北京时间: %02d:%02d:%02d  %02d-%02d-20%02d\r\n",
                               g_simple_gps_data.hour, g_simple_gps_data.minute, g_simple_gps_data.second,
                               g_simple_gps_data.day, g_simple_gps_data.month, g_simple_gps_data.year);
                    }
                    else
                    {
                        // RTC也读取失败，使用默认值
                        g_simple_gps_data.hour = 0;
                        g_simple_gps_data.minute = 0;
                        g_simple_gps_data.second = 0;
                        g_simple_gps_data.day = 1;
                        g_simple_gps_data.month = 1;
                        g_simple_gps_data.year = 24;
                        printf("RTC读取失败，使用默认北京时间\r\n");
                    }
                    
                    printf("协议GPS数据设置为RTC北京时间（GPS无效）\r\n");
                }
                
                // 清除接收缓冲区
                gnss_rx_len = 0;
                memset(gnss_rx_buf, 0, GNSS_BUF_SIZE);
                
                // 标记GPS数据采集完成
                gps_collect_complete = true;
                gps_collect_state = 0;
                printf("=== GPS数据采集完成 ===\r\n");
            }
            else if((HAL_GetTick() - gps_collect_time) > 60000)  // 60秒超时
            {
                printf("=== GPS数据采集超时 (60秒) ===\r\n");
                printf("最终状态 - 接收完成: %d, 缓冲区长度: %d\r\n", gnss_rx_complete, gnss_rx_len);
                
                if(gnss_rx_len > 0 && gnss_rx_len <= 10)
                {
                    printf("提示: GPS只接收到少量数据，检查:\r\n");
                    printf("1. GPS模块是否正确连接\r\n");
                    printf("2. GPS天线是否正常\r\n");
                    printf("3. GPS模块是否上电\r\n");
                    printf("4. 串口波特率是否匹配\r\n");
                }
                
                // 超时也使用RTC北京时间作为备选
                printf("GPS采集超时，使用RTC北京时间作为备选...\r\n");
                
                g_simple_gps_data.valid = 0;  // 标记GPS数据无效
                g_simple_gps_data.latitude = 0.0;
                g_simple_gps_data.longitude = 0.0;
                g_simple_gps_data.altitude = 0.0;
                g_simple_gps_data.satellites = 0;
                g_simple_gps_data.speed = 0.0;
                
                // 从RTC获取北京时间
                if(RTC_Read_Time())
                {
                    g_simple_gps_data.hour = g_rtc_time.hour;
                    g_simple_gps_data.minute = g_rtc_time.min;
                    g_simple_gps_data.second = g_rtc_time.sec;
                    g_simple_gps_data.day = g_rtc_time.date;
                    g_simple_gps_data.month = g_rtc_time.month;
                    g_simple_gps_data.year = g_rtc_time.year;
                    
                    printf("超时备选 - RTC北京时间: %02d:%02d:%02d  %02d-%02d-20%02d\r\n",
                           g_simple_gps_data.hour, g_simple_gps_data.minute, g_simple_gps_data.second,
                           g_simple_gps_data.day, g_simple_gps_data.month, g_simple_gps_data.year);
                }
                else
                {
                    // 使用默认值
                    g_simple_gps_data.hour = 0;
                    g_simple_gps_data.minute = 0;
                    g_simple_gps_data.second = 0;
                    g_simple_gps_data.day = 1;
                    g_simple_gps_data.month = 1;
                    g_simple_gps_data.year = 24;
                    printf("超时备选 - RTC读取失败，使用默认北京时间\r\n");
                }
                
                // 标记GPS数据采集完成
                gps_collect_complete = true;
                gps_collect_state = 0;
            }
            break;
            
        default:
            gps_collect_state = 0;
            break;
    }
}

/* GY95T数据采集状态机 */
void GY95TCollect_StateMachine(void)
{
    extern GY95T_DataDef gy95t_data;  // 外部变量声明
    
    switch(gy95t_collect_state)
    {
        case 0: // 开始采集GY95T数据
            gy95t_collect_time = HAL_GetTick();
            gy95t_collect_state = 1;
            break;
            
        case 1: // 采集GY95T数据
            if((HAL_GetTick() - gy95t_collect_time) > 200)
            {
                // 读取GY95T数据
                if(GY95T_ReadData(&gy95t_data) == HAL_OK)
                {
                    // 转换并存储数据
                    g_gy95t_data.acc_x = gy95t_data.acc_x_g;
                    g_gy95t_data.acc_y = gy95t_data.acc_y_g;
                    g_gy95t_data.acc_z = gy95t_data.acc_z_g;
                    
                    g_gy95t_data.gyro_x = gy95t_data.gyro_x_dps;
                    g_gy95t_data.gyro_y = gy95t_data.gyro_y_dps;
                    g_gy95t_data.gyro_z = gy95t_data.gyro_z_dps;
                    
                    g_gy95t_data.mag_x = gy95t_data.mag_x_raw * 0.001f;  // 转换为高斯
                    g_gy95t_data.mag_y = gy95t_data.mag_y_raw * 0.001f;
                    g_gy95t_data.mag_z = gy95t_data.mag_z_raw * 0.001f;
                    
                    g_gy95t_data.roll = gy95t_data.roll_deg;
                    g_gy95t_data.pitch = gy95t_data.pitch_deg;
                    g_gy95t_data.yaw = gy95t_data.yaw_deg;
                    
                    g_gy95t_data.q0 = gy95t_data.q0;
                    g_gy95t_data.q1 = gy95t_data.q1;
                    g_gy95t_data.q2 = gy95t_data.q2;
                    g_gy95t_data.q3 = gy95t_data.q3;
                    
                    g_gy95t_data.temperature = gy95t_data.temp_c;
                    g_gy95t_data.calibration_level = gy95t_data.level;
                    
                    gy95t_collect_complete = true;
                    printf("GY95T Data Collection Complete\r\n");
                }
                gy95t_collect_state = 0;
            }
            break;
            
        default:
            gy95t_collect_state = 0;
            break;
    }
}

