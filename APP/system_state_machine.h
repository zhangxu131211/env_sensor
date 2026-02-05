#ifndef __SYSTEM_STATE_MACHINE_H
#define __SYSTEM_STATE_MACHINE_H

#include "stdint.h"
#include "stdbool.h"
#include "wind_speed.h"

/* 系统状态机初始化 */
void System_StateMachine_Init(void);

/* 系统状态机主任务 */
void System_StateMachine_MainTask(void);

/* 环境数据采集状态机 */
void EnvCollect_StateMachine(void);

/* GPS数据采集状态机 - 带RTC备选和北京时间 */
void GPSCollect_StateMachine(void);

/* GY95T数据采集状态机 */
void GY95TCollect_StateMachine(void);

/* 风速数据采集状态机 */
void WindSpeedCollect_StateMachine(void);

/* 协议帧打包 */
void Protocol_PackFrame(void);

/* 数据采集完成标志 */
extern bool env_collect_complete;
extern bool gps_collect_complete;
extern bool gy95t_collect_complete;
extern bool wind_speed_collect_complete;

/* 传感器数据存储 */
extern float g_t117_temperature;
extern WindSpeed_DataDef g_wind_speed_data;

#endif /* __SYSTEM_STATE_MACHINE_H */

