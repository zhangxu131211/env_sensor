/* 系统状态机相关头文件 */

#ifndef __SYSTEM_STATE_MACHINE_H__
#define __SYSTEM_STATE_MACHINE_H__

#include "stdint.h"
#include "string.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "app_env_collect.h"
#include "stdbool.h"

/* 数据采集完成标志 */
extern bool env_collect_complete;   /* 环境数据采集完成标志 */
extern bool gps_collect_complete;   /* GPS数据采集完成标志 */
extern bool gy95t_collect_complete; /* GY95T数据采集完成标志 */

/* 函数声明 */
void System_StateMachine_Init(void);
void System_StateMachine_MainTask(void);
void EnvCollect_StateMachine(void);
void GPSCollect_StateMachine(void);
void GY95TCollect_StateMachine(void);

#endif /* __SYSTEM_STATE_MACHINE_H__ */

