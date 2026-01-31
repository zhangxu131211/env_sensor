/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
typedef struct
{
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t date;
  uint8_t month;
  uint8_t year;
} RTC_Time_Date_t;

extern RTC_Time_Date_t g_rtc_time;
/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t RTC_Read_Time(void);
uint8_t RTC_Set_Time(RTC_Time_Date_t *time_date);
uint8_t RTC_Update_From_GPS(char *utc_time, char *utc_date);
void RTC_Print_Time(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

